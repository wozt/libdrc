// Copyright (c) 2013, Mema Hacking, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <algorithm>
#include <drc/screen.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <drc/internal/astrm-packet.h>
#include <drc/internal/h264-encoder.h>
#include <drc/internal/tsf.h>
#include <drc/internal/udp.h>
#include <algorithm>
#include <drc/internal/video-streamer.h>
#include <drc/internal/vstrm-packet.h>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace drc {

namespace {

s32 GetTimestamp() {
  // Keep the last good value: GetTsf fails while the MAC is powered down or
  // if the adapter goes away, and returning a stale timestamp is far better
  // than returning whatever happened to be on the stack.
  //
  // Read the counter itself rather than free-running on the monotonic clock
  // and slewing toward it: reading the TSF costs a USB transfer, so the
  // monotonic clock has to be anchored *around* that read, and getting that
  // wrong drags the timestamps into the past at a millisecond a second. Video
  // tolerates it; the GamePad drops audio that arrives late, and goes silent.
  static s32 last_timestamp = 0;
  u64 tsf;

  if (GetTsf(&tsf) == 0) {
    last_timestamp = static_cast<s32>(tsf & 0xFFFFFFFF);
  }
  return last_timestamp;
}

void GenerateVstrmPackets(std::vector<VstrmPacket>* vstrm_packets,
                          const H264ChunkArray& chunks, u32 timestamp,
                          bool idr, bool* vstrm_inited, u16* vstrm_seqid) {

  // Set the init flag on the first frame ever sent.
  bool init_flag = !*vstrm_inited;
  if (init_flag) {
    *vstrm_inited = true;
  }

  for (size_t i = 0; i < chunks.size(); ++i) {
    bool first_chunk = (i == 0);
    bool last_chunk = (i == chunks.size() - 1);

    const byte* chunk_data;
    size_t chunk_size;
    std::tie(chunk_data, chunk_size) = chunks[i];

    bool first_packet = true;
    do {
      const byte* pkt_data = chunk_data;
      size_t pkt_size = std::min(kMaxVstrmPayloadSize, chunk_size);

      chunk_data += pkt_size;
      chunk_size -= pkt_size;

      bool last_packet = (chunk_size == 0);

      vstrm_packets->resize(vstrm_packets->size() + 1);
      VstrmPacket* pkt = &vstrm_packets->at(vstrm_packets->size() - 1);

      u16 seqid = (*vstrm_seqid)++;
      if (*vstrm_seqid >= 1024) {
        *vstrm_seqid = 0;
      }
      pkt->SetSeqId(seqid);

      pkt->SetPayload(pkt_data, pkt_size);
      pkt->SetTimestamp(timestamp);

      pkt->SetInitFlag(init_flag);
      pkt->SetFrameBeginFlag(first_chunk && first_packet);
      pkt->SetChunkEndFlag(last_packet);
      pkt->SetFrameEndFlag(last_chunk && last_packet);

      pkt->SetIdrFlag(idr);
      pkt->SetFrameRate(VstrmFrameRate::k59_94Hz);  // TODO(delroth): setting?

      first_packet = false;
    } while (chunk_size != 0);
  }
}

void GenerateAstrmPacket(AstrmPacket* pkt, u32 ts) {
  pkt->ResetPacket();
  pkt->SetPacketType(AstrmPacketType::kVideoFormat);
  pkt->SetTimestamp(0x00100000);

  byte payload[24] = {
      0x00, 0x00, 0x00, 0x00,
      0x80, 0x3e, 0x00, 0x00,
      0x80, 0x3e, 0x00, 0x00,
      0x80, 0x3e, 0x00, 0x00,
      0x80, 0x3e, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
  };
  payload[0] = ts & 0xFF;
  payload[1] = (ts >> 8) & 0xFF;
  payload[2] = (ts >> 16) & 0xFF;
  payload[3] = (ts >> 24) & 0xFF;
  pkt->SetPayload(payload, sizeof (payload));
}

int GetEnvInt(const char* name, int default_value, int min_value,
              int max_value) {
  const char* value = getenv(name);
  if (!value) {
    return default_value;
  }

  int parsed = atoi(value);
  if (parsed < min_value) {
    return min_value;
  }
  if (parsed > max_value) {
    return max_value;
  }
  return parsed;
}

}  // namespace

VideoStreamer::VideoStreamer(const std::string& vid_dst,
                             const std::string& aud_dst)
    : astrm_client_(new UdpClient(aud_dst)),
      vstrm_client_(new UdpClient(vid_dst)),
      encoder_(new H264Encoder()),
      enc_idr_(false),
      resync_wanted_(false),
      reinit_wanted_(false),
      sweep_wanted_(false),
      enc_pending_(false),
      pre_encoded_(false),
      resync_evt_(NULL) {
}

VideoStreamer::~VideoStreamer() {
  Stop();
}

bool VideoStreamer::Start() {
  if (!astrm_client_->Start() || !vstrm_client_->Start()) {
    Stop();
    return false;
  }

  StartEM();
  return true;
}

void VideoStreamer::Stop() {
  StopEM();
  resync_evt_ = NULL;

  astrm_client_->Stop();
  vstrm_client_->Stop();
}

void VideoStreamer::PushFrame(std::vector<byte>* frame) {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  frame_ = std::move(*frame);
}

void VideoStreamer::PushEncodedFrame(const byte* data, const size_t* sizes,
                                     bool idr) {
  if (!data || !sizes) {
    return;
  }
  size_t total = 0;
  for (int i = 0; i < kH264ChunksPerFrame; ++i) {
    total += sizes[i];
  }
  if (total == 0) {
    return;
  }
  std::lock_guard<std::mutex> lk(enc_mutex_);
  // Copied rather than referenced: the sender's buffer is its encoder's
  // and is overwritten by its next frame, while this one is consumed by
  // another thread whenever it gets there.
  enc_frame_.assign(data, data + total);
  for (int i = 0; i < kH264ChunksPerFrame; ++i) {
    enc_sizes_[i] = sizes[i];
  }
  enc_idr_ = idr;
  enc_pending_ = true;
  pre_encoded_ = true;
}

void VideoStreamer::ReinitStream() {
  reinit_wanted_ = true;
}

void VideoStreamer::RequestSweep() {
  sweep_wanted_ = true;
}

bool VideoStreamer::TakeResyncRequest() {
  return resync_wanted_.exchange(false);
}

void VideoStreamer::ResyncStream() {
  if (resync_evt_) {
    resync_evt_->Trigger();
  }
}

void VideoStreamer::InitEventsAndRun() {
  bool resync_requested = false;
  int dbg_resync = 0;
  /* Sweeps asked for, and when the last one was: see the note beside
   * them for why this is not the resync interval. */
  int sweeps = 0;
  s32 last_sweep_ts = 0;
  bool last_sweep_valid = false;
  const int refresh_period = GetEnvInt("DRC_REFRESH", 30, 10, 240);
  resync_evt_ = NewTriggerableEvent([&](Event*) {
    resync_requested = true;
    // Also recorded for a caller that encodes elsewhere; harmless on
    // the encoding path, which never reads it.
    resync_wanted_ = true;
    dbg_resync++;
    return true;
  });

  std::vector<byte> encoding_frame;

  bool vstrm_inited = false;
  u16 vstrm_seqid = 0;
  // Horodatage du dernier IDR envoye, pour limiter les resyncs (voir plus bas).
  s32 last_idr_ts = 0;
  bool last_idr_valid = false;
  const int resync_interval_us = GetEnvInt("DRC_RESYNC_US", 500000, 0,
                                           5000000);
  const bool resync_restart = GetEnvInt("DRC_RESYNC_RESTART", 1, 0, 1) != 0;
  const int tx_spread_us = GetEnvInt("DRC_TX_SPREAD_US", 11000, 0, 16000);
  std::vector<VstrmPacket> vstrm_packets;
  bool send_idr = false;

  // Spread a frame's packets across the frame interval instead of emptying it
  // in one go. On a capture of a real console the median gap between two large
  // packets is 3.2 ms and only 6.5% leave back to back; libdrc dumps the whole
  // frame at once. That burst is what the link loses, and losing part of a
  // frame desynchronises the decoder.
  //
  // Pacing runs on its own thread: spreading over 11 ms of a 16.7 ms frame
  // inside the encoding thread starves the encoder and the rate collapses to
  // 30 fps. Here the previous frame goes out while the next one is encoded.
  std::mutex tx_mutex;
  std::condition_variable tx_cv;
  std::vector<VstrmPacket> tx_queue;
  bool tx_quit = false;

  std::thread tx_thread([&] {
    std::vector<VstrmPacket> batch;
    for (;;) {
      {
        std::unique_lock<std::mutex> lk(tx_mutex);
        tx_cv.wait(lk, [&] { return tx_quit || !tx_queue.empty(); });
        if (tx_quit) {
          return;
        }
        batch.swap(tx_queue);
        tx_queue.clear();
      }

      // Pacing is a local timing concern, so it runs off the monotonic clock:
      // spacing packets needs elapsed time, not the TSF, and reading the TSF
      // here would cost a USB transfer per packet.
      const size_t n = batch.size();
      const auto tx_start = std::chrono::steady_clock::now();
      for (size_t i = 0; i < n; ++i) {
        vstrm_client_->Send(batch[i].GetBytes(), batch[i].GetSize());
        if (tx_spread_us > 0 && i + 1 < n) {
          auto due = tx_start + std::chrono::microseconds(
              (i + 1) * (s64)tx_spread_us / n);
          std::this_thread::sleep_until(due);
        }
      }
      batch.clear();
    }
  });

  AstrmPacket astrm_packet;
  s32 next_timestamp = GetTimestamp();
  TimerEvent* timer_evt = NewTimerEvent(1, [&](Event*) {
    if (vstrm_inited) {
      astrm_client_->Send(astrm_packet.GetBytes(), astrm_packet.GetSize());
      if (!vstrm_packets.empty()) {
        {
          std::lock_guard<std::mutex> lk(tx_mutex);
          // The pacer is still on the previous frame: append rather than drop,
          // it catches up by shortening its own gaps.
          tx_queue.insert(tx_queue.end(),
                          std::make_move_iterator(vstrm_packets.begin()),
                          std::make_move_iterator(vstrm_packets.end()));
        }
        tx_cv.notify_one();
        vstrm_packets.clear();
      }
      if (send_idr && resync_requested) {
        resync_requested = false;
      }
    }

    s32 timestamp = GetTimestamp();

    // The already-encoded path, which is not a variation on the one
    // below but a shortcut past all of it: no latching a picture, no
    // encoder, no restart. Whoever produced these chunks decided what
    // they are, including whether this frame is a recovery point.
    if (pre_encoded_) {
      std::vector<byte> chunk_bytes;
      size_t chunk_sizes[kH264ChunksPerFrame];
      bool have = false;
      bool is_idr = false;
      {
        std::lock_guard<std::mutex> lk(enc_mutex_);
        if (enc_pending_) {
          chunk_bytes.swap(enc_frame_);
          for (int i = 0; i < kH264ChunksPerFrame; ++i) {
            chunk_sizes[i] = enc_sizes_[i];
          }
          is_idr = enc_idr_;
          enc_pending_ = false;
          have = true;
        }
      }
      if (have) {
        H264ChunkArray chunks;
        const byte* at = chunk_bytes.data();
        for (int i = 0; i < kH264ChunksPerFrame; ++i) {
          chunks[i] = std::make_tuple(at, chunk_sizes[i]);
          at += chunk_sizes[i];
        }
        // EVERY IDR re-initialises the stream, not just the first.
        //
        // On the encoding path a recovery point is produced by throwing
        // the encoder away, and that path rewinds vstrm_inited and the
        // sequence ids with it -- because a fresh encoder restarts its
        // frame numbering, and a packetiser still counting from before
        // hands the GamePad an IDR that does not line up with what it
        // is told. Here the encoder is in another process and does
        // exactly the same thing, so this has to follow.
        //
        // Doing it only for the first one is why the pad asked for a
        // keyframe sixty times a second for ever: every later recovery
        // point arrived mis-sequenced, it could not use any of them,
        // and it kept asking.
        // Asked for from outside: the next frame is the start of a
        // stream as far as the pad is concerned, which is the only way
        // to tell one that has lost the sequence to try again.
        if (reinit_wanted_.exchange(false)) {
          vstrm_inited = false;
        }
        if (is_idr && !vstrm_inited) {
          vstrm_seqid = 0;
        }
        GenerateVstrmPackets(&vstrm_packets, chunks, timestamp, is_idr,
                             &vstrm_inited, &vstrm_seqid);
        // The synchronisation packet, which is NOT optional and is easy
        // to leave out here because it has nothing to do with encoding.
        // The loop sends one to the astrm port every pass, from this
        // variable; a branch that never fills it in sends an empty
        // packet sixty times a second, and the GamePad -- which takes
        // its timebase from these -- shows nothing at all while the
        // video packets arrive perfectly.
        GenerateAstrmPacket(&astrm_packet, timestamp);
        vstrm_inited = true;
        if (getenv("DRC_STATS")) {
          // The same counters the encoding path prints, because this
          // path needs them more: there is no encoder here to blame, so
          // "packets are going out" and "packets are not going out" is
          // the whole diagnosis and nothing else in the process can say
          // which it is.
          static int dbg_frames = 0, dbg_idr = 0, dbg_pkts = 0;
          static s32 dbg_t0 = 0;
          dbg_frames++; if (is_idr) dbg_idr++;
          dbg_pkts += (int)vstrm_packets.size();
          if (dbg_t0 == 0) dbg_t0 = timestamp;
          if ((s32)(timestamp - dbg_t0) > 1000000) {
            fprintf(stderr, "[drc] pre-encoded: %d frames/s, %d IDR/s, %d pkts, "
                            "%d resync\n",
                    dbg_frames, dbg_idr, dbg_pkts, dbg_resync);
            dbg_frames = 0; dbg_idr = 0; dbg_pkts = 0; dbg_resync = 0;
            dbg_t0 = timestamp;
          }
        }
        if (resync_requested) {
          // Nothing here can answer it -- the encoder is elsewhere. The
          // sender is told through its own protocol and restarts its
          // encoder there; this only stops the request repeating.
          resync_requested = false;
        }
      }
    } else {

    LatchOnCurrentFrame(&encoding_frame);
    if (encoding_frame.size() > 0) {
      // The GamePad asks for a keyframe whenever it cannot decode. Answering
      // every request makes each frame an IDR, which on real video costs more
      // than it repairs, so requests are rate-limited to one per
      // resync_interval_us.
      //
      // Answering means restarting the encoder outright. Simply asking for an
      // IDR does nothing under intra refresh: x264 restarts its refresh wave
      // and never emits another NAL_SLICE_IDR, so a decoder that has lost the
      // sequence stays frozen forever while audio plays on. Restarting also
      // re-flags the stream as initialised and rewinds the sequence ids, which
      // is what the GamePad sees at the start of any stream.
      // Answering a resync request means restarting the encoder, because
      // asking for an IDR does nothing under intra refresh: x264 restarts its
      // refresh wave and never emits another NAL_SLICE_IDR. Without this the
      // GamePad's picture simply never comes back.
      //
      // It does not help a GamePad that has dropped the session rather than
      // lost the picture - 43 restarts over 21 seconds changed nothing there,
      // and only reassociating cleared it - but that is a different failure.
      send_idr = !vstrm_inited;
      // An explicit ReinitStream(), which is not the same thing as the
      // pad asking and must not be rate-limited like one.
      //
      // The caller asks for this when it has decided the pad is stuck,
      // having already waited seconds to be sure, and it needs the WHOLE
      // gesture rather than half of it: a fresh encoder, a rewound
      // sequence, an IDR, and the init flag on the packets that carry
      // it. Re-arming the flag alone -- which is all this used to do --
      // announces a new stream and then hands the pad a P-frame
      // predicting from pictures it does not have, which is exactly as
      // undecodable as what it was already stuck on.
      /*
       * An explicit ReinitStream(), doing what a restart does here.
       *
       * The init flag alone was tried and is worse than nothing: it
       * announces the start of a stream and then hands the pad a
       * P-frame predicting from pictures it does not have, so there is
       * nothing to start from and it stays stuck for good. A recovery
       * point is expensive -- an intra frame does not fit DRH's five
       * 1400-byte packets and gets split -- but a split frame it can
       * sometimes use beats a frame it certainly cannot.
       *
       * Not rate-limited like a resync request: the caller has already
       * waited seconds before asking.
       */
      /* The explicit ask, from a caller that has already waited seconds.
       * Same answer, for the same reason: a sweep the pad can decode
       * beats an IDR it cannot. */
      /*
       * The explicit ask, from a caller that has already waited seconds
       * and watched the sweeps fail to help. THIS one restarts: a pad
       * that has lost the sequence outright has nothing to sweep back
       * into, and an IDR it may not manage to read is better than
       * nothing it certainly cannot.
       */
      if (reinit_wanted_.exchange(false)) {
        encoder_->Restart();
        vstrm_inited = false;
        vstrm_seqid = 0;
        send_idr = true;
      }
      /*
       * A keyframe request is answered with a REFRESH, not a restart.
       *
       * Restarting produces an IDR, and an IDR is the one frame this
       * protocol cannot carry: five chunks, 1400 bytes each, and an
       * intra frame of real video is many times that. Measured, it is
       * not a poor answer but the cause of the failure -- the second an
       * IDR went out took 28 packets for one image and drew 48 more
       * requests, while every second without one sat at exactly 5
       * packets and zero. The pad asked, was answered with something it
       * could not read, and asked again.
       *
       * A sweep spreads the same intra macroblocks over the refresh
       * period. Nothing is bigger than the frames around it, the
       * picture repairs over half a second, and the loop never starts.
       * It is also what the console does: libdrc's own note is that
       * x264 under intra refresh never emits NAL_SLICE_IDR again after
       * the first frame, which is only a problem if you insist on one.
       */
      /*
       * A keyframe request is answered with a REFRESH SWEEP, not an IDR.
       *
       * Measured: the second an IDR goes out needs 28 packets for one
       * image where five are allowed, and draws 48 further requests;
       * the seconds around it sit at exactly 5 and zero. The pad asks,
       * is answered with the one frame this protocol cannot carry, and
       * asks again. A sweep spreads the same intra macroblocks over the
       * refresh period, so nothing is bigger than its neighbours.
       *
       * The FIRST frame of a stream is still an IDR and must be: a
       * sweep repairs a picture, it cannot provide one to a decoder
       * that has never had anything. That is `send_idr = !vstrm_inited`
       * above, and removing it is how this was tried once and left a
       * black panel.
       *
       * Spaced by twice the sweep, not by the resync interval. x264's
       * own note is that a refresh asked for while one is running only
       * begins when that one ends -- so asking every half second, which
       * is exactly how long a sweep takes, queues them nose to tail and
       * the picture never settles.
       */
      const s32 sweep_us = 2 * (refresh_period * 1000000 / 60);
      /* Asked for from outside, and spaced the same way: a caller that
       * has waited three seconds still must not queue sweeps nose to
       * tail, because one begun while another runs only starts when
       * that one ends. */
      const bool asked_sweep = sweep_wanted_.exchange(false);
      if ((asked_sweep || (resync_restart && resync_requested)) && vstrm_inited &&
          (!last_sweep_valid || (s32)(timestamp - last_sweep_ts) > sweep_us)) {
        encoder_->Refresh();
        last_sweep_ts = timestamp;
        last_sweep_valid = true;
        sweeps++;
      }
      if (send_idr) { last_idr_ts = timestamp; last_idr_valid = true; }
      const H264ChunkArray& chunks = encoder_->Encode(encoding_frame, send_idr);
      // Flag the packets from what the encoder actually produced: x264 emits
      // periodic IDRs of its own, and marking those as non-IDR leaves the
      // GamePad's decoder stuck forever after any reference loss.
      const bool frame_is_idr = encoder_->CurrentFrameIsIdr();
      GenerateVstrmPackets(&vstrm_packets, chunks, timestamp, frame_is_idr,
                           &vstrm_inited, &vstrm_seqid);
      if (getenv("DRC_STATS")) {
        // Per-second telemetry. "resync" is the count of keyframe requests
        // from the GamePad: a healthy stream sits at 0, and anything near the
        // frame rate means it cannot decode what we send.
        static int dbg_frames = 0, dbg_idr = 0, dbg_pkts = 0, dbg_max = 0;
        static s32 dbg_t0 = 0;
        dbg_frames++; if (frame_is_idr) dbg_idr++;
        dbg_pkts += (int)vstrm_packets.size();
        if ((int)vstrm_packets.size() > dbg_max) dbg_max = (int)vstrm_packets.size();
        if (dbg_t0 == 0) dbg_t0 = timestamp;
        if ((s32)(timestamp - dbg_t0) > 1000000) {
          time_t wall = time(NULL);
          char hhmmss[16];
          strftime(hhmmss, sizeof hhmmss, "%H:%M:%S", localtime(&wall));
          fprintf(stderr,
                  "[drc] %s %d frames/s, %d IDR/s, %d pkts, max %d/img, "
                  "%d resync, %d sweeps, spread=%dus\n",
                  hhmmss, dbg_frames, dbg_idr, dbg_pkts, dbg_max, dbg_resync,
                  sweeps, tx_spread_us);
          dbg_frames = 0; dbg_idr = 0; dbg_pkts = 0; dbg_resync = 0; dbg_max = 0;
          sweeps = 0;
          dbg_t0 = timestamp;
        }
      }
      GenerateAstrmPacket(&astrm_packet, timestamp);

      vstrm_inited = true;
      resync_requested = false;
    }
    }  /* the encoding path; the already-encoded one is above */

    timestamp = GetTimestamp();
    const s32 frame_interval = static_cast<s32>(1000000.0/59.94);
    next_timestamp += frame_interval;
    s32 delta_next = next_timestamp - timestamp;
    if (delta_next < 1000) {
      next_timestamp = timestamp + frame_interval;
      delta_next = frame_interval;
    }
    timer_evt->RearmTimer((u64)delta_next * 1000);

    return true;
  });

  ThreadedEventMachine::InitEventsAndRun();

  {
    std::lock_guard<std::mutex> lk(tx_mutex);
    tx_quit = true;
  }
  tx_cv.notify_all();
  tx_thread.join();
}

void VideoStreamer::LatchOnCurrentFrame(std::vector<byte>* latched_frame) {
  std::lock_guard<std::mutex> lk(frame_mutex_);
  if (frame_.size() != 0) {
    *latched_frame = std::move(frame_);
  }
}

}  // namespace drc
