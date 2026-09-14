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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <drc/internal/astrm-packet.h>
#include <drc/internal/h264-encoder.h>
#include <drc/internal/tsf.h>
#include <drc/internal/udp.h>
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

void VideoStreamer::ResyncStream() {
  if (resync_evt_) {
    resync_evt_->Trigger();
  }
}

void VideoStreamer::InitEventsAndRun() {
  bool resync_requested = false;
  int dbg_resync = 0;
  resync_evt_ = NewTriggerableEvent([&](Event*) {
    resync_requested = true;
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
    LatchOnCurrentFrame(&encoding_frame);
    if (encoding_frame.size() > 0) {
      // Only the caller asking for it, via Streamer::ResyncStream(), forces a
      // keyframe now - and never more than one per resync_interval_us, since
      // a keyframe is ~25 packets where an ordinary frame is 5.
      send_idr = !vstrm_inited;
      if (resync_requested && vstrm_inited &&
          (!last_idr_valid ||
           (s32)(timestamp - last_idr_ts) > resync_interval_us)) {
        send_idr = true;
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
        static int dbg_frames = 0, dbg_idr = 0, dbg_pkts = 0;
        static s32 dbg_t0 = 0;
        dbg_frames++; if (frame_is_idr) dbg_idr++;
        dbg_pkts += (int)vstrm_packets.size();
        if (dbg_t0 == 0) dbg_t0 = timestamp;
        if ((s32)(timestamp - dbg_t0) > 1000000) {
          fprintf(stderr,
                  "[drc] %d frames/s, %d IDR/s, %d pkts, %d resync, "
                  "spread=%dus, dt=%d us\n",
                  dbg_frames, dbg_idr, dbg_pkts, dbg_resync, tx_spread_us,
                  (int)(timestamp - dbg_t0));
          dbg_frames = 0; dbg_idr = 0; dbg_pkts = 0; dbg_resync = 0;
          dbg_t0 = timestamp;
        }
      }
      GenerateAstrmPacket(&astrm_packet, timestamp);

      vstrm_inited = true;
      resync_requested = false;
    }

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
