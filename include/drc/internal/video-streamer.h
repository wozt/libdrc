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

#pragma once

#include <drc/internal/events.h>
#include <drc/internal/h264-encoder.h>
#include <drc/types.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace drc {

class H264Encoder;
class UdpClient;

class VideoStreamer : public ThreadedEventMachine {
 public:
  // VideoStreamer needs to send synchronization packets to the astrm port,
  // hence the aud_dst parameter.
  VideoStreamer(const std::string& vid_dst, const std::string& aud_dst);
  virtual ~VideoStreamer();

  bool Start();
  void Stop();

  // Needs YUV420P at the right size (kScreenWidth x kScreenHeight).
  void PushFrame(std::vector<byte>* frame);

  // Hands over a frame that has ALREADY been encoded in the pad's own
  // format, skipping this streamer's encoder entirely.
  //
  // Why this exists: everything upstream of here is a second lossy pass.
  // A sender that can produce DRH-sliced H.264 itself -- same settings,
  // same five chunks, same QP 32 -- has no reason to have its picture
  // decoded and re-encoded on the way through. The packetiser never
  // cared where the chunks came from.
  //
  // `data` is the five chunks packed one after another and `sizes` says
  // how long each is; both are copied, so the caller's buffers are free
  // immediately. `idr` must say what the frame ACTUALLY is, not what was
  // asked for -- the pad only treats a chunk as a recovery point when
  // the vstrm IDR flag is set.
  //
  // The encoder is not constructed while this is in use, so a caller on
  // this path does not need drc-x264 present at all.
  void PushEncodedFrame(const byte* data, const size_t* sizes, bool idr);

  // Require an IDR to be sent next.
  void ResyncStream();

  // Whether the pad has asked for a recovery point since this was last
  // called, and clears the asking.
  //
  // On the encoding path this streamer answers such a request itself.
  // On the pre-encoded one it CANNOT -- the encoder is in another
  // process -- and swallowing the request silently is worse than
  // useless: the pad asks on every frame it cannot decode, sixty times
  // a second, for ever, and the picture never starts. This is how that
  // request reaches whoever can act on it.
  bool TakeResyncRequest();

  // Tells the pad to re-initialise its decoder from the next frame.
  //
  // A vstrm packet carries an "init" flag, and it is the only thing in
  // this protocol that says "forget what you were doing and start
  // again". libdrc sets it on the first frame of a stream and never
  // afterwards -- which is why a pad that has lost the sequence stays
  // lost: everything it is sent is a continuation of a stream it can no
  // longer follow.
  //
  // Calling this re-arms that flag and rewinds the sequence ids, so the
  // next frame looks like the beginning of a stream. It is only half of
  // the answer: that frame also has to be an IDR, or there is nothing
  // for the decoder to start from.
  void ReinitStream();

  // Asks for a refresh sweep: the gentle answer, and the one that fits.
  // ReinitStream() above is the heavy one -- it restarts the encoder and
  // so produces an IDR, which is the frame this protocol cannot carry.
  // A caller escalating should try this first and reach for that only
  // when sweeps have not helped.
  void RequestSweep();

 protected:
  virtual void InitEventsAndRun();

 private:
  void LatchOnCurrentFrame(std::vector<byte>* latched_frame);

  std::unique_ptr<UdpClient> astrm_client_;
  std::unique_ptr<UdpClient> vstrm_client_;

  std::unique_ptr<H264Encoder> encoder_;

  std::mutex frame_mutex_;
  std::vector<byte> frame_;

  // The already-encoded path. Separate from frame_ on purpose: the two
  // are different things and a streamer only ever uses one of them.
  std::mutex enc_mutex_;
  std::vector<byte> enc_frame_;
  size_t enc_sizes_[kH264ChunksPerFrame];
  bool enc_idr_;
  std::atomic<bool> resync_wanted_;
  std::atomic<bool> reinit_wanted_;
  std::atomic<bool> sweep_wanted_;
  bool enc_pending_;
  // Latched once, by whichever entry point is used first: a stream that
  // changed halfway would hand the pad two different encoders' output
  // with one sequence of packet ids across both.
  bool pre_encoded_;

  TriggerableEvent* resync_evt_;
};

}  // namespace drc
