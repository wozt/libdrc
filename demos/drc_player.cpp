// drc_player - video (stdin RGBA 864x480) + audio (FIFO s16 48kHz stereo)
// vers le Wii U GamePad. Cadence deleguee a ffmpeg -r 60.
#include <drc/streamer.h>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>

using drc::byte;
using drc::s16;
static const int W = 864, H = 480;

static bool read_full(int fd, uint8_t* buf, size_t n) {
  size_t got = 0;
  while (got < n) {
    ssize_t r = read(fd, buf + got, n - got);
    if (r <= 0) return false;
    got += (size_t)r;
  }
  return true;
}

// Thread audio : lecture BLOQUANTE d'un chunk a la fois. Cela se cadence
// naturellement sur le rythme temps-reel de ffmpeg (-re) et correspond
// exactement a ce que consomme libdrc (384 samples toutes les 8 ms).
// Drainer plus vite ferait gonfler la file interne jusqu'a son plafond de
// 16 s, apres quoi PushSamples jette tout en silence : le son "s'arrete".
static void audio_thread(drc::Streamer* s, const char* fifo) {
  int fd = open(fifo, O_RDONLY);
  if (fd < 0) { perror("open audio fifo"); return; }
  const size_t N = 384;                  // 384 samples stereo = 8 ms
  std::vector<s16> buf(N * 2);
  long chunks = 0;
  auto last = std::chrono::steady_clock::now();
  for (;;) {
    if (!read_full(fd, (uint8_t*)buf.data(), buf.size() * sizeof(s16))) {
      close(fd);
      fd = open(fifo, O_RDONLY);         // writer parti : on reattend
      if (fd < 0) return;
      continue;
    }
    s->PushAudSamples(buf);
    chunks++;
    auto now = std::chrono::steady_clock::now();
    if (now - last >= std::chrono::seconds(1)) {
      fprintf(stderr, "[drc-audio] %ld chunks lus/s\n", chunks);
      chunks = 0;
      last = now;
    }
  }
}

int main() {
  drc::Streamer* s = new drc::Streamer();
  if (!s->Start()) { fprintf(stderr, "Streamer Start failed\n"); return 1; }
  fprintf(stderr, "drc_player: %dx%d video + audio vers la GamePad\n", W, H);

  const char* afifo = getenv("DRC_AUDIO_FIFO");
  std::thread at;
  if (afifo) at = std::thread(audio_thread, s, afifo);

  const size_t fsz = (size_t)W * H * 4;
  long n = 0;
  for (;;) {
    std::vector<byte> frame(fsz);
    if (!read_full(0, (uint8_t*)frame.data(), fsz)) {
      fprintf(stderr, "fin video (%ld frames)\n", n); break;
    }
    s->PushVidFrame(&frame, W, H, drc::PixelFormat::kRGBA);
    n++;
  }
  s->Stop();
  return 0;
}
