// hello_drc - affiche "HELLO WORLD" sur l'ecran du Wii U GamePad via libdrc.
#include <drc/streamer.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

using drc::u8;
using drc::byte;

static const int W = 854, H = 480;

// Police 5x7 pour les lettres necessaires a "HELLO WORLD".
struct Glyph { char c; uint8_t rows[7]; };
static const Glyph kFont[] = {
  {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
  {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
  {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
  {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
  {'W', {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}},
  {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
  {'D', {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}},
  {' ', {0,0,0,0,0,0,0}},
};

static const uint8_t* glyph(char c) {
  for (const auto& g : kFont) if (g.c == c) return g.rows;
  return kFont[7].rows; // espace
}

static void put(std::vector<byte>& fb, int x, int y, u8 r, u8 g, u8 b) {
  if (x < 0 || y < 0 || x >= W || y >= H) return;
  size_t o = 4 * (size_t(y) * W + x);
  fb[o] = r; fb[o+1] = g; fb[o+2] = b; fb[o+3] = 255;
}

static void draw_text(std::vector<byte>& fb, const char* s, int x0, int y0,
                      int scale, u8 r, u8 g, u8 b) {
  int x = x0;
  for (const char* p = s; *p; ++p) {
    const uint8_t* rows = glyph(*p);
    for (int ry = 0; ry < 7; ++ry)
      for (int rx = 0; rx < 5; ++rx)
        if (rows[ry] & (1 << (4 - rx)))
          for (int sy = 0; sy < scale; ++sy)
            for (int sx = 0; sx < scale; ++sx)
              put(fb, x + rx*scale + sx, y0 + ry*scale + sy, r, g, b);
    x += (5 + 1) * scale; // 1 colonne d'espace entre lettres
  }
}

int main() {
  drc::Streamer* streamer = new drc::Streamer();
  if (!streamer->Start()) {
    fprintf(stderr, "echec Streamer->Start()\n");
    return 1;
  }
  fprintf(stderr, "Streamer demarre, envoi de HELLO WORLD vers la GamePad...\n");

  const char* msg = "HELLO WORLD";
  int scale = 10;
  int text_w = (int)strlen(msg) * (5 + 1) * scale;
  int tx = (W - text_w) / 2, ty = (H - 7*scale) / 2;

  int frame = 0;
  for (;;) {
    std::vector<byte> fb(size_t(W) * H * 4);
    // fond bleu nuit qui pulse legerement pour prouver que c'est du live
    u8 bg = (u8)(20 + (frame % 60));
    for (size_t i = 0; i < fb.size(); i += 4) {
      fb[i] = 0; fb[i+1] = 0; fb[i+2] = bg; fb[i+3] = 255;
    }
    draw_text(fb, msg, tx, ty, scale, 255, 255, 255);
    streamer->PushVidFrame(&fb, W, H, drc::PixelFormat::kRGBA);
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
    ++frame;
  }
  return 0;
}
