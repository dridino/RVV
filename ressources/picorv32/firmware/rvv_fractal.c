// ajoute par Adrien

#include <riscv_vector.h>

#include "firmware.h"

#define WIDTH 128
#define LOG_W 7
#define HEIGHT 64
#define MAX_IT 255  // Fits directly into 1 byte

#define SHIFT_X 4
#define SHIFT_Y 5
#define LIMIT (1 << 20)

static inline int32_t iabs(int32_t v);

static inline int32_t iabs(int32_t v) {
  int32_t m = v >> 31;
  return (v ^ m) - m;
}

void rvv_fractal(void) {
  unsigned char image[WIDTH * HEIGHT];

  unsigned int num_cycles1, num_cycles2;
  __asm__ volatile("rdcycle %0;" : "=r"(num_cycles1));

  for (int py = 0; py < HEIGHT; py++) {
    for (int px = 0; px < WIDTH; px++) {
      // Map pixel to integer coordinate space
      int32_t x = (px - WIDTH / 2) << 14;
      int32_t y = (py - HEIGHT / 2) << 13;

      // Julia-style constants
      int32_t cx = x >> 2;
      int32_t cy = y >> 2;

      uint8_t iter = 0;
      uint8_t iter_val = 0;

      for (; iter < MAX_IT; iter++) {
        int32_t x_new = x + (y >> SHIFT_X) + cx;
        int32_t y_new = y - (x >> SHIFT_Y) + cy;

        x = x_new;
        y = y_new;

        iter_val = (iabs(x) | iabs(y)) > LIMIT ? iter_val : iter;

        // if ((iabs(x) | iabs(y)) > LIMIT) break;
      }
      image[(py << LOG_W) + px] = iter_val;
    }
  }

  __asm__ volatile("rdcycle %0;" : "=r"(num_cycles2));

  print_str("computation took : ");
  print_dec(num_cycles2 - num_cycles1);
  print_str(" cycles\n");

  // Write PGM header
  print_str("P5\n");
  print_dec(WIDTH);
  print_str(" ");
  print_dec(HEIGHT);
  print_str("\n255\n");

  for (int py = 0; py < HEIGHT; py++)
    for (int px = 0; px < WIDTH; px++) {
      print_dec(image[(py << LOG_W) + px]);
      if (px != WIDTH - 1)
        print_str(" ");
      else
        print_str("\n");
    }
  return;
}