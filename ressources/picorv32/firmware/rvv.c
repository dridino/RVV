// ajoute par Adrien

#include <riscv_vector.h>

#include "firmware.h"

void rvv(void) {
  int16_t a[8], b[8], c[8];

  // init
  int16_t acc = 0;
  for (int i = 0; i < 8; i++) {
    a[i] = i;
    acc += i;
    b[i] = acc;
  }

  // vector
  __asm__ volatile(
      "vsetvli t0, x0, e16, m1\n"
      "vle16.v v0, (%0)\n"
      "vle16.v v1, (%1)\n"
      "vadd.vv v2, v0, v1\n"
      "vse16.v v2, (%2)\n"
      :
      : "r"(a), "r"(b), "r"(c)
      : "t0", "v0", "v1", "v2");

  for (int i = 0; i < 8; i++) {
    print_str("(scalar, vector) : ");
    print_dec(a[i] + b[i]);
    print_str(" ");
    print_dec(c[i]);
    print_str("\n");
    if (c[i] != a[i] + b[i]) {
      print_str("ERROR !\n");
      __asm__ volatile("ebreak");
      return;
    }
  }

  print_str("RVV from C..OK\n");
}