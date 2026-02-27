# See LICENSE for license details.

#*****************************************************************************
# vmsgtu.S
#-----------------------------------------------------------------------------
#
# Test vmsgtu instruction
#

// #include "riscv_test.h"
// #include "test_macros.h"
 
/* RVTEST_RV32U
RVTEST_CODE_BEGIN */

.text

.start:
  nop
  vsetvli t0, x0, e8, m1
  
  vmv.v.i v0, 0
  vmv.v.i v3, 0
  vmv.v.i v16, 0

  la    x18, tdat1
  la    x19, mask1
  la    x20, result

  vle8.v v1, (x18) // 2C
  // vle8.v v3, (x19)
  vle8.v v0, (x19)

  // =================================================
  // = SEW = 8b | LMUL = 1/2 | same
  // =================================================

  vsetvli t0, x0, e8, mf2
  
  vmsgtu.vv v16, v1, v1
  vsetvli t0, x0, e8, m1
  vse8.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 8b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e8, m1
  
  vle8.v v3, (x18), v0.t
  vmsgtu.vv v16, v3, v1
  vse8.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 8b | LMUL = 1 | diff | vx
  // =================================================

  vsetvli t0, x0, e8, m1
  
  vle8.v v3, (x18), v0.t
  addi t1, x0, 0x40
  vmsgtu.vx v16, v3, t1
  vse8.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x55
  slli x22, x22, 8
  addi x22, x22, 0x41
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmv.v.i v16, 0
  vmsgtu.vv v16, v3, v1
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmv.v.i v16, 0
  vle16.v v0, (x19)
  vle16.v v3, (x18), v0.t
  vmsgtu.vv v16, v3, v1
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmv.v.i v16, 0
  vmv.v.i v3, 0
  vle16.v v0, (x19)
  vle16.v v3, (x18), v0.t
  vmsgtu.vi v16, v3, 0xf
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x55
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 32b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e32, m1
  
  vmv.v.i v16, 0
  vmsgtu.vv v16, v3, v1
  vse32.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 2 | diff
  // =================================================

  vsetvli t0, x0, e16, m2
  
  vle16.v v1, (x18)
  vmv.v.i v3, 0

  vsetvli t0, x0, e8, m2
  vle8.v v3, (x18), v0.t
  vsetvli t0, x0, e16, m2
  
  vmv.v.i v16, 0
  vmsgtu.vv v16, v3, v1
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  // =================================================
  // = SEW = 16b | LMUL = 1 | masked
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmv.v.i v16, 0
  vmsgtu.vv v16, v1, v1, v0.t
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 1 | diff
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmv.v.i v16, 0
  vmv.v.i v3, 0
  la x19, mask2
  vle16.v v0, (x19)
  vsetvli t0, x0, e8, m1
  vle8.v v3, (x18), v0.t
  vsetvli t0, x0, e16, m1
  vmsgtu.vv v16, v3, v1
  vse16.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  bne  t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne  t0, x22, .trap

  j .start
  nop

.trap:
  la x23, err
  lw t0, (x23)
  nop
  
/* TEST_PASSFAIL 
 
RVTEST_CODE_END */

  .data
/* RVTEST_DATA_BEGIN

  TEST_DATA */

tdat:
tdat1:  .word 0x12345678
tdat2:  .word 0x87654321
tdat3:  .word 0xabcdabcd
tdat4:  .word 0xbeefbeef
tdat5:  .word 0xba0bab11
tdat6:  .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0
tdat7:  .word 0x04030201
        .word 0x08070605
        .word 0x0C0B0A09
        .word 0x000F0E0D

tdat8:  .word 0x00030001
        .word 0x08000600
        .word 0x00000000
        .word 0x000F0E0D

mask1:  .word 0x55555555
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
         
mask2:  .word 0xaaaaaaaa
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000

result: .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        
clear: .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
err:    .word 0xE0E0E0E0

//RVTEST_DATA_END
