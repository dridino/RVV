# See LICENSE for license details.

#*****************************************************************************
# vmvnr.S
#-----------------------------------------------------------------------------
#
# Test vmvnr instruction
#

// #include "riscv_test.h"
// #include "test_macros.h"
 
/* RVTEST_RV32U
RVTEST_CODE_BEGIN */

.text

.start:
  nop
  vsetvli t0, x0, e32, m8
  
  vmv.v.i v0, 0
  vmv.v.i v3, 0
  vmv.v.i v16, 0

  la    x18, tdat1
  la    x19, mask1
  la    x20, result

  vle32.v v1, (x18) // 2C
  // vle8.v v3, (x19)
  // vle8.v v0, (x19)

  // =================================================
  // = SEW = 8b | LMUL = 1/2 | 1r
  // =================================================

  vsetvli t0, x0, e8, mf2

  vmv1r.v v9, v1

  vsetvli t0, x0, e8, m1
  vmseq.vv v20, v1, v9
  vse8.v v20, (x20)

  lw t0, (x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  bne t0, x22, .trap

  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap

  // =================================================
  // = SEW = 32b | LMUL = 8 | 2r
  // =================================================

  vsetvli t0, x0, e32, m8

  vmv2r.v v9, v1

  vsetvli t0, x0, e32, m2
  vmv.v.i v20, 0
  vmseq.vv v20, v1, v9
  vsetvli t0, x0, e32, m1
  vse32.v v20, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xff
  bne t0, x22, .trap

  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap

  // =================================================
  // = SEW = 16b | LMUL = 4 | 4r
  // =================================================

  vsetvli t0, x0, e16, m4

  vmv4r.v v9, v1

  vsetvli t0, x0, e32, m4
  vmv.v.i v20, 0
  vmseq.vv v20, v1, v9
  vsetvli t0, x0, e32, m1
  vse32.v v20, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xff
  bne t0, x22, .trap

  lw t0, 4(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 8(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap
  
  lw t0, 12(x20)
  addi x22, x0, 0x0
  bne t0, x22, .trap

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
