# See LICENSE for license details.

#*****************************************************************************
# vcpop.S
#-----------------------------------------------------------------------------
#
# Test vcpop.mm instruction
#

// #include "riscv_test.h"
// #include "test_macros.h"
 
/* RVTEST_RV32U
RVTEST_CODE_BEGIN */

.text

.start:
  nop
  vsetvli t0, x0, e8, m1
  
  la    x18, clear
  vle8.v v2, (x18)
  vle8.v v8, (x18)
  vle8.v v9, (x18)
  vle8.v v10, (x18)
  vle8.v v0, (x18)

  la    x18, tdat1
  la    x19, tdat7
  la    x20, result
  la    x21, mask1

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)

  vle8.v v0, (x21)

  // =================================================
  // = SEW = 8b | LMUL = 1/2
  // =================================================

  vsetvli t0, x0, e8, mf2
  
  vcpop.m t0, v1
  
  addi x22, x0, 0x04
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vcpop.m t0, v1
  
  addi x22, x0, 0x04
  bne t0,x22,.trap
  nop
  
  // =================================================
  // = SEW = 16b | masked
  // =================================================

  vcpop.m t0, v1, v0.t
  
  addi x22, x0, 0x00
  bne t0,x22,.trap

  // =================================================
  // = vl = 0
  // =================================================

  add t0, x0, x0
  vsetvli t0, t0, e16, m1
  addi t0, t0, 4
  vcpop.m t0, v1
  
  addi x22, x0, 0x00
  beq t0,x22,.start
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

mask1:  .word 0x05050505
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
         
mask2:  .word 0x50505050
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
