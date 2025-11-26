# See LICENSE for license details.

#*****************************************************************************
# vmnor.S
#-----------------------------------------------------------------------------
#
# Test vmnor.mm instruction
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

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)

  // =================================================
  // = SEW = 8b | LMUL = 1/2 |
  // =================================================

  vsetvli t0, x0, e8, mf2
  
  vmnor.mm v0, v1, v1
  vsetvli t0, x0, e8, m1
  vse8.v v0, (x20)

  lw t0, (x20)
  addi x22, x0, 0xed
  slli x22, x22, 8
  addi x22, x22, 0xcb
  slli x22, x22, 8
  addi x22, x22, 0xa9
  slli x22, x22, 8
  addi x22, x22, 0x87
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x9a
  slli x22, x22, 8
  addi x22, x22, 0xbc
  slli x22, x22, 8
  addi x22, x22, 0xde
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x54
  slli x22, x22, 8
  addi x22, x22, 0x32
  slli x22, x22, 8
  addi x22, x22, 0x54
  slli x22, x22, 8
  addi x22, x22, 0x32
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
  slli x22, x22, 8
  addi x22, x22, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b
  // =================================================

  vsetvli t0, x0, e16, m1
  
  vmnor.mm v0, v1, v3
  vse16.v v0, (x20)

  lw t0, (x20)
  addi x22, x0, 0xe9
  slli x22, x22, 8
  addi x22, x22, 0xc8
  slli x22, x22, 8
  addi x22, x22, 0xa9
  slli x22, x22, 8
  addi x22, x22, 0x86
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x70
  slli x22, x22, 8
  addi x22, x22, 0x98
  slli x22, x22, 8
  addi x22, x22, 0xb8
  slli x22, x22, 8
  addi x22, x22, 0xda
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x50
  slli x22, x22, 8
  addi x22, x22, 0x30
  slli x22, x22, 8
  addi x22, x22, 0x54
  slli x22, x22, 8
  addi x22, x22, 0x32
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
  slli x22, x22, 8
  addi x22, x22, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b
  // =================================================

  vadd.vv v0, v31, v31
  vsetvli t0, x0, e32, m2
  vmnor.mm v0, v1, v3
  vsetvli t0, x0, e16, m1
  vse16.v v0, (x20) 

  lw t0, (x20)
  addi x22, x0, 0xe9
  slli x22, x22, 8
  addi x22, x22, 0xc8
  slli x22, x22, 8
  addi x22, x22, 0xa9
  slli x22, x22, 8
  addi x22, x22, 0x86
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x70
  slli x22, x22, 8
  addi x22, x22, 0x98
  slli x22, x22, 8
  addi x22, x22, 0xb8
  slli x22, x22, 8
  addi x22, x22, 0xda
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x50
  slli x22, x22, 8
  addi x22, x22, 0x30
  slli x22, x22, 8
  addi x22, x22, 0x54
  slli x22, x22, 8
  addi x22, x22, 0x32
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
  slli x22, x22, 8
  addi x22, x22, 0x41
  slli x22, x22, 8
  addi x22, x22, 0x10
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
