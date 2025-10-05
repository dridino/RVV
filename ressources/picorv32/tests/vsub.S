# See LICENSE for license details.

#*****************************************************************************
# vand.S
#-----------------------------------------------------------------------------
#
# Test vand instruction.
#

// #include "riscv_test.h"
// #include "test_macros.h"
 
/* RVTEST_RV32U
RVTEST_CODE_BEGIN */

.text

.start:
  nop
  la    x18, tdat1
  la    x19, tdat7
  la    x20, result

  vsetvli t0, x0, e8, m1

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)

  // =================================================
  // = .VX
  // =================================================

  addi t0, x0, 0x09
  vsub.vx v2, v1, t0
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x09
  slli x22, x22, 8
  addi x22, x22, 0x2b
  slli x22, x22, 8
  addi x22, x22, 0x4d
  slli x22, x22, 8
  addi x22, x22, 0x6f
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x7e
  slli x22, x22, 8
  addi x22, x22, 0x5c
  slli x22, x22, 8
  addi x22, x22, 0x3a
  slli x22, x22, 8
  addi x22, x22, 0x18
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xa2
  slli x22, x22, 8
  addi x22, x22, 0xc4
  slli x22, x22, 8
  addi x22, x22, 0xa2
  slli x22, x22, 8
  addi x22, x22, 0xc4
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xb5
  slli x22, x22, 8
  addi x22, x22, 0xe6
  slli x22, x22, 8
  addi x22, x22, 0xb5
  slli x22, x22, 8
  addi x22, x22, 0xe6
  bne t0,x22,.trap
  nop

  // =================================================
  // = .VX
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t1, x0, 0x09
  vsub.vx v2, v1, t1
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x12
  slli x22, x22, 8
  addi x22, x22, 0x2b
  slli x22, x22, 8
  addi x22, x22, 0x56
  slli x22, x22, 8
  addi x22, x22, 0x6f
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x87
  slli x22, x22, 8
  addi x22, x22, 0x5c
  slli x22, x22, 8
  addi x22, x22, 0x43
  slli x22, x22, 8
  addi x22, x22, 0x18
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xab
  slli x22, x22, 8
  addi x22, x22, 0xc4
  slli x22, x22, 8
  addi x22, x22, 0xab
  slli x22, x22, 8
  addi x22, x22, 0xc4
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xbe
  slli x22, x22, 8
  addi x22, x22, 0xe6
  slli x22, x22, 8
  addi x22, x22, 0xbe
  slli x22, x22, 8
  addi x22, x22, 0xe6
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


result: .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
err:    .word 0xE0E0E0E0

//RVTEST_DATA_END
