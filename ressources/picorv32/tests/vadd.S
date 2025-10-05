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
  // = VADD.VX | sew larger than lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t0, x0, 0xf0
  vadd.vx v2, v1, t0
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x13
  slli x22, x22, 8
  addi x22, x22, 0x24
  slli x22, x22, 8
  addi x22, x22, 0x57
  slli x22, x22, 8
  addi x22, x22, 0x68
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x88
  slli x22, x22, 8
  addi x22, x22, 0x55
  slli x22, x22, 8
  addi x22, x22, 0x44
  slli x22, x22, 8
  addi x22, x22, 0x11
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xac
  slli x22, x22, 8
  addi x22, x22, 0xbd
  slli x22, x22, 8
  addi x22, x22, 0xac
  slli x22, x22, 8
  addi x22, x22, 0xbd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xbf
  slli x22, x22, 8
  addi x22, x22, 0xdf
  slli x22, x22, 8
  addi x22, x22, 0xbf
  slli x22, x22, 8
  addi x22, x22, 0xdf
  bne t0,x22,.trap
  nop
  
  // =================================================
  // = .VI
  // =================================================

  vsetvli t0, x0, e8, m1
  vadd.vi v2, v1, 0x09
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x1b
  slli x22, x22, 8
  addi x22, x22, 0x3d
  slli x22, x22, 8
  addi x22, x22, 0x5f
  slli x22, x22, 8
  addi x22, x22, 0x81
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x90
  slli x22, x22, 8
  addi x22, x22, 0x6e
  slli x22, x22, 8
  addi x22, x22, 0x4c
  slli x22, x22, 8
  addi x22, x22, 0x2a
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xb4
  slli x22, x22, 8
  addi x22, x22, 0xd6
  slli x22, x22, 8
  addi x22, x22, 0xb4
  slli x22, x22, 8
  addi x22, x22, 0xd6
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xc7
  slli x22, x22, 8
  addi x22, x22, 0xf8
  slli x22, x22, 8
  addi x22, x22, 0xc7
  slli x22, x22, 8
  addi x22, x22, 0xf8
  bne t0,x22,.trap
  nop

  // =================================================
  // = .VX
  // =================================================

  addi t0, x0, 0x19
  vadd.vx v2, v1, t0
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x2b
  slli x22, x22, 8
  addi x22, x22, 0x4d
  slli x22, x22, 8
  addi x22, x22, 0x6f
  slli x22, x22, 8
  addi x22, x22, 0x91
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0xa0
  slli x22, x22, 8
  addi x22, x22, 0x7e
  slli x22, x22, 8
  addi x22, x22, 0x5c
  slli x22, x22, 8
  addi x22, x22, 0x3a
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xc4
  slli x22, x22, 8
  addi x22, x22, 0xe6
  slli x22, x22, 8
  addi x22, x22, 0xc4
  slli x22, x22, 8
  addi x22, x22, 0xe6
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xd7
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0xd7
  slli x22, x22, 8
  addi x22, x22, 0x08
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xD708D708)
  
  // =================================================
  // = .VV
  // =================================================

  vadd.vv v2, v1, v3
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x16
  slli x22, x22, 8
  addi x22, x22, 0x37
  slli x22, x22, 8
  addi x22, x22, 0x58
  slli x22, x22, 8
  addi x22, x22, 0x79
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x8F
  slli x22, x22, 8
  addi x22, x22, 0x6C
  slli x22, x22, 8
  addi x22, x22, 0x49
  slli x22, x22, 8
  addi x22, x22, 0x26
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xB7
  slli x22, x22, 8
  addi x22, x22, 0xD8
  slli x22, x22, 8
  addi x22, x22, 0xB5
  slli x22, x22, 8
  addi x22, x22, 0xD6
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xBE
  slli x22, x22, 8
  addi x22, x22, 0xFE
  slli x22, x22, 8
  addi x22, x22, 0xCC
  slli x22, x22, 8
  addi x22, x22, 0xFC
  beq t0,x22,.start
  nop
  // TEST_CASE( 4, t0, 0xBEFECCFC)

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
