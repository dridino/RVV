# See LICENSE for license details.

#*****************************************************************************
# vsll.S
#-----------------------------------------------------------------------------
#
# Test vmax instruction.
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
  // = VSLL.VX | sew = lane width
  // =================================================

  addi t0, x0, 0x4
  vsll.vx v2, v1, t0
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x20
  slli x22, x22, 8
  addi x22, x22, 0x40
  slli x22, x22, 8
  addi x22, x22, 0x60
  slli x22, x22, 8
  addi x22, x22, 0x80
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x70
  slli x22, x22, 8
  addi x22, x22, 0x50
  slli x22, x22, 8
  addi x22, x22, 0x30
  slli x22, x22, 8
  addi x22, x22, 0x10
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xb0
  slli x22, x22, 8
  addi x22, x22, 0xd0
  slli x22, x22, 8
  addi x22, x22, 0xb0
  slli x22, x22, 8
  addi x22, x22, 0xd0
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xe0
  slli x22, x22, 8
  addi x22, x22, 0xf0
  slli x22, x22, 8
  addi x22, x22, 0xe0
  slli x22, x22, 8
  addi x22, x22, 0xf0
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSLL.VX | sew larger than lane width | sham = lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t0, x0, 0x8
  vsll.vx v2, v1, t0
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x65
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x21
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xef
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xef
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSLL.VX | sew larger than lane width | sham larger than lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t0, x0, 0xb
  vsll.vx v2, v1, t0
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0xa0
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xc0
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x28
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x68
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x68
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSLL.VI | sew larger than lane width | sham larger than lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  vsll.vi v2, v1, 0xb
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0xa0
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xc0
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x28
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x68
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x68
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSLL.VV | sew = lane width
  // =================================================

  vsetvli t0, x0, e8, m1
  vsll.vv v2, v1, v3
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x20
  slli x22, x22, 8
  addi x22, x22, 0xa0
  slli x22, x22, 8
  addi x22, x22, 0x58
  slli x22, x22, 8
  addi x22, x22, 0xf0
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x87
  slli x22, x22, 8
  addi x22, x22, 0x80
  slli x22, x22, 8
  addi x22, x22, 0xc0
  slli x22, x22, 8
  addi x22, x22, 0x20
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xb0
  slli x22, x22, 8
  addi x22, x22, 0x68
  slli x22, x22, 8
  addi x22, x22, 0xac
  slli x22, x22, 8
  addi x22, x22, 0x9a
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xbe
  slli x22, x22, 8
  addi x22, x22, 0x80
  slli x22, x22, 8
  addi x22, x22, 0x80
  slli x22, x22, 8
  addi x22, x22, 0xe0
  beq t0,x22,.trap
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
