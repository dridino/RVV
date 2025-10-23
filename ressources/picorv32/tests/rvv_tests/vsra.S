# See LICENSE for license details.

#*****************************************************************************
# vsra.S
#-----------------------------------------------------------------------------
#
# Test vsra instruction.
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
  // = VSRA.VX | sew = lane width
  // =================================================

  addi t0, x0, 0x4
  vsra.vx v2, v1, t0
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x01
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x05
  slli x22, x22, 8
  addi x22, x22, 0x07
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0xf8
  slli x22, x22, 8
  addi x22, x22, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x02
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xfa
  slli x22, x22, 8
  addi x22, x22, 0xfc
  slli x22, x22, 8
  addi x22, x22, 0xfa
  slli x22, x22, 8
  addi x22, x22, 0xfc
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xfb
  slli x22, x22, 8
  addi x22, x22, 0xfe
  slli x22, x22, 8
  addi x22, x22, 0xfb
  slli x22, x22, 8
  addi x22, x22, 0xfe
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSRA.VX | sew larger than lane width | sham = lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t0, x0, 0x8
  vsra.vx v2, v1, t0
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x12
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x56
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0x87
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x43
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xab
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xab
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xbe
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xbe
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSRA.VX | sew larger than lane width | sham larger than lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  addi t0, x0, 0xb
  vsra.vx v2, v1, t0
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x0a
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf0
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x08
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf5
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf5
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf7
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf7
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSRA.VI | sew larger than lane width | sham larger than lane width
  // =================================================

  vsetvli t0, x0, e16, m1
  vsra.vi v2, v1, 0xb
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x0a
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf0
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x08
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf5
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf5
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf7
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf7
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSRA.VI | sew larger than lane width | sham larger than lane width
  // =================================================

  vsetvli t0, x0, e32, m1
  vsra.vi v2, v1, 0x1b
  vse32.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x02
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf0
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf5
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xf7
  bne t0,x22,.trap
  nop

  // =================================================
  // = VSRA.VV | sew = lane width
  // =================================================

  vsetvli t0, x0, e8, m1
  vsra.vv v2, v1, v3
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x01
  slli x22, x22, 8
  addi x22, x22, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x15
  slli x22, x22, 8
  addi x22, x22, 0x3c
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x16375879)
  
  lw t0, 4(x20)
  addi x22, x0, 0x87
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x01
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x8F6C4926)
  
  lw t0, 8(x20)
  addi x22, x0, 0xfa
  slli x22, x22, 8
  addi x22, x22, 0xf9
  slli x22, x22, 8
  addi x22, x22, 0xea
  slli x22, x22, 8
  addi x22, x22, 0xe6
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xB7D8B5D6)
  
  lw t0, 12(x20)
  addi x22, x0, 0xbe
  slli x22, x22, 8
  addi x22, x22, 0xff
  slli x22, x22, 8
  addi x22, x22, 0xfe
  slli x22, x22, 8
  addi x22, x22, 0xff
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
