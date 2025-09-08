# See LICENSE for license details.

#*****************************************************************************
# vand.S
#-----------------------------------------------------------------------------
#
# Test vand instruction.
#

// #include "riscv_test.h"
// #include "test_macros.h"
/* 
RVTEST_RV32U
RVTEST_CODE_BEGIN */
.text

.start:
  la    x1, tdat1
  la    x2, tdat7
  la    x3, result

  vsetvli t0, x0, e8, m1
  
  vle8.v v1, (x1)
  vle8.v v2, (x2)

  vadd.vv v3, v1, v2

  vse8.v v3, (x3)


  /* addi x0,x0,0
  vle32.v v1, (x1)
  vle32.v v3, (x2)
  
  vsetvli t0, x0, e32, m1
  vadd.vv v5, v1, v3
  vadd.vv v6, v2, v4

  vsetvli t0, x0, e32, m2
  vse32.v v5, (x3)
  vsetvli t0, x0, e32, m1
  vle32.v v1, (x3)
  vle32.v v3, (x4) */

  /* TEST_CASE( 1, x1, 12, \
    li  x1, 5; \
    li  x2, 7; \
    add x1, x1, x2; \
  ) */

  lw x8, (x3)
  addi x9, x0, 0x16
  slli x9, x9, 8
  addi x9, x9, 0x37
  slli x9, x9, 8
  addi x9, x9, 0x58
  slli x9, x9, 8
  addi x9, x9, 0x79
  bne x8,x9,.trap
  nop
  
  lw x8, 4(x3)
  addi x9, x0, 0x8F
  slli x9, x9, 8
  addi x9, x9, 0x6C
  slli x9, x9, 8
  addi x9, x9, 0x49
  slli x9, x9, 8
  addi x9, x9, 0x26
  bne x8,x9,.trap
  nop
  
  lw x8, 8(x3)
  addi x9, x0, 0xB7
  slli x9, x9, 8
  addi x9, x9, 0xD8
  slli x9, x9, 8
  addi x9, x9, 0xB5
  slli x9, x9, 8
  addi x9, x9, 0xD6
  bne x8,x9,.trap
  nop
  
  lw x8, 12(x3)
  addi x9, x0, 0xBE
  slli x9, x9, 8
  addi x9, x9, 0xFE
  slli x9, x9, 8
  addi x9, x9, 0xCC
  slli x9, x9, 8
  addi x9, x9, 0xFC
  beq x8,x9,.start
  nop

.trap:
  la x3, err
  lw x8, (x3)
  nop
  // TEST_PASSFAIL 
/* 
RVTEST_CODE_END
 */
  .data
/* RVTEST_DATA_BEGIN

  TEST_DATA
 */
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
// RVTEST_DATA_END
