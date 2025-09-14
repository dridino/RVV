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
  la    x18, tdat1
  la    x19, tdat7
  la    x20, result

  vsetvli t0, x0, e8, m1

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)
  vadd.vv v2, v1, v3
  vse8.v v2, (x20)


  /* addi x0,x0,0
  vle32.v v1, (x18)
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

  lw x21, (x20)
  addi x22, x0, 0x16
  slli x22, x22, 8
  addi x22, x22, 0x37
  slli x22, x22, 8
  addi x22, x22, 0x58
  slli x22, x22, 8
  addi x22, x22, 0x79
  bne x21,x22,.trap
  nop
  
  lw x21, 4(x20)
  addi x22, x0, 0x8F
  slli x22, x22, 8
  addi x22, x22, 0x6C
  slli x22, x22, 8
  addi x22, x22, 0x49
  slli x22, x22, 8
  addi x22, x22, 0x26
  beq x21,x22,.start
  nop
  
  lw x21, 8(x20)
  addi x22, x0, 0xB7
  slli x22, x22, 8
  addi x22, x22, 0xD8
  slli x22, x22, 8
  addi x22, x22, 0xB5
  slli x22, x22, 8
  addi x22, x22, 0xD6
  bne x21,x22,.trap
  nop
  
  lw x21, 12(x20)
  addi x22, x0, 0xBE
  slli x22, x22, 8
  addi x22, x22, 0xFE
  slli x22, x22, 8
  addi x22, x22, 0xCC
  slli x22, x22, 8
  addi x22, x22, 0xFC
  beq x21,x22,.start
  nop

.trap:
  la x23, err
  lw x21, (x23)
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
