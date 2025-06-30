# See LICENSE for license details.

#*****************************************************************************
# vand.S
#-----------------------------------------------------------------------------
#
# Test vand instruction.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV32U
RVTEST_CODE_BEGIN
  addi x8, x0, 7
  vsetvli t0, x0, e32, m2
  addi x8, x0, 15
  li x9, 15
  la    x1, tdat1
  la    x2, tdat2

  addi x0,x0,0
  vle32.v v1, (x1)
  addi x0,x0,1
  vle32.v v3, (x2)
  
  li x4, 11
  vsetvli t0, x0, e32, m1
  // vand.vv v5, v1, v3
  // vand.vv v6, v2, v4

  // to print vectors state
  vsetvli t0, x0, e64, mf4
  // vle32.v v8, (x1)

  TEST_CASE( 1, x1, 0, \
    li  x1, 5; \
    li  x2, 7; \
    add x1, x1, x2; \
  )

  addi x8,x0,10

  TEST_PASSFAIL 

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

tdat:
tdat1:  .word 0x12345678
tdat2:  .word 0x87654321
tdat3:  .word 0xabcdabcd
tdat4:  .word 0xbeefbeef
tdat5:  .word 0xba0bab11
        .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0
        .word 0xbeefbeef
        .word 0xc1a0c1a0

RVTEST_DATA_END
