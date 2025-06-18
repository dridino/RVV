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

  vsetvli t0, x0, e32, m1
  la    x1, tdat1
  la    x2, tdat2

  vle32.v v1, (x1)
  vle32.v v3, (x2)
  
  vsetvli t0, x0, e8, m1
  vand.vv v5, v1, v3

  // to print vectors state
  vsetvli t0, x0, e32, mf4
  vle32.v v9, (x1)

  TEST_CASE( 1, x1, 12, \
    li  x1, 5; \
    li  x2, 7; \
    add x1, x1, x2; \
  )

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

RVTEST_DATA_END
