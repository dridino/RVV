# See LICENSE for license details.

#*****************************************************************************
# vload_vstore.S
#-----------------------------------------------------------------------------
#
# Test vload / vstore instructions.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV32U
RVTEST_CODE_BEGIN

  vsetvli t0, x0, e32, m1
  la      x1, index1
  la      x3, tdat12
  la      x6, tdat1

  li         x2, 5

  // vle8.v    v1, (x1)
  // vse8.v    v1, (x5)
  // vle64.v     v3, (x5)
  // vsse64.v    v3, (x5), x2

  // vle64.v    v1, (x5)
  // vsse64.v    v1, (x5), x2
  // vlse64.v    v3, (x5), x2

  // vle8.v      v31, (x1) // load indices
  // vle16.v     v3, (x6) // load data
  // vsuxei8.v   v3, (x6), v31 // store indexed
  // vluxei8.v   v5, (x6), v31 // indexed load

  /* vle32.v       v1, (x6)
  vle32.v       v2, (x3)

  vle8.v        v10, (x1)

  vsuxseg2ei8.v    v1, (x3), v10
  vluxseg2ei8.v    v4, (x3), v10 */

  vl8re64.v v1, (x6)
  vs8r.v v1, (x3)
  vl8re64.v v1, (x3)

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
tdat5:  .word 0xba0bab01
tdat6:  .word 0x22222222
tdat7:  .word 0x33333333
tdat8:  .word 0x44444444
tdat9:  .word 0xbeefbeef
tdat11:  .word 0x55555555
tdat12:  .word 0x66666666
tdat13:  .word 0x77777777
tdat14:  .word 0x99999999
tdat15:  .word 0x10101010
tdat16:  .word 0x12121212
tdat17:  .word 0x44444444
tdat18:  .word 0xa0000001
tdat19:  .word 0xa0000002
tdat20:  .word 0xa0000003
tdat21:  .word 0xa0000004
tdat22:  .word 0xa0000005
tdat23:  .word 0xa0000006
tdat24:  .word 0xa0000007
tdat25:  .word 0xa0000008
tdat26:  .word 0xa0000009
tdat27:  .word 0xa000000A
tdat28:  .word 0xa000000B
tdat29:  .word 0xa000000C
tdat30:  .word 0xa000000D
tdat31:  .word 0xa000000E
tdat32:  .word 0xa000000F
tdat33:  .word 0xa0000010

index1:   .word 0x04000400
index2:   .word 0x04000400
index3:   .word 0x0B0A0908
index4:   .word 0x0F0E0D0C

seg1:     .word 0x11221122
seg2:     .word 0x11221122
seg3:     .word 0x11221122
seg4:     .word 0x11221122
seg5:     .word 0x11221122
seg6:     .word 0x11221122
seg7:     .word 0x11221122
seg8:     .word 0x11221122
RVTEST_DATA_END
