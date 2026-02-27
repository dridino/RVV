# See LICENSE for license details.

#*****************************************************************************
# vmask.S
#-----------------------------------------------------------------------------
#
# Test masking.
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

  la    x18, tdat1
  la    x19, tdat7
  la    x20, result
  la    x21, mask1

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)

  // =================================================
  // = SEW = 16b | MASK : 0x05050505
  // =================================================

  vsetvli t0, x0, e16, m1
  vl1re16.v v0, (x21)
  
  vand.vv v2, v1, v1, v0.t // mask v1 with v0.t
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x56
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x43
  slli x22, x22, 8
  addi x22, x22, 0x21
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | MASK : 0x05050505
  // =================================================

  vsetvli t0, x0, e8, m1
  vl1re8.v v0, (x21)

  vand.vv v8, v1, v1, v0.t // mask v1 with v0.t
  vse8.v v8, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | MASK : 0x05050505 | load
  // =================================================

  vsetvli t0, x0, e8, m1
  vl1re8.v v0, (x21)

  vle8.v v9, (x18), v0.t
  vse8.v v9, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b | MASK : 0x05050505 | load
  // =================================================

  vsetvli t0, x0, e16, m1
  vl1re16.v v0, (x21)

  vle16.v v10, (x18), v0.t
  vse16.v v10, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x56
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x43
  slli x22, x22, 8
  addi x22, x22, 0x21
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | MASK : 0x05050505 | store
  // =================================================

  vsetvli t0, x0, e8, m1
  vl1re8.v v0, (x21)
  vse8.v v31, (x20) // reset dest
  vse8.v v1, (x20), v0.t
  
  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b | MASK : 0x05050505 | store with leftovers
  // =================================================

  vsetvli t0, x0, e16, m1
  vl1re16.v v0, (x21)
  vse16.v v1, (x20), v0.t
  
  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x56
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x43
  slli x22, x22, 8
  addi x22, x22, 0x21
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 16b | MASK : 0x05050505 | store
  // =================================================

  vsetvli t0, x0, e16, m1
  vl1re16.v v0, (x21)
  vse16.v v31, (x20) // reset dest
  vse16.v v1, (x20), v0.t
  
  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x56
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x43
  slli x22, x22, 8
  addi x22, x22, 0x21
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | MASK : 0x05050505 | LMUL=2
  // =================================================

  vl1re8.v v0, (x21)
  vsetvli t0, x0, e8, m2
  vse8.v v30, (x20) // reset dest
  vle8.v v1, (x18)
  vand.vv v8, v30, v30 // reset v8
  vand.vv v8, v1, v1, v0.t // masked vand
  vse8.v v8, (x20)
  
  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  
  lw t0, 16(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x11
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 20(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 24(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xef
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0xef
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 28(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | MASK : 0x05050505 | test no 0 written when masked out
  // =================================================

  vsetvli t0, x0, e8, m1
  vl1re8.v v0, (x21)
  vand.vv v2, v3, v3
  vand.vv v2, v1, v1, v0.t // mask v1 with v0.t
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x34
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x78
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x05
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x875c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x0C
  slli x22, x22, 8
  addi x22, x22, 0xcd
  slli x22, x22, 8
  addi x22, x22, 0x0A
  slli x22, x22, 8
  addi x22, x22, 0xcd
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0xabc4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x0F
  slli x22, x22, 8
  addi x22, x22, 0x0E
  slli x22, x22, 8
  addi x22, x22, 0x0D
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

mask1:  .word 0x05050505
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
         
mask2:  .word 0x50505050
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
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
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        
clear:  .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
err:    .word 0xE0E0E0E0

//RVTEST_DATA_END
