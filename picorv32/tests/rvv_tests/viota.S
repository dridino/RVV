# See LICENSE for license details.

#*****************************************************************************
# viota.S
#-----------------------------------------------------------------------------
#
# Test viota instruction
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

  vle8.v v1, (x18) // 2C
  vle8.v v3, (x19)
  la    x19, mask2
  vle8.v v0, (x19)

  // =================================================
  // = SEW = 8b | LMUL = 1 |
  // =================================================

  vsetvli t0, x0, e8, m1
  viota.m v2, v1
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x005c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x05
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x04
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x06
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | LMUL = 1 | masked
  // =================================================

  vsetvli t0, x0, e8, m1
  viota.m v2, v1, v0.t
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x01
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x005c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x05
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x04
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x02
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | LMUL = 1/2 |
  // =================================================
  
  vmv.v.i v2, 0
  vsetvli t0, x0, e8, mf2
  viota.m v2, v1
  vsetvli t0, x0, e8, m1
  vse8.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x005c4318)
  
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
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
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
  // = SEW = 16b | LMUL = 1 |
  // =================================================

  vsetvli t0, x0, e16, m1
  viota.m v2, v1
  vse16.v v2, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
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
  // TEST_CASE( 2, t0, 0x005c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x03
  bne t0,x22,.trap
  nop

  // =================================================
  // = SEW = 8b | LMUL = 2 |
  // =================================================
  
  vmv.v.i v16, 0
  vsetvli t0, x0, e8, m2
  viota.m v16, v1
  vse8.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x005c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x05
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x04
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x06
  bne t0,x22,.trap
  nop

  addi x20, x20, 16
  lw t0, (x20)
  addi x22, x0, 0x09
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x08
  bne t0,x22,.trap
  nop
  
  lw t0, 4(x20)
  addi x22, x0, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0a
  slli x22, x22, 8
  addi x22, x22, 0x09
  bne t0,x22,.trap
  nop
  
  lw t0, 8(x20)
  addi x22, x0, 0x0c
  slli x22, x22, 8
  addi x22, x22, 0x0c
  slli x22, x22, 8
  addi x22, x22, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0b
  bne t0,x22,.trap
  nop
  
  lw t0, 12(x20)
  addi x22, x0, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0c
  bne t0,x22,.trap
  nop

  addi x20, x20, -16

  // =================================================
  // = SEW = 8b | LMUL = 4 |
  // =================================================
  
  vmv.v.i v16, 0
  vsetvli t0, x0, e8, m2
  vle8.v v1, (x18)
  vsetvli t0, x0, e8, m4
  viota.m v16, v1
  vse8.v v16, (x20)

  lw t0, (x20)
  addi x22, x0, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  slli x22, x22, 8
  addi x22, x22, 0x00
  bne t0,x22,.trap
  nop
  // TEST_CASE( 1, t0, 0x122b566f)
  
  lw t0, 4(x20)
  addi x22, x0, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x03
  slli x22, x22, 8
  addi x22, x22, 0x02
  slli x22, x22, 8
  addi x22, x22, 0x01
  bne t0,x22,.trap
  nop
  // TEST_CASE( 2, t0, 0x005c4318)
  
  lw t0, 8(x20)
  addi x22, x0, 0x06
  slli x22, x22, 8
  addi x22, x22, 0x05
  slli x22, x22, 8
  addi x22, x22, 0x04
  slli x22, x22, 8
  addi x22, x22, 0x04
  bne t0,x22,.trap
  nop
  // TEST_CASE( 3, t0, 0x00c4abc4)
  
  lw t0, 12(x20)
  addi x22, x0, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x07
  slli x22, x22, 8
  addi x22, x22, 0x06
  bne t0,x22,.trap
  nop

// 2
  addi x20, x20, 16
  lw t0, (x20)
  addi x22, x0, 0x09
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x08
  slli x22, x22, 8
  addi x22, x22, 0x08
  bne t0,x22,.trap
  nop
  
  lw t0, 4(x20)
  addi x22, x0, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0a
  slli x22, x22, 8
  addi x22, x22, 0x09
  bne t0,x22,.trap
  nop
  
  lw t0, 8(x20)
  addi x22, x0, 0x0c
  slli x22, x22, 8
  addi x22, x22, 0x0c
  slli x22, x22, 8
  addi x22, x22, 0x0b
  slli x22, x22, 8
  addi x22, x22, 0x0b
  bne t0,x22,.trap
  nop
  
  lw t0, 12(x20)
  addi x22, x0, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0d
  slli x22, x22, 8
  addi x22, x22, 0x0c
  bne t0,x22,.trap
  nop

// 3
  addi x20, x20, 16
  lw t0, (x20)
  addi x22, x0, 0x0e
  slli x22, x22, 8
  addi x22, x22, 0x0e
  slli x22, x22, 8
  addi x22, x22, 0x0e
  slli x22, x22, 8
  addi x22, x22, 0x0d
  bne t0,x22,.trap
  nop
  
  lw t0, 4(x20)
  addi x22, x0, 0x0f
  slli x22, x22, 8
  addi x22, x22, 0x0f
  slli x22, x22, 8
  addi x22, x22, 0x0e
  slli x22, x22, 8
  addi x22, x22, 0x0e
  bne t0,x22,.trap
  nop
  
  lw t0, 8(x20)
  addi x22, x0, 0x11
  slli x22, x22, 8
  addi x22, x22, 0x11
  slli x22, x22, 8
  addi x22, x22, 0x10
  slli x22, x22, 8
  addi x22, x22, 0x0f
  bne t0,x22,.trap
  nop
  
  lw t0, 12(x20)
  addi x22, x0, 0x12
  slli x22, x22, 8
  addi x22, x22, 0x11
  slli x22, x22, 8
  addi x22, x22, 0x11
  slli x22, x22, 8
  addi x22, x22, 0x11
  bne t0,x22,.trap
  nop

// 4
  addi x20, x20, 16
  lw t0, (x20)
  addi x22, x0, 0x15
  slli x22, x22, 8
  addi x22, x22, 0x14
  slli x22, x22, 8
  addi x22, x22, 0x14
  slli x22, x22, 8
  addi x22, x22, 0x13
  bne t0,x22,.trap
  nop
  
  lw t0, 4(x20)
  addi x22, x0, 0x17
  slli x22, x22, 8
  addi x22, x22, 0x16
  slli x22, x22, 8
  addi x22, x22, 0x15
  slli x22, x22, 8
  addi x22, x22, 0x15
  bne t0,x22,.trap
  nop
  
  lw t0, 8(x20)
  addi x22, x0, 0x1a
  slli x22, x22, 8
  addi x22, x22, 0x19
  slli x22, x22, 8
  addi x22, x22, 0x18
  slli x22, x22, 8
  addi x22, x22, 0x17
  bne t0,x22,.trap
  nop
  
  lw t0, 12(x20)
  addi x22, x0, 0x1a
  slli x22, x22, 8
  addi x22, x22, 0x1a
  slli x22, x22, 8
  addi x22, x22, 0x1a
  slli x22, x22, 8
  addi x22, x22, 0x1a
  bne t0,x22,.trap
  nop

  addi x20, x20, -16
  addi x20, x20, -16
  addi x20, x20, -16

  j .start
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
tdat2:  .word 0x8765c321
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

tdat8:  .word 0x00030001
        .word 0x08000600
        .word 0x00000000
        .word 0x000F0E0D

mask1:  .word 0x05050505
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
         
mask2:  .word 0x50505050
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
        
clear: .word 0x00000000
        .word 0x00000000
        .word 0x00000000
        .word 0x00000000
err:    .word 0xE0E0E0E0

//RVTEST_DATA_END
