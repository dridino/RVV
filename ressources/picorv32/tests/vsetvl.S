# See LICENSE for license details.

#*****************************************************************************
# sub.S
#-----------------------------------------------------------------------------
#
# Test sub instruction.
#

#include "riscv_test.h"
#include "test_macros.h"

RVTEST_RV32U
RVTEST_CODE_BEGIN

  # ---------------------------
  # LMUL = 1
  # ---------------------------

  li a0, 32
  
  TEST_VSETVL(1, t0, 4, vsetvli t0, a0, e32);
  TEST_VSETVL(2, t0, 8, vsetvli t0, a0, e16);
  TEST_VSETVL(3, t0, 16, vsetvli t0, a0, e8);
  TEST_VSETVL(51, t0, 4, li a1, 0b00010000; vsetvl t0, a0, a1);
  TEST_VSETVL(52, t0, 8, li a1, 0b00001000; vsetvl t0, a0, a1);
  TEST_VSETVL(53, t0, 16, li a1, 0b00000000; vsetvl t0, a0, a1);


  # ---------------------------
  # LMUL = 2
  # ---------------------------

  li a0, 32
  
  TEST_VSETVL(4, t0, 8, vsetvli t0, a0, e32, m2);
  TEST_VSETVL(5, t0, 16, vsetvli t0, a0, e16, m2);
  TEST_VSETVL(6, t0, 32, vsetvli t0, a0, e8, m2);
  TEST_VSETVL(54, t0, 8, li a1, 0b00010001; vsetvl t0, a0, a1);
  TEST_VSETVL(55, t0, 16, li a1, 0b00001001; vsetvl t0, a0, a1);
  TEST_VSETVL(56, t0, 32, li a1, 0b00000001; vsetvl t0, a0, a1);


  # ---------------------------
  # LMUL = 4
  # ---------------------------

  li a0, 64

  TEST_VSETVL(7, t0, 16, vsetvli t0, a0, e32, m4);
  TEST_VSETVL(8, t0, 32, vsetvli t0, a0, e16, m4);
  TEST_VSETVL(9, t0, 64, vsetvli t0, a0, e8, m4);
  TEST_VSETVL(57, t0, 16, li a1, 0b00010010; vsetvl t0, a0, a1);
  TEST_VSETVL(58, t0, 32, li a1, 0b00001010; vsetvl t0, a0, a1);
  TEST_VSETVL(59, t0, 64, li a1, 0b00000010; vsetvl t0, a0, a1);


  # ---------------------------
  # LMUL = 8
  # ---------------------------

  li a0, 128
  
  TEST_VSETVL(10, t0, 32, vsetvli t0, a0, e32, m8);
  TEST_VSETVL(11, t0, 64, vsetvli t0, a0, e16, m8);
  TEST_VSETVL(12, t0, 128, vsetvli t0, a0, e8, m8);
  TEST_VSETVL(60, t0, 32, li a1, 0b00010011; vsetvl t0, a0, a1);
  TEST_VSETVL(61, t0, 64, li a1, 0b00001011; vsetvl t0, a0, a1);
  TEST_VSETVL(62, t0, 128, li a1, 0b00000011; vsetvl t0, a0, a1);
  

  # ---------------------------
  # LMUL = 1/2
  # ---------------------------

  li a0, 32
  
  TEST_VSETVL(13, t0, 2, vsetvli t0, a0, e32, mf2);
  TEST_VSETVL(14, t0, 4, vsetvli t0, a0, e16, mf2);
  TEST_VSETVL(15, t0, 8, vsetvli t0, a0, e8, mf2);
  TEST_VSETVL(63, t0, 2, li a1, 0b00010111; vsetvl t0, a0, a1);
  TEST_VSETVL(64, t0, 4, li a1, 0b00001111; vsetvl t0, a0, a1);
  TEST_VSETVL(65, t0, 8, li a1, 0b00000111; vsetvl t0, a0, a1);
  

  # ---------------------------
  # LMUL = 1/4
  # ---------------------------

  li a0, 64
  
  TEST_VSETVL(16, t0, 1, vsetvli t0, a0, e32, mf4);  
  TEST_VSETVL(17, t0, 2, vsetvli t0, a0, e16, mf4);
  TEST_VSETVL(18, t0, 4, vsetvli t0, a0, e8, mf4);
  TEST_VSETVL(66, t0, 1, li a1, 0b00010110; vsetvl t0, a0, a1);
  TEST_VSETVL(67, t0, 2, li a1, 0b00001110; vsetvl t0, a0, a1);
  TEST_VSETVL(68, t0, 4, li a1, 0b00000110; vsetvl t0, a0, a1);


  # ---------------------------
  # LMUL = 1/8
  # ---------------------------

  li a0, 128
    
  TEST_VSETVL(19, t0, 0, vsetvli t0, a0, e32, mf8);
  TEST_VSETVL(20, t0, 1, vsetvli t0, a0, e16, mf8);
  TEST_VSETVL(21, t0, 2, vsetvli t0, a0, e8, mf8);
  TEST_VSETVL(69, t0, 0, li a1, 0b00010101; vsetvl t0, a0, a1);
  TEST_VSETVL(70, t0, 1, li a1, 0b00001101; vsetvl t0, a0, a1);
  TEST_VSETVL(71, t0, 2, li a1, 0b00000101; vsetvl t0, a0, a1);


  # ---------------------------
  # AVL limiting
  # ---------------------------

  li a0, 8

  TEST_VSETVL(22, t0, 8, vsetvli t0, a0, e8);


  # ---------------------------
  # VSETIVLI
  # ---------------------------
  
  TEST_VSETVL(23, t0, 4, vsetivli t0, 31, e32);
  TEST_VSETVL(24, t0, 8, vsetivli t0, 31, e16);
  TEST_VSETVL(25, t0, 16, vsetivli t0, 31, e8);
  TEST_VSETVL(26, t0, 8, vsetivli t0, 31, e32, m2);
  TEST_VSETVL(27, t0, 16, vsetivli t0, 31, e16, m2);
  TEST_VSETVL(28, t0, 31, vsetivli t0, 31, e8, m2);


  # ---------------------------
  # special values for AVL
  # ---------------------------

  TEST_VSETVL(29, t0, 16, vsetvli t0, x0, e8);
  TEST_VSETVL(30, t0, 8, vsetvli t0, x0, e16);
  vsetvli x0, x0, e8 # should keep the same vl value

  TEST_PASSFAIL

RVTEST_CODE_END

  .data
RVTEST_DATA_BEGIN

  TEST_DATA

RVTEST_DATA_END
