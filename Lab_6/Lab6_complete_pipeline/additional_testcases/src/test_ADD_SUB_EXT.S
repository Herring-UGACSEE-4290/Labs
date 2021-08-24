//----------------------------------------------------------------------------
//The information contained in this file may only be used by a person
//authorised under and to the extent permitted by a subsisting licensing 
//agreement from Arm Limited or its affiliates 
//
//(C) COPYRIGHT 2020 Arm Limited or its affiliates
//ALL RIGHTS RESERVED.
//Licensed under the ARM EDUCATION INTRODUCTION TO COMPUTER ARCHITECTURE 
//EDUCATION KIT END USER LICENSE AGREEMENT.
//See https://www.arm.com/-/media/Files/pdf/education/computer-architecture-education-kit-eula
//
//This entire notice must be reproduced on all copies of this file
//and copies of this file may only be made by a person if such person is
//permitted to do so under the terms of a subsisting license agreement
//from Arm Limited or its affiliates.
//----------------------------------------------------------------------------
#include "as_macro.S"

/** Test Arithmetic Extended register instructions

* Test SP is source
* Test SP is destination on ADD/SUB
* Test RZR is destination on ADDS/SUBS
* Test various widths (X/W)
* Test flag setting of ADDS/SUBS
* Test shift value on operand M
* Test sign extension on operand M

*/

.macro UNIT_TEST op:req, reg_d:req, reg_n:req, reg_m:req, cmp:req, ext:req, lshift=0
		\op  \reg_d , \reg_n , \reg_m , \ext #\lshift
		CMP  \reg_d , \cmp
		BNE  _test_fail
.endm

.data
TEST_INIT     =      0x0123456789ABCDEF
TEST_INIT_SP  =      0xABABCDCDEFEF0101 
_TEST_INIT_SP: .quad TEST_INIT_SP
_TEST_INIT:    .quad TEST_INIT

#define TEST_1_SHAMT 2
#define TEST_1_EXT   UXTB
#define TEST_1_REGSZ X
#define TEST_1_OP    SUB
AS_ARITH_EXT TEST_1_OP, TEST_1_REGSZ, TEST_INIT, TEST_INIT, TEST_1_EXT, TEST_1_SHAMT
_TEST_1_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_2_SHAMT 1
#define TEST_2_EXT   SXTH
#define TEST_2_REGSZ X
#define TEST_2_OP    ADD
AS_ARITH_EXT TEST_2_OP, TEST_2_REGSZ, TEST_INIT, TEST_INIT, TEST_2_EXT, TEST_2_SHAMT
_TEST_2_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_3_SHAMT 3
#define TEST_3_EXT   SXTW
#define TEST_3_REGSZ X
#define TEST_3_OP    SUB
AS_ARITH_EXT TEST_3_OP, TEST_3_REGSZ, TEST_INIT, TEST_INIT, TEST_3_EXT, TEST_3_SHAMT
_TEST_3_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_4_SHAMT 4
#define TEST_4_EXT   SXTX
#define TEST_4_REGSZ X
#define TEST_4_OP    ADD
AS_ARITH_EXT TEST_4_OP, TEST_4_REGSZ, TEST_INIT, TEST_INIT, TEST_4_EXT, TEST_4_SHAMT
_TEST_4_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_5_SHAMT 1
#define TEST_5_EXT   UXTW
#define TEST_5_REGSZ X
#define TEST_5_OP    ADD
AS_ARITH_EXT TEST_5_OP, TEST_5_REGSZ, TEST_INIT, TEST_INIT, TEST_5_EXT, TEST_5_SHAMT
_TEST_5_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_6_SHAMT 2
#define TEST_6_EXT   UXTH
#define TEST_6_REGSZ W
#define TEST_6_OP    SUB
AS_ARITH_EXT TEST_6_OP, TEST_6_REGSZ, TEST_INIT, TEST_INIT, TEST_6_EXT, TEST_6_SHAMT
_TEST_6_RESULT:  .quad AS_ARITH_EXT_OUT

#define TEST_7_SHAMT 0
#define TEST_7_EXT   UXTB
#define TEST_7_REGSZ W
#define TEST_7_OP    ADD
AS_ARITH_EXT TEST_7_OP, TEST_7_REGSZ, TEST_INIT, TEST_INIT, TEST_7_EXT, TEST_7_SHAMT
_TEST_7_RESULT:  .quad AS_ARITH_EXT_OUT

_TEST_SP_RESULT_A: .quad TEST_INIT_SP+TEST_INIT
_TEST_SP_RESULT_S: .quad TEST_INIT_SP-TEST_INIT

.text
_start:		// Set the SP to some value != 0
		LDR  X2 , _TEST_INIT_SP
		MOV  SP , X2

		// Set X0 to another value != 0
		LDR  X0 , _TEST_INIT

		LDR  X3 , _TEST_SP_RESULT_A
		LDR  X4 , _TEST_SP_RESULT_S
		// Test the SP is the N operand ALWAYS
		UNIT_TEST ADD , X1 , SP , X0 , X3, UXTX
		UNIT_TEST ADDS, X1 , SP , X0 , X3, SXTX	
		UNIT_TEST SUB , W1 , WSP, W0 , W4, UXTW
		UNIT_TEST SUBS, W1 , WSP, W0 , W4, SXTW

		// Test the SP is the destination in ADD/SUB
		UNIT_TEST ADD , SP , SP , X0 , X3, UXTX
		UNIT_TEST SUB , SP , SP , X0 , X2, SXTX

		MOVZ X5, 0
		MOV  X6, SP
		// Test XZR is the destination in ADDS/SUBS
		UNIT_TEST ADDS, XZR, X0, X0 , X5, UXTX
		UNIT_TEST SUBS, XZR, SP, X0 , X5, SXTX

		// Test XZR is the M operand ALWAYS
		UNIT_TEST ADD , X1 , SP , XZR , X6 , UXTX
		UNIT_TEST ADDS, X1 , SP , WZR , X6 , SXTW
		UNIT_TEST SUB , W1 , WSP, WZR , W6 , UXTH
		UNIT_TEST SUBS, W1 , WSP, WZR , W6 , UXTB, 2

		// Apply various shifts on both widths
		LDR  X3 , _TEST_1_RESULT
		UNIT_TEST TEST_1_OP, X1 , X0 , W0 , X3 , TEST_1_EXT, TEST_1_SHAMT
		LDR  X3 , _TEST_2_RESULT
		UNIT_TEST TEST_2_OP, X1 , X0 , W0 , X3 , TEST_2_EXT, TEST_2_SHAMT
		LDR  X3 , _TEST_3_RESULT
		UNIT_TEST TEST_3_OP, X1 , X0 , W0 , X3 , TEST_3_EXT, TEST_3_SHAMT
		LDR  X3 , _TEST_4_RESULT
		UNIT_TEST TEST_4_OP, X1 , X0 , X0 , X3 , TEST_4_EXT, TEST_4_SHAMT
		LDR  X3 , _TEST_5_RESULT
		UNIT_TEST TEST_5_OP, X1 , X0 , W0 , X3 , TEST_5_EXT, TEST_5_SHAMT
		LDR  X3 , _TEST_6_RESULT
		UNIT_TEST TEST_6_OP, W1 , W0 , W0 , W3 , TEST_6_EXT, TEST_6_SHAMT
		CMP  X3 , X1 // Test registers match on full width
		LDR  X3 , _TEST_7_RESULT
		UNIT_TEST TEST_7_OP, W1 , W0 , W0 , W3 , TEST_7_EXT, TEST_7_SHAMT
		CMP  X3 , X1 // Test registers match on full width
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0

