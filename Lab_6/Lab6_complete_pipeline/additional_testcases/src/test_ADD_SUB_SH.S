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

/** Test Arithmetic shifted register instructions

* Test RZR is source
* Test RZR is destination
* Test various widths (X/W)
* Test flag setting of ADDS/SUBS
* Test shift operation & value on operand M

*/

.macro UNIT_TEST op:req, reg_d:req, reg_n:req, reg_m:req, cmp:req, shift:vararg
	.ifb \shift
		\op  \reg_d , \reg_n , \reg_m
	.else
		\op  \reg_d , \reg_n , \reg_m , \shift
	.endif

		CMP  \reg_d , \cmp
		BNE  _test_fail
.endm

.data
TEST_INIT=           0x0123456789ABCDEF
_TEST_INIT_SP: .quad 0xABABCDCDEFEF0101
_NEG_TEST_INIT:.quad -TEST_INIT
_TEST_INIT:    .quad TEST_INIT

#define TEST_1_SHAMT 12
#define TEST_1_SHOP  LSL
#define TEST_1_REGSZ X
#define TEST_1_OP    SUB
AS_ARITH_SH TEST_1_OP, TEST_1_REGSZ, TEST_INIT, TEST_INIT, TEST_1_SHOP, TEST_1_SHAMT
_TEST_1_RESULT:  .quad AS_ARITH_SH_OUT

#define TEST_2_SHAMT 16
#define TEST_2_SHOP  LSR
#define TEST_2_REGSZ X
#define TEST_2_OP    ADD
AS_ARITH_SH TEST_2_OP, TEST_2_REGSZ, TEST_INIT, TEST_INIT, TEST_2_SHOP, TEST_2_SHAMT
_TEST_2_RESULT:  .quad AS_ARITH_SH_OUT

#define TEST_3_SHAMT 48
#define TEST_3_SHOP  ASR
#define TEST_3_REGSZ X
#define TEST_3_OP    SUB
AS_ARITH_SH TEST_3_OP, TEST_3_REGSZ, TEST_INIT, TEST_INIT, TEST_3_SHOP, TEST_3_SHAMT
_TEST_3_RESULT:  .quad AS_ARITH_SH_OUT

#define TEST_4_SHAMT 16
#define TEST_4_SHOP  ASR
#define TEST_4_REGSZ W
#define TEST_4_OP    ADD
AS_ARITH_SH TEST_4_OP, TEST_4_REGSZ, TEST_INIT, TEST_INIT, TEST_4_SHOP, TEST_4_SHAMT
_TEST_4_RESULT: .quad AS_ARITH_SH_OUT

.text
_start:		// Set the SP to some value != 0
		LDR  X2 , _TEST_INIT_SP
		MOV  SP , X2

		// Set X0 to another value != 0
		LDR  X0 , _TEST_INIT

		// Apply instructions
		//  XXX  X1 , X0 , XZR
		UNIT_TEST ADD , X1 , X0 , XZR, X0
		UNIT_TEST ADDS, X1 , X0 , XZR, X0
		UNIT_TEST SUB , X1 , X0 , XZR, X0
		UNIT_TEST SUBS, X1 , X0 , XZR, X0
		
		LDR  X3 , _NEG_TEST_INIT
		//  XXX  X1 , XZR, X0
		UNIT_TEST ADD , X1 , XZR, X0 , X0
		UNIT_TEST ADDS, X1 , XZR, X0 , X0
		UNIT_TEST SUB , X1 , XZR, X0 , X3
		UNIT_TEST SUBS, X1 , XZR, X0 , X3

		MOVZ X3, 0
		//  XXX  XZR, X0 , X0
		UNIT_TEST ADD , XZR, X0 , X0 , X3
		UNIT_TEST ADDS, XZR, X0 , X0 , X3
		UNIT_TEST SUB , XZR, X0 , X0 , X3
		UNIT_TEST SUBS, XZR, X0 , X0 , X3

		// Check SP hasn't changed for XZR
		CMP SP, X2
		BNE _test_fail

		// Set X0 to some value of width X
		MOV  W3 , W0
		SUBS W1 , W0 , WZR
		BPL  _test_fail
		CMP  X1 , X3
		BNE  _test_fail

		// Apply various shifts on both widths
		LDR  X3 , _TEST_1_RESULT
		UNIT_TEST TEST_1_OP, X1 , X0 , X0 , X3 , TEST_1_SHOP #TEST_1_SHAMT
		LDR  X3 , _TEST_2_RESULT
		UNIT_TEST TEST_2_OP, X1 , X0 , X0 , X3 , TEST_2_SHOP #TEST_2_SHAMT
		LDR  X3 , _TEST_3_RESULT
		UNIT_TEST TEST_3_OP, X1 , X0 , X0 , X3 , TEST_3_SHOP #TEST_3_SHAMT
		LDR  X3 , _TEST_4_RESULT
		UNIT_TEST TEST_4_OP, W1 , W0 , W0 , W3 , TEST_4_SHOP #TEST_4_SHAMT
		CMP  X3 , X1
		BNE  _test_fail

		// Test overflow flag setting on X
		MOV  X0 , 0x7FFFFFFFFFFFFFFF
		MOV  X1 , 0x1
		MOV  X2 , -1
		MOV  X4 , -2

		ADDS XZR, X0 , X1
		BVC  _test_fail

		SUBS XZR, X0 , X2
		BVC  _test_fail

		SUBS XZR, X4 , X0
		BVC  _test_fail

		// Test overflow flag setting on W
		MOV  W3 , 0x7FFFFFFF

		ADDS WZR, W3 , W1
		BVC  _test_fail

		SUBS WZR, W3 , W2
		BVC  _test_fail

		SUBS WZR, W4 , W3
		BVC  _test_fail

		// Test carry flag setting on X
		ORR  X0 , X0 , 0x8000000000000000
		
		ADDS XZR, X0 , X1
		BCC  _test_fail

		SUBS XZR, X0 , X0
		BCC  _test_fail

		// Test carry flag setting on W
		ORR  W0 , W0 , 0x80000000

		ADDS WZR, W0 , W1
		BCC  _test_fail

		SUBS WZR, W0 , W0
		BCC  _test_fail

		YIELD
_test_fail:	BRK 0

