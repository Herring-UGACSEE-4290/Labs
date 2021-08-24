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

/** Test Load & Store register offset instructions
*
* Test various sizes (B/H/W/X)
* Test various sign extension (LSL/SXTX/UXTW/SXTW)
* Test shift on various sizes (B/H/W/X)
* Test Rn is SP
* Test Rm is RZR
* Test Rt is RZR
* Test persistence of stores
* 
*/

#define PTR_DIFF 0x1234
TEST_LD_UVAL=0x7888AAAA3CCB190F
TEST_LD_SVAL=~TEST_LD_UVAL
TEST_LD_V1=0x4433333333222211
TEST_LD_V2=0x8877777777666655
.data
_test_data_u: .quad TEST_LD_UVAL

_test_sign_ext:
.quad TEST_LD_V1
.quad TEST_LD_V2

.org PTR_DIFF
_test_data_s: .quad TEST_LD_SVAL
_store_data:
.quad 0x0, 0x0

// X0: input base address
// X1: input address offset
// X2: input expected value

.macro LDR_UNIT_TEST sz=NULL, reg_sz:req, rm_sz=X, ext=LSL, sh=0, rm=1, rt=3
	.ifc \sz, NULL
		LDR	\reg_sz\rt, [X0, \rm_sz\rm, \ext \sh]
	.else
		LDR\sz	\reg_sz\rt, [X0, \rm_sz\rm, \ext \sh]
	.endif
		CMP	X\rt, X2
		BNE	_test_fail
.endm

.text
_start:
/******************************************************************************/
// Test various sizes & load extensions
		ADR	X0 , _test_data_u
		MOV	X1 , 0

		AS_MOVX	W,2, UXTB, TEST_LD_UVAL
		LDR_UNIT_TEST  B, W
		AS_MOVX	W,2, SXTB, TEST_LD_UVAL
		LDR_UNIT_TEST SB, W
		AS_MOVX	X,2, SXTB, TEST_LD_UVAL
		LDR_UNIT_TEST SB, X

		AS_MOVX	W,2, UXTH, TEST_LD_UVAL
		LDR_UNIT_TEST  H, W
		AS_MOVX	W,2, SXTH, TEST_LD_UVAL
		LDR_UNIT_TEST SH, W
		AS_MOVX	X,2, SXTH, TEST_LD_UVAL
		LDR_UNIT_TEST SH, X

		AS_MOVX	W,2, UXTW, TEST_LD_UVAL
		LDR_UNIT_TEST   , W
		AS_MOVX	X,2, SXTW, TEST_LD_UVAL
		LDR_UNIT_TEST SW, X

		AS_MOVX	X,2, UXTX, TEST_LD_UVAL
		LDR_UNIT_TEST   , X

		MOVZ	X1 , PTR_DIFF

		AS_MOVX	W,2, UXTB, TEST_LD_SVAL
		LDR_UNIT_TEST  B, W
		AS_MOVX	W,2, SXTB, TEST_LD_SVAL
		LDR_UNIT_TEST SB, W
		AS_MOVX	X,2, SXTB, TEST_LD_SVAL
		LDR_UNIT_TEST SB, X

		AS_MOVX	W,2, UXTH, TEST_LD_SVAL
		LDR_UNIT_TEST  H, W
		AS_MOVX	W,2, SXTH, TEST_LD_SVAL
		LDR_UNIT_TEST SH, W
		AS_MOVX	X,2, SXTH, TEST_LD_SVAL
		LDR_UNIT_TEST SH, X

		AS_MOVX	W,2, UXTW, TEST_LD_SVAL
		LDR_UNIT_TEST   , W
		AS_MOVX	X,2, SXTW, TEST_LD_SVAL
		LDR_UNIT_TEST SW, X

		AS_MOVX	X,2, UXTX, TEST_LD_SVAL
		LDR_UNIT_TEST   , X

/******************************************************************************/
// Test various sign extensions & shifts

		ADR	X0 , _test_sign_ext
		MOVZ	X4 , 0x7e7e, LSL 48
		ORR	X4 , X4 , 1
		MOV	W1 , W4

		AS_MOVX W,2 , UXTB, (TEST_LD_V1>>8)
		LDR_UNIT_TEST  B, W, X,  LSL, 0, 1
		LDR_UNIT_TEST  B, W, W, UXTW, 0, 4

		AS_MOVX W,2 , UXTH, (TEST_LD_V1>>16)
		LDR_UNIT_TEST  H, W, X,  LSL, 1, 1
		LDR_UNIT_TEST  H, W, W, UXTW, 1, 4

		AS_MOVX W,2 , UXTW, (TEST_LD_V1>>32)
		LDR_UNIT_TEST   , W, X,  LSL, 2, 1
		LDR_UNIT_TEST   , W, W, UXTW, 2, 4

		AS_MOVX X,2 , UXTX, TEST_LD_V2
		LDR_UNIT_TEST   , X, X,  LSL, 3, 1
		LDR_UNIT_TEST   , X, W, UXTW, 3, 4

		ADD	X0 , X0 , 16
		ORR	X4 , X4 , 0xFFFFFFFE
		BIC	X4 , X4 , 0x1
		ORR	X1 , X4 , 0xFFFFFFFF00000000

		AS_MOVX W,2 , UXTB, (TEST_LD_V2>>48)
		LDR_UNIT_TEST  B, W, X, SXTX, 0, 1
		LDR_UNIT_TEST  B, W, W, SXTW, 0, 4

		AS_MOVX W,2 , UXTH, (TEST_LD_V2>>32)
		LDR_UNIT_TEST  H, W, X, SXTX, 1, 1
		LDR_UNIT_TEST  H, W, W, SXTW, 1, 4

		AS_MOVX W,2 , UXTW, TEST_LD_V2
		LDR_UNIT_TEST   , W, X, SXTX, 2, 1
		LDR_UNIT_TEST   , W, W, SXTW, 2, 4

		AS_MOVX X,2 , UXTX, TEST_LD_V1
		LDR_UNIT_TEST   , X, X, SXTX, 3, 1
		LDR_UNIT_TEST   , X, W, SXTW, 3, 4
		
/******************************************************************************/
// Test Rn is SP, Rm is RZR, Rt is RZR

		ADR	X0 , _test_data_s
		MOV	SP , X0
		MOV	X1 , 0
		AS_MOVX	X,2 , UXTX, TEST_LD_SVAL

		LDR	X3 , [SP , X1]
		CMP	X3 , X2
		BNE	_test_fail

		LDR	X4 , [SP , XZR]
		CMP	X4 , X2
		BNE	_test_fail

		LDR	XZR, [SP , X1]
		CMP	SP , X0
		BNE	_test_fail
		CMP	SP , X0 // Check after pipline flush
		BNE	_test_fail

/******************************************************************************/
// Test persistence of stores

		ADR	X10, _store_data
		MOV	X1 , 8

// Value to store: X0
// Base address:   X10
.macro STORE_TEST inst:req, rt_sz:req, val:req, inc:req, rt=0, ZR=ZR
	AS_MOVX	X,\rt, UXTX, \val

l1\@:
		\inst	\rt_sz\rt, [X10, X1]

		ROR	X\rt , X\rt , (\inc*4)
		ROR	X\rt , X\rt , (\inc*4)
		ADD	X1 , X1 , \inc
		CMP	X1 , 16
		BLT	l1\@

		LDR	X3 , _store_data+8
		CMP	X3 , X0
		BNE	_test_fail

		ADD	X10, X10, X1
		MOV	X1, -\inc
l2\@:
		\inst	\rt_sz\ZR, [X10, X1]
		SUB	X1 , X1, \inc
		CMP	X1 , -8
		BGE	l2\@

		LDR	X3 , _store_data+8
		CMP	X3 , XZR
		BNE	_test_fail

		ADR	X10, _store_data
		MOV	X1 , 8
.endm
 
#define BYTE_STR_VAL 0x03020100FFFEFDFC
		STORE_TEST STRB, W, BYTE_STR_VAL, 1

#define HALF_STR_VAL 0x12348765ACBDFE90
		STORE_TEST STRH, W, HALF_STR_VAL, 2

#define WORD_STR_VAL 0xFEDCBA9801234567
		STORE_TEST STR , W, WORD_STR_VAL, 4

#define QUAD_STR_VAL 0x0123456789ABCDEF
		STORE_TEST STR , X, QUAD_STR_VAL, 8

		YIELD
_test_fail:	BRK	0
