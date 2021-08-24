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

/** Test Load & Store w/ immediate offset instructions
*
* Test various sizes (B/H/W/X)
* Test sign extension on various sizes (B/H/W)
* Test Rt sizes (W/X)
* Test various types (unscaled/pre-/post-)
* Test Rn is SP
* Test RT is RZR
* Test sign extension of 9-bit immediate
* Test persistence of stores
*
*/

.macro MEM_VAL_GEN idx:req

		MEM_VAL_64u=0xDEADABBA00000000

	.if \idx & 0x1
		MEM_VAL_32u=0xFEAD0000
	.else
		MEM_VAL_32u=0x6EAD0000
	.endif

	.if \idx & 0x2
		MEM_VAL_16u = 0xC300
	.else
		MEM_VAL_16u = 0x7B00
	.endif

	.if \idx & 0x4
		MEM_VAL_8u  = 0xA0 | (\idx & 0x1F)
	.else
		MEM_VAL_8u  = 0x40 | (\idx & 0x1F)
	.endif

		MEM_VAL= MEM_VAL_64u | MEM_VAL_32u | MEM_VAL_16u | MEM_VAL_8u
.endm

.data
.org 40
_array_start:
MEM_VAL_GEN 0
.quad MEM_VAL
MEM_VAL_GEN 1
.quad MEM_VAL
MEM_VAL_GEN 2
.quad MEM_VAL
MEM_VAL_GEN 3
.quad MEM_VAL
MEM_VAL_GEN 4
.quad MEM_VAL
MEM_VAL_GEN 5
.quad MEM_VAL
MEM_VAL_GEN 6
.quad MEM_VAL
MEM_VAL_GEN 7
.quad MEM_VAL
MEM_VAL_GEN 8
.quad MEM_VAL
MEM_VAL_GEN 9
.quad MEM_VAL
MEM_VAL_GEN 10
.quad MEM_VAL
MEM_VAL_GEN 11
.quad MEM_VAL
MEM_VAL_GEN 12
.quad MEM_VAL
MEM_VAL_GEN 13
.quad MEM_VAL
MEM_VAL_GEN 14
.quad MEM_VAL
MEM_VAL_GEN 15
.quad MEM_VAL
MEM_VAL_GEN 16
.quad MEM_VAL
MEM_VAL_GEN 17
.quad MEM_VAL
MEM_VAL_GEN 18
.quad MEM_VAL
MEM_VAL_GEN 19
.quad MEM_VAL
MEM_VAL_GEN 20
.quad MEM_VAL
MEM_VAL_GEN 21
.quad MEM_VAL
MEM_VAL_GEN 22
.quad MEM_VAL
MEM_VAL_GEN 23
.quad MEM_VAL
MEM_VAL_GEN 24
.quad MEM_VAL
MEM_VAL_GEN 25
.quad MEM_VAL
MEM_VAL_GEN 26
.quad MEM_VAL
MEM_VAL_GEN 27
.quad MEM_VAL
MEM_VAL_GEN 28
.quad MEM_VAL
MEM_VAL_GEN 29
.quad MEM_VAL
MEM_VAL_GEN 30
.quad MEM_VAL
MEM_VAL_GEN 31
.quad MEM_VAL
_array_end:
_store_data:
.quad 0x0000000000000000

.macro AS_CONCAT p1:req, p2:req r:vararg
		\p1\p2 , \r
.endm

.text
_start:		ADR	X0 , _array_start

// Test various sizes and sign extensions
.macro LDR_BATCH_TEST unit_test:req
		\unit_test  B,  0, UXTB, W
		\unit_test SB,  0, SXTB, W
		\unit_test  B,  7, UXTB, W
		\unit_test SB,  7, SXTB, W
		\unit_test SB,  7, SXTB, X

		\unit_test  H,  8, UXTH, W
		\unit_test SH,  8, SXTH, W
		\unit_test  H, 15, UXTH, W
		\unit_test SH, 15, SXTH, W
		\unit_test SH, 15, SXTH, X

		\unit_test   , 16, UXTW, W
		\unit_test   , 31, UXTW, W
		\unit_test SW, 31, SXTW, X

		\unit_test   , 24, UXTX, X
.endm

/******************************************************************************/

.macro LDUR_UNIT_TEST sx=NULL, idx:req, sign_ext:req, sz:req, rt=1
		IDX=\idx * 8
		MEM_VAL_GEN \idx
		MOV	X3 , X0
	.ifc \sx, NULL
		LDUR	\sz\rt, [X0,IDX]
	.else
		LDUR\sx	\sz\rt, [X0,IDX]
	.endif
		AS_MOVX	\sz,2 , \sign_ext, MEM_VAL
		CMP	X\rt  , X2
		BNE	_test_fail
		CMP	X3 , X0
		BNE	_test_fail
.endm

		LDR_BATCH_TEST LDUR_UNIT_TEST

/******************************************************************************/

.macro LDR_PRE_UNIT_TEST sx=NULL, idx:req, sign_ext:req, sz:req, rt=1
		IDX=\idx * 8
		MEM_VAL_GEN \idx
		ADD	X3 , X0 , IDX
	.ifc \sx, NULL
		LDR	\sz\rt, [X0,IDX]!
	.else
		LDR\sx	\sz\rt, [X0,IDX]!
	.endif
		AS_MOVX	\sz,2 , \sign_ext, MEM_VAL
		CMP	X\rt  , X2
		BNE	_test_fail
		CMP	X3 , X0
		BNE	_test_fail
		SUB	X0 , X0 , IDX
.endm

		LDR_BATCH_TEST LDR_PRE_UNIT_TEST

/******************************************************************************/

.macro LDR_POST_UNIT_TEST sx=NULL, idx:req, sign_ext:req, sz:req, rt=1
		IDX=\idx * 8
		MEM_VAL_GEN 0
		ADD	X3 , X0 , IDX
	.ifc \sx, NULL
		LDR	\sz\rt, [X0], IDX
	.else
		LDR\sx	\sz\rt, [X0], IDX
	.endif
		AS_MOVX	\sz,2 , \sign_ext, MEM_VAL
		CMP	X\rt  , X2
		BNE	_test_fail
		CMP	X3 , X0
		BNE	_test_fail
		SUB	X0 , X0 , IDX
.endm

		LDR_BATCH_TEST LDR_POST_UNIT_TEST

/******************************************************************************/
// Test Rn is SP
		ADD	SP , X0 , 12
		LDUR	X1 , [X0 , 12]
		LDUR	X2 , [SP]
		CMP	X1 , X2
		BNE	_test_fail

/******************************************************************************/
// Test Rt is RZR
		MOV	X1 , SP
		LDUR	XZR, [X0]
		CMP	SP , X1
		BNE	_test_fail

/******************************************************************************/
// Test sign extension of 9-bit immediate
		ADR	X10, _array_end
		LDUR	X1 , [X10, -256]
		LDUR	X2 , [X0]
		CMP	X1 , X2
		BNE	_test_fail

/******************************************************************************/
// Test persistence of stores

		ADR	X20, _store_data
		ADR	X21, _store_data+8
		MOV	X22, X20

.macro STORE_TEST inst:req, rt:req, val:req, inc:req
		AS_MOVX	X,0, UXTX , \val

l1\@:
	.ifc \rt, W
		\inst	W0, [X20], \inc
	.else
		\inst	X0, [X20], \inc
	.endif

		ROR	X0 , X0 , (\inc*4)
		ROR	X0 , X0 , (\inc*4)
		CMP	X20, X21
		BNE	l1\@

		LDR	X1 , _store_data
		CMP	X1 , X0
		BNE	_test_fail
l2\@:
	.ifc \rt, W
		\inst	WZR, [X20, -\inc]!
	.else
		\inst	XZR, [X20, -\inc]!
	.endif

		CMP	X20, X22
		BNE	l2\@

		LDR	X1 , _store_data
		CMP	X1 , XZR
		BNE	_test_fail
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
