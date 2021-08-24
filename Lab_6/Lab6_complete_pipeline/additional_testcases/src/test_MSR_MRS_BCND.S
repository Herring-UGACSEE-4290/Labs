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

/** Test MRS, MSR and branch conditional instructions

* Test backward branch
* Test forward branch
* Test branch on various conditions set by MSR
* Test Rt is RZR for MSR/MRS

*/

.macro TEST_BCND sfx:req, nsfx:req, cond:req
	.if \cond
		B\sfx	l1\@
		BRK	0
l1\@:		B\nsfx	_test_fail
	.else
		B\nsfx	l2\@
		BRK	0
l2\@:		B\sfx	_test_fail
	.endif
.endm

#define N_FLAG 0x8
#define Z_FLAG 0x4
#define C_FLAG 0x2
#define V_FLAG 0x1

.macro TEST_NZCV val:req
	TEST_BCND MI, PL, (\val & N_FLAG)
	TEST_BCND EQ, NE, (\val & Z_FLAG)
	TEST_BCND CS, CC, (\val & C_FLAG)
	TEST_BCND VS, VC, (\val & V_FLAG)
	TEST_BCND HI, LS, ((\val & C_FLAG) && ((\val & Z_FLAG) == 0))
	TEST_BCND GE, LT, ((\val & N_FLAG) && 1) == ((\val & V_FLAG) && 1)
	TEST_BCND GT, LE, (((\val & N_FLAG) && 1) == ((\val & V_FLAG) && 1)) && ((\val & Z_FLAG) == 0)
	B	lp\@
	BRK	0
lp\@:
.endm

.macro TEST_PSTATE val:req
	MOV	W0 , (\val << 28)
	MSR	NZCV, X0

	TEST_NZCV \val
	MRS	X1, NZCV
	CMP	X0, X1
	BNE	_test_fail
.endm

.text
_start:

/*****************************************************************************/
// Test branching forward & backward

		B	_start_forw
		BRK	0

/*****************************************************************************/
// Test various PSTATE conditions

_start_back:	TEST_PSTATE 0
		TEST_PSTATE 1
		TEST_PSTATE 2
		TEST_PSTATE 3
		TEST_PSTATE 4
		TEST_PSTATE 5
		TEST_PSTATE 6
		TEST_PSTATE 7
		TEST_PSTATE 8
		TEST_PSTATE 9
		TEST_PSTATE 0xA
		TEST_PSTATE 0xB
		TEST_PSTATE 0xC
		TEST_PSTATE 0xD
		TEST_PSTATE 0xE
		TEST_PSTATE 0xF

/*****************************************************************************/
// Test RZR is Rt for MSR & MRS

		MOV	W0 , (0xA << 28)
		MOV	WSP, W0
		MSR	NZCV, XZR

		TEST_NZCV 0
		MRS	X1 , NZCV
		CMP	X1 , XZR
		BNE	_test_fail

		CMP	SP , X0
		BNE	_test_fail

		YIELD
_test_fail:	BRK	0

_start_forw:	B	_start_back
		BRK	0
