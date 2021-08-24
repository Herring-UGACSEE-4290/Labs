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

/** Test branch immediate and register instructions

* Test backward branch immediate
* Test forward branch immediate
* Test branch & link
* Test Rt is RZR for branch register

*/

.data
_test_state: .word 0x00

.text
_start:
		LDR	W0 , _test_state
		CMP	W0 , WZR
		BEQ	_test_b_imm
		ADD	W0 , W0 , 1
		STR	W0 , [SP, XZR]
		RET

/*****************************************************************************/
// Test branching forward & backward

_test_b_imm:	B	_test_b_forw
		BRK	0
		BRK	0
		BRK	0

_test_b_back:	ADR	X1 , .+8
		BL	_test_bl_forw
		BRK	0
		BRK	0
		BRK	0

_test_bl_back:	CMP	X30, X1
		BNE	_test_fail
		B	_test_br
_test_b_forw:	B	_test_b_back
		BRK	0
		BRK	0
		BRK	0

_test_bl_forw:	CMP	X30, X1
		BNE	_test_fail
		ADR	X1 , .+8
_test_bl_end:	BL	_test_bl_back
		BRK	0
		BRK	0
		BRK	0

/*****************************************************************************/
// Test branch register

_test_br:	ADR	X0 , _test_br_tgt
		BR	X0
		BRK	0
		BRK	0
		BRK	0

_test_br_tgt:	ADR	X0 , _test_blr_tgt
		ADR	X1 , .+8
		BLR	X0
		BRK	0
		BRK	0
		BRK	0

_test_blr_tgt:	CMP	X30, X1
		BNE	_test_fail
		ADR	X0 , _test_ret_tgt
		RET	X0
		BRK	0
		BRK	0
		BRK	0

_test_ret_tgt:
/*****************************************************************************/
// Test branch register Rt is RZR
		ADR	X1 , _test_state
		MOV	SP , X1
		MOV	W0 , 1
		STR	W0 , [SP, XZR]

		ADR	X30, .+8
		BR	XZR
		BLR	XZR
		ADR	X30, .+8
		RET	XZR

		LDR	W0 , [SP, XZR]
		CMP	W0 , 4
		BNE	_test_fail
		YIELD
_test_fail:	BRK	0
