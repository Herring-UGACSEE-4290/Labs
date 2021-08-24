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

/** Test Load literal instructions

* Test backward address
* Test forward address
* Test sign extension
* Test W register size
* Test Rt is RZR

*/

BACK_LD_VAL=    0x0123456789ABCDEF
FORW_LD_VAL=    0x0011223344556677

.macro MOVX sz:req, reg:req, value:req
	AS_MOVX	\sz, \reg, UXTX, \value
.endm

.text
_start:		B _start_skip

_BACK_LD:	.quad BACK_LD_VAL

_start_skip:	// Test backward address
		LDR	X0 , _BACK_LD
		MOVX	X,1,  BACK_LD_VAL
		CMP	X0 , X1
		BNE	_test_fail

		// Test forward address
		LDR	X0 , _FORW_LD
		MOVX	X,1,  FORW_LD_VAL
		CMP	X0 , X1
		BNE	_test_fail

		// Test W register size
		LDR	W0 , _FORW_LD
		MOVX	W,1,  FORW_LD_VAL
		CMP	X0, X1
		BNE	_test_fail

		// Test RZR is Rt
		MOV	SP , X0
		LDR	XZR, _BACK_LD
		CMP	SP , X0
		BNE	_test_fail
		NOP
		NOP
		// Check again after pipeflush
		CMP	SP , X0
		BNE	_test_fail

		MOV	X2 , 0
		// Test sign extension
		LDRSW	X0 , _BACK_LD
		MOVX	X,1,  BACK_LD_VAL
		ADD	X1 , X2, W1, SXTW
		CMP	X0 , X1
		BNE	_test_fail

		LDRSW	X0 , _FORW_LD
		MOVX	X,1,  FORW_LD_VAL
		ADD	X1 , X2, W1, SXTW
		CMP	X0 , X1
		BNE	_test_fail

		YIELD
_test_fail:	BRK	0

.data
_FORW_LD:	.quad FORW_LD_VAL
