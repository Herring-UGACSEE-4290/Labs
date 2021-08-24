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

/** Test Conditional Select instructions

* Test RZR is source
* Test RZR is destination
* Test various widths (X/W)
* Test condition check is done correctly
* Test CSEL, CSINC, CSINV, CSNEG

*/

.data
INITIAL_SP: .quad 0xFEDCBA9876543210
INITIAL_X0: .quad 0xDEADABBABABEF007
INITIAL_X1: .quad 0xBEEF0008DABBF879

.text
_start:		LDR	X2 , INITIAL_SP
		MOV	SP , X2
		LDR	X0 , INITIAL_X0
		LDR	X1 , INITIAL_X1

		// Clear PSTATE
		TST	X0 , X0

		// Test RZR is Rn
.macro RZR_RN_TEST inst:req
		\inst	X10, XZR, X0, AL
		CMP	X10, XZR
		BNE	_test_fail
.endm

		RZR_RN_TEST CSEL
		RZR_RN_TEST CSINC
		RZR_RN_TEST CSINV
		RZR_RN_TEST CSNEG

		// Test RZR is Rm
.macro RZR_RM_TEST inst:req
		\inst	X10, X0, XZR, VS
		CMP	X10, X11
		BNE	_test_fail
.endm

		MOV	X11, 0
		RZR_RM_TEST CSEL

		MOV	X11, 1
		RZR_RM_TEST CSINC

		MOV	X11, 0xFFFFFFFFFFFFFFFF
		RZR_RM_TEST CSINV

		MOV	X11, 0
		RZR_RM_TEST CSNEG

		// Test RZR is Rd (Destination)
.macro RZR_RD_TEST inst:req
		\inst	XZR, X1, X0, AL
		CMP	SP , X2
		BNE	_test_fail
		TST	XZR, XZR
		BNE	_test_fail
.endm

		RZR_RD_TEST CSEL
		RZR_RD_TEST CSINC
		RZR_RD_TEST CSINV
		RZR_RD_TEST CSNEG

		// Test selection on HI, GT & GE as well on inverse.
.macro PSTATE_CND_TEST CND_1:req, CND_2:req
		MOV	X10, XZR
		MOV	X20, XZR
		CMP	X11, X12
		CSEL	X10, X1 , X0, \CND_1
		CSEL	X20, X0 , X1, \CND_2
		CMP	X10, X1
		BNE	_test_fail
		CMP	X20, X1
		BNE	_test_fail
.endm

		MOV	X11, 0xF
		MOV	X12, 0x8
		PSTATE_CND_TEST HI, LS
		PSTATE_CND_TEST GT, LE
		PSTATE_CND_TEST GE, LT

		MOV	X11, 0x8000 << 48
		MOV	X12, 0x8
		PSTATE_CND_TEST HI, LS
		PSTATE_CND_TEST LE, GT
		PSTATE_CND_TEST LT, GE

		MOV	X11, X12
		PSTATE_CND_TEST LS, HI
		PSTATE_CND_TEST LE, GT
		PSTATE_CND_TEST GE, LT

		// Test on 32-bit width
		TST	XZR, XZR // Set Z flag

		MOV	W11, W0
		CSEL	W10, W0 , W1, EQ
		CMP	X10, X11
		BNE	_test_fail

		MOV	W11, W1
		CSEL	W10, W0 , W1, NE
		CMP	X10, X11
		BNE	_test_fail
		
		YIELD
_test_fail:	BRK 0

