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

/** Test Arithmetic w/Carry register instructions

* Test RZR is source
* Test RZR is destination
* Test various widths (X/W)
* Test flag setting of ADCS/SBCS
* Test result corresponds to the state of C flag

*/

.macro UNIT_TEST op:req, reg_d:req, reg_n:req, reg_m:req, cmp:req
		\op  \reg_d , \reg_n , \reg_m
		CMP  \reg_d , \cmp
		BNE  _test_fail
.endm

.data
TEST_INIT=           0x0123456789ABCDEF
_TEST_INIT_SP: .quad 0xABABCDCDEFEF0101
_NEG_TEST_INIT:.quad -TEST_INIT
_TEST_INIT:    .quad TEST_INIT

.text
_start:		// Set the SP to some value != 0
		LDR  X2 , _TEST_INIT_SP
		MOV  SP , X2

		// Set X0 to another value != 0
		LDR  X0 , _TEST_INIT
		// Set X3 to -X0 (load a pre-assembled value)
		LDR  X3 , _NEG_TEST_INIT

		// Apply additions clearing carry every time (see UNIT_TEST: CMP)
		TST  X0 , X0
		UNIT_TEST ADC , X1 , X0 , XZR, X0
		TST  X0 , X0
		UNIT_TEST ADCS, X1 , X0 , XZR, X0
		TST  X0 , X0
		UNIT_TEST ADC , X1 , XZR, X0 , X0
		TST  X0 , X0
		UNIT_TEST ADCS, X1 , XZR, X0 , X0

		// Carry set by CMP

		// Apply subtractions
		UNIT_TEST SBC , X1 , XZR, X0 , X3
		UNIT_TEST SBCS, X1 , XZR, X0 , X3
		UNIT_TEST SBC , X1 , X0 , XZR, X0
		UNIT_TEST SBCS, X1 , X0 , XZR, X0

		MOVZ X3, 0
		//  Test with XZR as destination
		UNIT_TEST ADC , XZR, X0 , X0 , X3
		UNIT_TEST ADCS, XZR, X0 , X0 , X3
		UNIT_TEST SBC , XZR, X0 , X0 , X3
		UNIT_TEST SBCS, XZR, X0 , X0 , X3

		// Check SP hasn't changed for XZR
		CMP SP, X2
		BNE _test_fail

		// Test flag setting on a different width
		MOV  W3 , W0
		SBCS W1 , W0 , WZR
		BPL  _test_fail
		CMP  X1 , X3
		BNE  _test_fail

		// Test SBC with no carry
		MOV  X3 , -1
		TST  XZR, XZR
		UNIT_TEST SBC  X1 , X0 , X0 , X3
		TST  XZR, XZR
		UNIT_TEST SBCS W1 , W0 , W0 , W3
		
		// Test ADC with carry
		ADD  X3 , X0 , X0
		ADD  X3 , X3 , 1
		UNIT_TEST ADC  X1 , X0 , X0 , X3
		UNIT_TEST ADCS W1 , W0 , W0 , W3

		YIELD
_test_fail:	BRK 0

