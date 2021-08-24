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

/** Test Conditional Compare Register instructions

* Test RZR is source
* Test PSTATE is updated
* Test various widths (X/W)
* Test condition check is done correctly

*/

.data
INITIAL_SP: .quad 0xAAAA0000AAAA0000
INITIAL_X0: .quad 0x0000555500005555

.text
_start:		LDR  X0 , INITIAL_SP
		MOV  SP , X0
		LDR  X0 , INITIAL_X0

		// Clear PSTATE
		TST  X0 , X0

		// Test RZR is the source for Rn
		CCMP XZR, X0 , # 0, AL
		BPL  _test_fail
		CCMN XZR, X0 , # 0, AL
		BMI  _test_fail

		// Test RZR is the source for Rm
		CCMP X0 , XZR, # 0, AL
		BMI  _test_fail
		CCMN X0 , XZR, # 0, AL
		BMI  _test_fail

		// Test with W register size
		MOV  W0 , W0 // Clear upper 32 bits
		CCMP W0 , W0 , # 0, AL
		BNE  _test_fail
		BCC  _test_fail

		// Test update of PSTATE via immediate
		CCMP XZR, XZR, #15, CC
		BVC  _test_fail
		BCC  _test_fail
		BNE  _test_fail
		BPL  _test_fail

		CCMN XZR, XZR, # 0, VC
		BVS  _test_fail
		BCS  _test_fail
		BEQ  _test_fail
		BMI  _test_fail

		CCMP XZR, XZR, #10, VS
		BVS  _test_fail
		BCC  _test_fail
		BEQ  _test_fail
		BPL  _test_fail

		CCMN XZR, XZR, # 5, VS
		BVC  _test_fail
		BCS  _test_fail
		BNE  _test_fail
		BMI  _test_fail

		YIELD
_test_fail:	BRK 0

