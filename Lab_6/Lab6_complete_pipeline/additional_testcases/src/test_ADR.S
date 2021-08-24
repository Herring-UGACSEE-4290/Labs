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
.arch armv8-a+nofp+nocrc+nocrypto+noras+nosimd

LABEL_OFFSET=0xABC
START_OFFSET=0x0
.global _start
.text
.org START_OFFSET
_start:		ADR X1 , _target_label // Forward address
		CMP X1 , START_OFFSET+LABEL_OFFSET
		BNE _test_fail

		ADR X30, _start        // Backward address
		CMP X30, START_OFFSET
		BNE _test_fail

		ADD SP, X1 , X1
		MOV X2, SP
		ADR XZR, _target_label // Zero register
		EOR X0 , XZR, X1
		CMP X0 , X1 , UXTX
		BNE _test_fail
		CMP SP , X2 , UXTX
		BNE _test_fail
		YIELD
_test_fail:	BRK 0


.org _start+LABEL_OFFSET
_target_label:
