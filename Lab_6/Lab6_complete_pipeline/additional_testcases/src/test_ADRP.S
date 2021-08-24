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

FRONT_LABEL_OFFSET =0x7000
BACK_LABEL_OFFSET  =0x1000

.global _start
.text
		B _start
_back_label:
.org BACK_LABEL_OFFSET
_start:		NOP
		ADRP X0 , _front_label          // Forward address
		CMP  X0 , FRONT_LABEL_OFFSET
		BNE  _test_fail

		ADRP X12, _back_label           // Backward address
		CMP  X12, XZR, UXTX
		BNE  _test_fail

		ADD  SP, X0 , X0
		MOV  X2, SP
		ADRP XZR, _front_label         // Zero register
		EOR  X1 , XZR, X0
		CMP  X1 , X0 , UXTX
		BNE  _test_fail
		CMP  SP , X2 , UXTX
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0
.org FRONT_LABEL_OFFSET
_front_label:
