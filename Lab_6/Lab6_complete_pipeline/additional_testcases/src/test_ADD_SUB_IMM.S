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

/* Test ADD, ADDS, SUB, SUBS

* Test flag setting on S
* Test flag correctness
* Test shift on immediate
* Test ADD/SUB result
* Test SP is X31/W31 target on ADD/SUB
* Test SP is X31/W31 source on all
* Test width modifier (X/W)

*/

IMM_CONST=0xBADDAD
.global _start
.text
_start:		MOV  X0 , 0
		ANDS X0 , X0, X0
		BNE  _test_fail

		ADD  X0 , X0, (IMM_CONST&0xFFF)
		CMP  X0 , (IMM_CONST&0xFFF)
		BNE  _test_fail

		LDR  X1 , _mem_const
		ADD  X0 , X0, (IMM_CONST&0xFFF000)
		CMP  X0 , X1
		BNE  _test_fail

		SUB  X0 , X0, (IMM_CONST&0xFFF)
		AND  X1 , X1, 0xFFF000
		CMP  X0 , X1
		BNE  _test_fail

		SUBS X0 , X0, (IMM_CONST&0xFFF000)
		BNE  _test_fail

		ORR  X0 , X0, 0xFFFFFFFF00000000
		CMP  W0 , 0
		BNE  _test_fail

		ADD  SP , X1, 0x0
		CMP  SP , X1
		BNE  _test_fail
		SUB  SP , SP, X1
		CMP  SP , 0
		BNE  _test_fail

		MOV  X1 , SP
		ADDS XZR, SP, 0x8
		CMP  SP , X1
		BNE  _test_fail

		SUBS WZR, WSP, 0x8
		CMP  WSP, W1
		BNE _test_fail

		YIELD
_test_fail:	BRK 0

.org 0x1000
.data
_mem_const: .quad IMM_CONST
