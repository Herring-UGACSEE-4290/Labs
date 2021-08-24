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

/* Test logical immediate operations

 * Test various immediate values
 * Test RZR is always the source
 * Test RZR is destination on ANDS
 * Test various immediate masks
 * Test Flag setting on ANDS
 * Test on various operand width (X/W)

*/

.global _start
.text

.macro TESTMASK imm:req
		ORR  X0 , X0, \imm
		MOV  X1 , X0
		ANDS X0 , X0, \imm
		BEQ  _test_fail
		CMP  X1 , X0, UXTX
		BNE  _test_fail
		AND  X0 , X1, \imm
		CMP  X1 , X0, UXTX
		BNE  _test_fail
		EOR  X0 , X0, \imm
		TST  X0 , X0
		BNE  _test_fail
.endm

IMM_CONST=0xAAAAAAAAAAAAAAAA

_start:		MOV  X0 , 0x0       // Initialise
		TST  X0 , X0
		BNE  _test_fail

		TESTMASK 0x1
		TESTMASK 0x2
		TESTMASK 0x0000800000000000
		TESTMASK 0x8000000000000000
		TESTMASK 0xFFFF0000FFFF0000
		TESTMASK 0xFFFFFFFF00000000

		ORR  SP , XZR, IMM_CONST
		ANDS XZR, XZR, ~IMM_CONST
		LDR  X0 , MEM_CONST
		CMP  SP , X0
		BNE  _test_fail
		TST  XZR, XZR
		BNE  _test_fail
		EOR  WSP, W0 , (~IMM_CONST & 0xFFFFFFFF)
		MOV  X0 , SP
		EOR  X0 , X0, 0xFFFFFFFF
		TST  X0 , X0
		BNE  _test_fail

		YIELD

_test_fail:	BRK 0

.data
MEM_CONST: .quad IMM_CONST
