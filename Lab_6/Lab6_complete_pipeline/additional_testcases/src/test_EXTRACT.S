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

/** Test Extract Instruction
* Source is RZR
* Destination is RZR
* Various widths (X/W)
* LSB is ZERO
*/

.data
_EXTR_TEST_INIT: .quad 0x0123456789ABCDEF
_EXTR_TEST_RINIT:.quad 0xFEDCBA9876543210
EXTR_TEST_IMM_1=32
_EXTR_TEST_1_M:  .quad 0x89ABCDEF00000000
_EXTR_TEST_1_N:  .quad 0x0000000001234567
EXTR_TEST_IMM_2=0
_EXTR_TEST_2_M:  .quad 0x0000000000000000
_EXTR_TEST_2_N:  .quad 0x0123456789ABCDEF
EXTR_TEST_IMM_W=8
_EXTR_TEST_W_M:  .quad 0x00000000EF765432
_EXTR_TEST_W_N:  .quad 0x000000001089ABCD

.global _start
.text
_start:		LDR  X30, _EXTR_TEST_RINIT
		LDR  X0 , _EXTR_TEST_INIT
		MOV  SP , X30
		
		LDR  X2 , _EXTR_TEST_1_M
		EXTR X1 , X0 , XZR, EXTR_TEST_IMM_1
		CMP  X1 , X2
		BNE  _test_fail
		LDR  X2 , _EXTR_TEST_1_N
		EXTR X1 , XZR, X0 , EXTR_TEST_IMM_1
		CMP  X1 , X2
		BNE  _test_fail

		LDR  X2 , _EXTR_TEST_2_M
		EXTR X1 , X0 , XZR, EXTR_TEST_IMM_2
		CMP  X1 , X2
		BNE  _test_fail
		LDR  X2 , _EXTR_TEST_2_N
		EXTR X1 , XZR, X0 , EXTR_TEST_IMM_2
		CMP  X1 , X2
		BNE  _test_fail

		// Any number greater than 0 would do
		EXTR XZR, X0 , X0 , 11  
		CMP  SP , X30
		BNE  _test_fail
		CMP  SP , X30
		BNE  _test_fail	

		LDR  X2 , _EXTR_TEST_W_M
		EXTR W1 , W0 , W30, EXTR_TEST_IMM_W
		CMP  X1 , X2
		BNE  _test_fail
		LDR  X2 , _EXTR_TEST_W_N
		EXTR W1 , W30, W0 , EXTR_TEST_IMM_W
		CMP  X1 , X2
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0
