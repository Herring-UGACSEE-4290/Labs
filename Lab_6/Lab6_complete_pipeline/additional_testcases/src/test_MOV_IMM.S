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

/* Test MOV immediate instructions

 * Destination is always RZR
 * Test MOVN
 * Test MOVK
 * Test various widths (W/X)

*/

MOV_IMM_1=0xAAAA
MOV_LSL_1=0x0
MOV_IMM_2=0xBBBB
MOV_LSL_2=16
MOV_IMM_3=0xCCCC
MOV_LSL_3=32
MOV_IMM_4=0xDDDD
MOV_LSL_4=48
MVN_IMM_1=0xEEEE
MVN_LSL_1=48
MVN_IMM_2=0xFFFF
MVN_LSL_2=16
MVK_IMM_1=0xFFFF
MVK_LSL_1=16
MVK_IMM_2=0x5555
MVK_LSL_2=0

.global _start
.text
_start:		ADR  X2 , _MOV_MEM

		MOVZ X0 , MOV_IMM_1, LSL #MOV_LSL_1
		LDUR X1 , [X2]
		CMP  X0 , X1
		BNE  _test_fail

		MOVZ X0 , MOV_IMM_2, LSL #MOV_LSL_2
		LDUR X1 , [X2, 8]
		CMP  X0 , X1
		BNE  _test_fail

		MOVZ X0 , MOV_IMM_3, LSL #MOV_LSL_3
		LDUR X1 , [X2, 16]
		CMP  X0 , X1
		BNE  _test_fail

		MOVZ X0 , MOV_IMM_4, LSL #MOV_LSL_4
		LDUR X1 , [X2, 24]
		CMP  X0 , X1
		BNE  _test_fail

		MOVN X0 , MVN_IMM_1, LSL #MVN_LSL_1
		LDUR X1 , [X2, 32]
		CMP  X0 , X1
		BNE  _test_fail

		MOVN X0 , MVN_IMM_2, LSL #MVN_LSL_2
		LDUR X1 , [X2, 40]
		CMP  X0 , X1
		BNE  _test_fail

		MOVK X0 , MVK_IMM_1, LSL #MVK_LSL_1
		LDUR X1 , [X2, 48]
		CMP  X0 , X1
		BNE  _test_fail

		MOVK W0 , MVK_IMM_2, LSL #MVK_LSL_2
		LDUR X1 , [X2, 56]
		CMP  X0 , X1
		BNE  _test_fail

		MOV  SP , 0xAAAAAAAAAAAAAAAA
		MOV  X0 , SP
		MOVZ XZR, 0xFFFF
		CMP  SP , X0
		BNE  _test_fail
		TST  XZR, X0
		BNE  _test_fail
		TST  X0 , XZR
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0

.data
_MOV_MEM: 
.quad (MOV_IMM_1<<MOV_LSL_1)
.quad (MOV_IMM_2<<MOV_LSL_2)
.quad (MOV_IMM_3<<MOV_LSL_3)
.quad (MOV_IMM_4<<MOV_LSL_4)
.quad ~(MVN_IMM_1<<MVN_LSL_1)
.quad ~(MVN_IMM_2<<MVN_LSL_2)
.quad ((~(MVN_IMM_2<<MVN_LSL_2) & ~(0xFFFF << MVK_LSL_1)) | (MVK_IMM_1 << MVK_LSL_1))
.quad (MVK_IMM_1 << MVK_LSL_1) | (MVK_IMM_2 << MVK_LSL_2)

