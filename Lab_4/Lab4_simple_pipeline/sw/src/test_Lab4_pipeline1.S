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
.global _start
.text
_start:	
		MOVZ	X0, #0xf
		MOVZ	X1, #0xe
		MOVZ	X2, #0xd
		MOVZ	X3, #0xc
		MOVZ	X4, #0xb
		
		ADD		X5, X0, #1
		ADD		X6, X1, X2
		SUBS	X7, X0, X1
		
_test2:
		ADD 	X9, X1, X2 
		AND 	X10, X9, X3
		ORR 	X11, X5, X9
		SUB 	X12, X9, X7

		NOP
		NOP
		NOP
		NOP
				
		YIELD
				