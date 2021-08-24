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
		// Move immediate instruction
		MOVZ	X0, #0xff
		
		// Branch instruction
		B	_TEST
		MOVZ	X0, #0xffff //this should not happen if branch is successful
		AND		X1, X0, #0x1 //this should not happen if branch is successful
		AND		X1, X0, #0x2 //this should not happen if branch is successful
		
_TEST:
		// Logical immediate instruction 
		AND		X1, X0, #0x00003ffc00003ffc
		
		// Bitfield instruction
		LSL		X2, X1, #1
		MOV		X24, #-252
		ASR 	X25, X24, #3 // same as SBFM    X25, X24, #3, #63
		
		// Add instruction
		ADD 	X4, X1, X0
		ADD 	X23, X1, X0, LSL #2
		
		// Subtract and update condition flag instruction
		MOVZ	X5, #0x0
		SUBS 	X6, X5, #0x1
		
		// Store load instructions
		MOVZ	X10, #0
		STUR 	X0, [X10]
		LDUR 	X11, [X10]
		
		// Additional instructions - UBFM and EXTR instructions
		UBFM	X20, X1, #2, #3
		UBFM 	X21, X1, #3, #2
		EXTR	X22, X0, X1, #2
			
		YIELD
