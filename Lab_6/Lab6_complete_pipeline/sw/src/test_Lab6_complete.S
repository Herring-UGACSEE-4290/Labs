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

//place move instructions here
	MOVZ	X0, #0x0100
	MOVZ	X1, #0x0101
	MOVZ	X5, #0x1
	MOVZ	X6, #0x2
	MOVZ	X7, #0x3
	MOVZ	X8, #0x4
	MOVZ	X9, #0x5
	MOVZ	X20, #0x013C

// store values in memory
	STURB 	W5, [X0]
	STURB 	W6, [X0, #1]
	STURB 	W7, [X0, #2]
	STURB 	W8, [X0, #3]
	STURB	WZR, [X0, #4]
	
// test load-store dependency
	LDURB 	W10, [X0]
	STURB 	W10, [X0, #5]

// test data dependency
	ADD 	X10, X9, #0 
	AND 	X11, X10, X7
	ORR 	X12, X8, X10
	SUB 	X13, X10, X5

// test load-use dependency
	LDURB 	W10, [X1]
	AND 	X11, X10, X7
	ORR 	X12, X8, X10
	SUB 	X13, X10, X5

// test Branch
	B 		_strcpyloop
	ADD		X11, X10, #100	// this shouldn't execute
	ADD		X11, X10, #200	// this shouldn't execute
	ADD		X11, X10, #300	// this shouldn't execute		
	
_strcpyloop: 
    LDRB	W2, [X0], #1  // Load byte into W2 from memory pointed to by X0 (*src)
    STRB 	W2, [X20], #1 // Store byte in W2 into memory pointed to by W2 (*dst)
    CMP   	X2, #0         // Was the byte 0? 
    BNE   	_strcpyloop    // If not, repeat the _strcpyloop
	
_test: 
	NOP
	NOP
	NOP
	NOP
	YIELD
