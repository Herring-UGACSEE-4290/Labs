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
.global _start

/** Test Logical shifted register instructions

* Test RZR is source
* Test RZR is destination
* Test various widths (X/W)
* Test negation of operand M
* Test flag setting of ANDS/BICS
* Test shift operation & value on operand M

*/

.data
TEST_LOGIC_INIT=0x0123456789ABCDEF
_TEST_LOGIC_INIT_SP: .quad 0xABABCDCDEFEF0101
_TEST_LOGIC_INIT:    .quad TEST_LOGIC_INIT
TEST_LOGIC_IMM_1=12 // LSL
_TEST_LOGIC_1:       .quad TEST_LOGIC_INIT&(TEST_LOGIC_INIT<<TEST_LOGIC_IMM_1)
TEST_LOGIC_IMM_2=16 // LSR
_TEST_LOGIC_2:       .quad TEST_LOGIC_INIT&(TEST_LOGIC_INIT>>TEST_LOGIC_IMM_2)
TEST_LOGIC_IMM_3=24 // ROR
_TEST_LOGIC_3:       .quad TEST_LOGIC_INIT&((TEST_LOGIC_INIT>>TEST_LOGIC_IMM_3)|(TEST_LOGIC_INIT<<(64-TEST_LOGIC_IMM_3)))
TEST_LOGIC_IMM_4=48 // ASR
_TEST_LOGIC_4:       .quad TEST_LOGIC_INIT&(TEST_LOGIC_INIT>>TEST_LOGIC_IMM_4)
TEST_LOGIC_IMM_4W=16 // ASR
_TEST_LOGIC_4W:      .quad 0xFFFFFFFF&TEST_LOGIC_INIT&((0xFFFFFFFF<<(32-TEST_LOGIC_IMM_4W))|(TEST_LOGIC_INIT>>TEST_LOGIC_IMM_4W))


.macro UNIT_TEST op:req, reg_d:req, reg_n:req, reg_m:req, cmp:req, shift:vararg
	.ifb \shift
		\op  \reg_d , \reg_n , \reg_m
	.else
		\op  \reg_d , \reg_n , \reg_m , \shift
	.endif

	.ifc \op, ANDS
		BCS  _test_fail
		BVS  _test_fail
	.endif

	.ifc \op, BICS
		BCS  _test_fail
		BVS  _test_fail
	.endif

		CMP  \reg_d , \cmp
		BNE  _test_fail
.endm

.text
_start:		// Set the SP to some value != 0
		LDR  X2 , _TEST_LOGIC_INIT_SP
		MOV  SP , X2

		// Set X0 to another value != 0
		LDR  X0 , _TEST_LOGIC_INIT

		// Don't trust the zero register
		MOVZ X3 , 0

		// Apply instructions
		//  XXX  X1 , X0 , XZR
		UNIT_TEST AND , X1 , X0 , XZR, X3
		UNIT_TEST ORR , X1 , X0 , XZR, X0
		UNIT_TEST EOR , X1 , X0 , XZR, X0
		UNIT_TEST ANDS, X1 , X0 , XZR, X3
		
		//  XXX  X1 , XZR, X0
		UNIT_TEST AND , X1 , XZR, X0 , X3
		UNIT_TEST ORR , X1 , XZR, X0 , X0
		UNIT_TEST EOR , X1 , XZR, X0 , X0
		UNIT_TEST ANDS, X1 , XZR, X0 , X3

		//  XXX  XZR, X0 , X0
		UNIT_TEST AND , XZR, X0, X0 , X3
		UNIT_TEST ORR , XZR, X0, X0 , X3
		UNIT_TEST EOR , XZR, X0, X0 , X3
		UNIT_TEST ANDS, XZR, X0, X0 , X3

		// Check SP hasn't changed for XZR
		CMP SP, X2
		BNE _test_fail

		// Set X0 to some value of width X
		// Apply instructions
		MOV  W3 , W0
		ANDS W1 , W0 , W0
		BPL  _test_fail
		CMP  X1 , X3
		BNE  _test_fail

		// Apply negated instructions
		MOV  X3 , 0xFFFFFFFFFFFFFFFF
		UNIT_TEST BIC , X1 , X0 , X0 , XZR
		UNIT_TEST ORN , X1 , X0 , X0 , X3
		UNIT_TEST EON , X1 , X0 , X0 , X3
		UNIT_TEST BICS, X1 , X0 , X0 , XZR

		// Apply various shifts on both widths
		LDR  X3 , _TEST_LOGIC_1
		UNIT_TEST ANDS, X1 , X0 , X0 , X3 , LSL #TEST_LOGIC_IMM_1
		LDR  X3 , _TEST_LOGIC_2
		UNIT_TEST ANDS, X1 , X0 , X0 , X3 , LSR #TEST_LOGIC_IMM_2
		LDR  X3 , _TEST_LOGIC_3
		UNIT_TEST ANDS, X1 , X0 , X0 , X3 , ROR #TEST_LOGIC_IMM_3
		LDR  X3 , _TEST_LOGIC_4
		UNIT_TEST ANDS, X1 , X0 , X0 , X3 , ASR #TEST_LOGIC_IMM_4
		LDR  X3 , _TEST_LOGIC_4W
		UNIT_TEST ANDS, W1 , W0 , W0 , W3 , ASR #TEST_LOGIC_IMM_4W
		CMP  X3 , X1
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0

