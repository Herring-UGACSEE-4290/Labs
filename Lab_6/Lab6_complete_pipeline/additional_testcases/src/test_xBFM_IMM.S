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

/** Test Bitfield move instructions

* Test Destination is always XZR
* Test Source is always XZR
* Test Sign extension of SBFM
* Test Zero extension of UBFM
* Test Retain of Rd in BFM
* Test IMMS > IMMR
* Test IMMS < IMMR
* Test IMMS = IMMR
* Test various widths (X/W)

*/
.global _start

.data
_BFM_INITIAL: .quad 0x0123456789ABCDEF
_BFM_TEST_1_IMMR=40
_BFM_TEST_1_IMMS=55
_BFM_TEST_1:  .quad 0x0123456789AB2345
_BFM_TEST_2_IMMR=16
_BFM_TEST_2_IMMS=15
_BFM_TEST_2:  .quad 0xCDEF456789ABCDEF
_BFM_TEST_3_IMMR=47
_BFM_TEST_3_IMMS=47
_BFM_TEST_3:  .quad 0x0123456789ABCDEE

_UBFM_INITIAL: .quad 0x0123456789ABCDEF
_UBFM_TEST_1_IMMR=32
_UBFM_TEST_1_IMMS=55
_UBFM_TEST_1:  .quad 0x0000000000234567
_UBFM_TEST_2_IMMR=60
_UBFM_TEST_2_IMMS=7
_UBFM_TEST_2:  .quad 0x0000000000000EF0
_UBFM_TEST_3_IMMR=63
_UBFM_TEST_3_IMMS=63
_UBFM_TEST_3:  .quad 0x0000000000000000

_SBFM_INITIAL: .quad 0x0123456789ABCDEF
_SBFM_TEST_1_IMMR=32
_SBFM_TEST_1_IMMS=47
_SBFM_TEST_1:  .quad 0x0000000000004567
_SBFM_TEST_2_IMMR=60
_SBFM_TEST_2_IMMS=11
_SBFM_TEST_2:  .quad 0xFFFFFFFFFFFFDEF0
_SBFM_TEST_3_IMMR=19
_SBFM_TEST_3_IMMS=19
_SBFM_TEST_3:  .quad 0xFFFFFFFFFFFFFFFF
_SBFM_TEST_W_IMMR=28
_SBFM_TEST_W_IMMS=11
_SBFM_TEST_W:  .quad 0x00000000FFFFDEF0

.macro UNIT_TEST inst:req, immr:req, imms:req, expectation:req
	.ifc \inst, BFM
		MOV   X0 , X1
	.endif
		\inst X0, X1, \immr, \imms
		LDR   X2, \expectation
		CMP   X0, X2
		BNE   _test_fail
.endm

.text
_start:		MOV  SP , 0xAAAAAAAAAAAAAAAA
		MOV  X0 , 0x5555555555555555
		MOV  X1 , SP
		BFM  XZR, X0 ,  0, 63
		CMP  SP , X1
		BNE  _test_fail
		TST  X0 , XZR
		BNE  _test_fail
		BFM  X0 , XZR,  0, 63
		TST  X0 , X0
		BNE  _test_fail
		////////////////////////////////////////////////////////////////

		LDR  X0 , _BFM_INITIAL
		MOV  X1 , X0

		UNIT_TEST BFM, _BFM_TEST_1_IMMR, _BFM_TEST_1_IMMS, _BFM_TEST_1
		UNIT_TEST BFM, _BFM_TEST_2_IMMR, _BFM_TEST_2_IMMS, _BFM_TEST_2
		UNIT_TEST BFM, _BFM_TEST_3_IMMR, _BFM_TEST_3_IMMS, _BFM_TEST_3

		////////////////////////////////////////////////////////////////

		LDR  X0 , _UBFM_INITIAL
		MOV  X1 , X0

		UNIT_TEST UBFM, _UBFM_TEST_1_IMMR, _UBFM_TEST_1_IMMS, _UBFM_TEST_1
		UNIT_TEST UBFM, _UBFM_TEST_2_IMMR, _UBFM_TEST_2_IMMS, _UBFM_TEST_2
		UNIT_TEST UBFM, _UBFM_TEST_3_IMMR, _UBFM_TEST_3_IMMS, _UBFM_TEST_3
		////////////////////////////////////////////////////////////////

		LDR  X0 , _SBFM_INITIAL
		MOV  X1 , X0

		UNIT_TEST SBFM, _SBFM_TEST_1_IMMR, _SBFM_TEST_1_IMMS, _SBFM_TEST_1
		UNIT_TEST SBFM, _SBFM_TEST_2_IMMR, _SBFM_TEST_2_IMMS, _SBFM_TEST_2
		UNIT_TEST SBFM, _SBFM_TEST_3_IMMR, _SBFM_TEST_3_IMMS, _SBFM_TEST_3
		////////////////////////////////////////////////////////////////

		SBFM W0 , W1, _SBFM_TEST_W_IMMR, _SBFM_TEST_W_IMMS
		LDR  X2 , _SBFM_TEST_W
		CMP  X0 , X2
		BNE  _test_fail

		YIELD
_test_fail:	BRK 0
