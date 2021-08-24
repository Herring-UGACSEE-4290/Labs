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

#define TRUE  1
#define FALSE 0

.macro AS_SHIFT sz:req, val:req, op:req, shamt:req
	.ifc \sz, W
		SIGN=0x80000000
		REG_SZ=32
		OUT_MASK=0xFFFFFFFF
	.else
		SIGN=0x8000000000000000
		REG_SZ=64
		OUT_MASK=0xFFFFFFFFFFFFFFFF
	.endif

	.if (SIGN & \val)
		MASK=0xFFFFFFFFFFFFFFFF
	.else
		MASK=0x0
	.endif

	.ifc \op, ASR
		AS_SHIFT_OUT= (MASK<<(REG_SZ-\shamt)) | (\val>>\shamt)
	.endif
	.ifc \op, LSR
		AS_SHIFT_OUT= (\val>>\shamt)
	.endif
	.ifc \op, LSL
		AS_SHIFT_OUT= (\val<<\shamt)
	.endif

	AS_SHIFT_OUT=AS_SHIFT_OUT & OUT_MASK

.endm

.macro AS_ARITH_SH op:req, reg_sz:req, opa:req, opb:req, shift:vararg

	.ifnb \shift
		AS_SHIFT \reg_sz, \opb, \shift
		OPB_SH=AS_SHIFT_OUT
	.else
		OPB_SH=\opb
	.endif

	.ifc \op, ADD
		AS_ARITH_SH_OUT=\opa + OPB_SH
	.else
		AS_ARITH_SH_OUT=\opa - OPB_SH
	.endif

	.ifc \reg_sz, W
		AS_ARITH_SH_OUT=AS_ARITH_SH_OUT&0xFFFFFFFF
	.else
		AS_ARITH_SH_OUT=AS_ARITH_SH_OUT&0xFFFFFFFFFFFFFFFF
	.endif
		
.endm

.macro AS_ARITH_EXT op:req, reg_sz:req, opa:req, opb:req, opb_ext:req, lshift=0
	.ifc \opb_ext, UXTB
		EXT_MASK = 0xFF
		SIGN_EXT = 0
	.endif
	.ifc \opb_ext, SXTB
		EXT_MASK = 0xFF
		SIGN_EXT = 1
	.endif
	.ifc \opb_ext, UXTH
		EXT_MASK = 0xFFFF
		SIGN_EXT = 0
	.endif
	.ifc \opb_ext, SXTH
		EXT_MASK = 0xFFFF
		SIGN_EXT = 1
	.endif
	.ifc \opb_ext, UXTW
		EXT_MASK = 0xFFFFFFFF
		SIGN_EXT = 0
	.endif
	.ifc \opb_ext, SXTW
		EXT_MASK = 0xFFFFFFFF
		SIGN_EXT = 1
	.endif
	.ifc \opb_ext, UXTX
		EXT_MASK = 0xFFFFFFFFFFFFFFFF
		SIGN_EXT = 0
	.endif
	.ifc \opb_ext, SXTX
		EXT_MASK = 0xFFFFFFFFFFFFFFFF
		SIGN_EXT = 1
	.endif

	SIGN_BIT = ~(EXT_MASK>>1) & (EXT_MASK)

	OPB_VAL = EXT_MASK & \opb
	.if SIGN_EXT
		.if (SIGN_BIT & OPB_VAL)
			OPB_VAL = OPB_VAL | ~EXT_MASK
		.endif
	.endif

	OPB_VAL = OPB_VAL << \lshift

	.ifc \op, ADD
		AS_ARITH_EXT_OUT=\opa + OPB_VAL
	.else
		AS_ARITH_EXT_OUT=\opa - OPB_VAL
	.endif

	.ifc \reg_sz, W
		AS_ARITH_EXT_OUT=AS_ARITH_EXT_OUT&0xFFFFFFFF
	.else
		AS_ARITH_EXT_OUT=AS_ARITH_EXT_OUT&0xFFFFFFFFFFFFFFFF
	.endif
	
.endm

.macro AS_MOVX sz:req, reg:req, ext:req, value:req

	AS_ARITH_EXT ADD, \sz, 0, \value, \ext

        .ifc \sz, X
                MOVK    \sz\reg, (AS_ARITH_EXT_OUT>>48)&0xFFFF, LSL 48
                MOVK    \sz\reg, (AS_ARITH_EXT_OUT>>32)&0xFFFF, LSL 32
        .endif  
                MOVK    \sz\reg, (AS_ARITH_EXT_OUT>>16)&0xFFFF, LSL 16
                MOVK    \sz\reg, (AS_ARITH_EXT_OUT>> 0)&0xFFFF, LSL 0
.endm


