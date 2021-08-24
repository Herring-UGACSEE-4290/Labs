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
`ifndef EDUCORE_HEAD
`define EDUCORE_HEAD

`define DATA_MEMORY_SIZE_BYTE  2'b00
`define DATA_MEMORY_SIZE_WORD  2'b01
`define DATA_MEMORY_SIZE_DWORD 2'b10
`define DATA_MEMORY_SIZE_QWORD 2'b11

`define I_TYPE_RESERVED    32'bxxx0_000x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
`define I_TYPE_DATA_IMM    32'bxxx1_00xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_PC_REL_ADDR   32'bxxxx_xx00_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_ADD_SUB       32'bxxxx_xx01_0xxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_LOGIC         32'bxxxx_xx10_0xxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_MOVE          32'bxxxx_xx10_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_BITFIELD      32'bxxxx_xx11_0xxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_IMM_EXTRACT       32'bxxxx_xx11_1xxx_xxxx_xxxx_xxxx_xxxx_xxxx
`define I_TYPE_DATA_REG    32'bxxxx_101x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_REG_LOGIC_SHIFT   32'bxxx0_xxx0_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_REG_ADD_SUB_SHIFT 32'bxxx0_xxx1_xx0x_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_REG_ADD_SUB_EXT   32'bxxx0_xxx1_xx1x_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_DATA_REG_ADD_SUB_WC    32'bxxx1_xxx0_000x_xxxx_0000_00xx_xxxx_xxxx
	`define I_TYPE_DATA_REG_COND_COMP_REG 32'bxxx1_xxx0_010x_xxxx_xxxx_0xxx_xxxx_xxxx
	`define I_TYPE_DATA_REG_COND_SEL      32'bxxx1_xxx0_100x_xxxx_xxxx_xxxx_xxxx_xxxx
`define I_TYPE_LDST        32'bxxxx_1x0x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_LDST_LOAD_LITERAL      32'bxx01_xxx0_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_LDST_UNSCALED_IMM      32'bxx11_xxx0_xx0x_xxxx_xxxx_00xx_xxxx_xxxx
	`define I_TYPE_LDST_POST_IDX          32'bxx11_xxx0_xx0x_xxxx_xxxx_01xx_xxxx_xxxx
	`define I_TYPE_LDST_PRE_IDX           32'bxx11_xxx0_xx0x_xxxx_xxxx_11xx_xxxx_xxxx
	`define I_TYPE_LDST_REG_OFFSET        32'bxx11_xxx0_xx1x_xxxx_xxxx_10xx_xxxx_xxxx
`define I_TYPE_BRANCH_SYS  32'bxxx1_01xx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_BRANCH_SYS_CND_BR      32'b010x_xx0x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_BRANCH_SYS_HINTS       32'b110x_xx01_0000_0011_0010_xxxx_xxx1_1111
	`define I_TYPE_BRANCH_SYS_REG_MOVE    32'b110x_xx01_00x1_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_BRANCH_SYS_UCND_BR_REG 32'b110x_xx1x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx
	`define I_TYPE_BRANCH_SYS_UCND_BR_IMM 32'bx00x_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx_xxxx

`define ERROR_OK              4'h0
`define ERROR_UNDEFINED       4'h1
`define ERROR_YIELD           4'h2

`define NEXT_PC_ADD 1'b0
`define NEXT_PC_RN  1'b1

`define PC_OP_NEXT  1'b0
`define PC_OP_COND  1'b1

`define INST_NOP 32'hD503_201F

`define FEXEC_N_PC      2'b00
`define FEXEC_N_PC_PAGE 2'b01
`define FEXEC_N_RN      2'b10

`define FEXEC_M_IMM 1'b0
`define FEXEC_M_RM  1'b1

`define BR_COND_PSTATE  1'b0
`define BR_COND_UNCOND  1'b1

`define LDST_SIZE_08 2'b00
`define LDST_SIZE_16 2'b01
`define LDST_SIZE_32 2'b10
`define LDST_SIZE_64 2'b11

`define ST_U_SZ 2'b00
`define LD_U_SZ 2'b01
`define LD_S_64 2'b10
`define LD_S_32 2'b11

`define MEM_ADDR_ALU 1'b0
`define MEM_ADDR_RN  1'b1

`define ALU_OP_B_BITEXT  1'b0
`define ALU_OP_B_WMASK   1'b1

`define ALU_OP_A_ZERO   2'b00
`define ALU_OP_A_EXEC_A 2'b01
`define ALU_OP_A_EXEC_N 2'b10

`define ALU_CMD_LOGIC 1'b0
`define ALU_CMD_AND   2'b00
`define ALU_CMD_ORR   2'b01
`define ALU_CMD_EOR   2'b10
`define ALU_CMD_ANDS  2'b11

`define ALU_CMD_ADD   1'b1
`define ALU_CMD_ADD_I 2'b10
`define ALU_CMD_ADD_0 2'b00
`define ALU_CMD_ADD_1 2'b01
`define ALU_CMD_ADD_C 2'b10

`define BARREL_IN_N  1'b0
`define BARREL_IN_M  1'b1

`define BARREL_U_IN_N 1'b0
`define BARREL_U_IN_L 1'b1

`define BARREL_OP_LSL  2'b00
`define BARREL_OP_LSR  2'b01
`define BARREL_OP_ASR  2'b10
`define BARREL_OP_ROR  2'b11

`define PSTATE_IN_ALU 2'b00
`define PSTATE_IN_REG 2'b01
`define PSTATE_IN_CND 2'b10 // Redirects to condition flags? ALU : NZCV

`define EX_OUT_PC_4   2'b00
`define EX_OUT_ALU    2'b01
`define EX_OUT_CND    2'b10
`define EX_OUT_CTRL   2'b11

`endif
