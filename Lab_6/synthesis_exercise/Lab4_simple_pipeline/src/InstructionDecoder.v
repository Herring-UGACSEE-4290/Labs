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
`include "Educore.vh"

module InstructionDecoder (
	input  [31:0] I, // Instruction to be decoded

	output reg  [4:0] read_reg_an,
	output reg  [4:0] read_reg_am,
	output reg  [4:0] read_reg_aa,
	output reg        read_n_sp,
	output reg  [1:0] exec_n_mux,
	output reg        exec_m_mux,
	output reg [63:0] immediate,

	// To Execute stage
	output reg  [5:0] shamt,
	output reg  [5:0] imm_sz,
	output reg        imm_n,
	output reg        FnH,
	output reg  [1:0] barrel_op,
	output reg        barrel_in_mux,
	output reg        barrel_u_in_mux,
	output reg        bitext_sign_ext,
	output reg  [1:0] alu_op_a_mux,
	output reg        alu_op_b_mux,
	output reg        wt_mask,
	output reg        alu_invert_b,
	output reg  [2:0] alu_cmd,
	output reg  [1:0] out_mux,
	output reg  [3:0] condition,
	output reg        pstate_en,
	output reg  [1:0] pstate_mux,
	output reg        br_condition_mux,
	output reg        nextPC_mux,
	output reg        PC_add_op_mux,

	// to memory stage
	output reg  [1:0] mem_size,
	output reg        mem_sign_ext,
	output reg        mem_read,
	output reg        mem_write,
	output reg        mem_addr_mux,
	output reg        load_FnH,

	// to writeback stage
	output reg  [4:0] wload_addr,
	output reg  [4:0] write_addr,
	output reg        wload_en,
	output reg        write_en,

	// Error detection
	output reg [3:0] decode_err
);

wire        pc_rel_op      = I[31];
wire [20:0] pc_rel_imm     = {I[23:5],I[30:29]};

wire [11:0] add_sub_imm    = I[21:10];
wire        add_sub_imm_sh = I[22];
wire        add_sub_op     = I[30];

wire  [1:0] imm_opc        = I[30:29];
wire  [5:0] imm_imms       = I[15:10];
wire  [5:0] imm_immr       = I[21:16];
wire        imm_N          = I[22];

wire [15:0] mov_imm        = I[20:5];
wire  [1:0] mov_hw         = I[22:21];

wire  [5:0] reg_src_op     = I[15:10];
wire  [4:0] reg_src_op2    = I[20:16];
wire  [1:0] reg_src_shift  = I[23:22];
wire        logic_inv      = I[21];
wire  [2:0] ext_option     = I[15:13];

wire        sys_mov_l      = I[21];
wire        ucnd_bl_imm    = I[31];
wire        ucnd_bl_reg    = I[21];
wire [25:0] ucnd_br_imm    = I[25:0];
wire [13:0] tst_br_imm     = I[18:5];
wire  [5:0] tst_br_bit     = {I[31],I[23:19]};

wire  [1:0] ldst_opc_lit   = I[31:30];
wire  [1:0] ldst_opc       = I[23:22];
wire  [1:0] ldst_size      = I[31:30];
wire        ldst_shift     = I[12];
wire [18:0] ldst_lit_imm   = I[23:5];
wire  [8:0] ldst_imm9      = I[20:12];

wire        reg_sf         = I[31];
wire        flag_s         = I[29];
wire  [4:0] rd_addr        = I[4:0];
wire  [4:0] rm_addr        = I[20:16];
wire  [4:0] rn_addr        = I[9:5];

/******************************************************************************/
always @ ( * ) begin
	// Register file read
	read_reg_an = rn_addr;
	read_reg_am = rm_addr;
	read_reg_aa = rd_addr;

	// Register load write
	wload_addr  = rd_addr;
end

// Stack pointer read (n port)
always @ ( * ) casex(I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_ADD_SUB:       read_n_sp = 1;
		default:                        read_n_sp = 0;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_ADD_SUB_EXT:   read_n_sp = 1;
		default:                        read_n_sp = 0;
	endcase
	`I_TYPE_LDST:                       read_n_sp = 1;
	default:                            read_n_sp = 0;
endcase

// Register file write enable execute channel
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_ADD_SUB:       write_en = ~flag_s | ~&rd_addr;
		`I_TYPE_DATA_IMM_LOGIC:         write_en = ~&imm_opc | ~&rd_addr;
		default:                        write_en = ~&rd_addr;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_ADD_SUB_EXT:   write_en = ~flag_s | ~&rd_addr;
		`I_TYPE_DATA_REG_COND_COMP_REG: write_en = 0;
		default:                        write_en = ~&rd_addr;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_PRE_IDX:           write_en = 1;
		`I_TYPE_LDST_POST_IDX:          write_en = 1;
		default:                        write_en = 0;
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_REG_MOVE:    write_en = sys_mov_l & ~&rd_addr;
		`I_TYPE_BRANCH_SYS_UCND_BR_IMM: write_en = ucnd_bl_imm;
		`I_TYPE_BRANCH_SYS_UCND_BR_REG: write_en = ucnd_bl_reg;
		default:                        write_en = 0;
	endcase
	default:                            write_en = 0;
endcase

// Register file write address execute channel
always @ ( * ) casex (I)
	`I_TYPE_LDST:                       write_addr = rn_addr;
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_UCND_BR_IMM: write_addr = 5'h1E;
		`I_TYPE_BRANCH_SYS_UCND_BR_REG: write_addr = 5'h1E;
		default:                        write_addr = rd_addr;
	endcase
	default:                            write_addr = rd_addr;
endcase

// Register file write enable load channel
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      wload_en = ~&rd_addr;
		default:                        wload_en = |ldst_opc & ~&rd_addr;
	endcase
	default:                            wload_en = 0;
endcase

// Execute pipeline register n multiplexer
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_PC_REL_ADDR:   exec_n_mux = {1'b0, pc_rel_op};
		default:                        exec_n_mux = `FEXEC_N_RN;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      exec_n_mux = `FEXEC_N_PC;
		default:                        exec_n_mux = `FEXEC_N_RN;
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_UCND_BR_REG: exec_n_mux = `FEXEC_N_RN;
		default:                        exec_n_mux = `FEXEC_N_PC;
	endcase
	default:                            exec_n_mux = `FEXEC_N_RN;
endcase

// Execute pipeline register m multiplexer
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_EXTRACT:       exec_m_mux = `FEXEC_M_RM;
		default:                        exec_m_mux = `FEXEC_M_IMM;
	endcase
	`I_TYPE_DATA_REG:                   exec_m_mux = `FEXEC_M_RM;
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_REG_OFFSET:        exec_m_mux = `FEXEC_M_RM;
		default:                        exec_m_mux = `FEXEC_M_IMM;
	endcase
	default:                            exec_m_mux = `FEXEC_M_IMM;
endcase

// Instruction immediate value
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_PC_REL_ADDR:   immediate = pc_rel_op? {{31{pc_rel_imm[20]}},pc_rel_imm, 12'h000} : {{43{pc_rel_imm[20]}},pc_rel_imm};
		`I_TYPE_DATA_IMM_ADD_SUB:       immediate = add_sub_imm_sh? {add_sub_imm, 12'h000} : add_sub_imm;
		`I_TYPE_DATA_IMM_MOVE:          immediate = mov_imm;
		default:                        immediate = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      immediate = {{43{ldst_lit_imm[18]}}, ldst_lit_imm, 2'b00};
		default:                        immediate = {{55{ldst_imm9[8]}},ldst_imm9};
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_UCND_BR_IMM: immediate = {{38{ucnd_br_imm[25]}}, ucnd_br_imm, 2'b00};
		default:                        immediate = {{43{ldst_lit_imm[18]}}, ldst_lit_imm, 2'b00};
	endcase
	default:                            immediate = rm_addr;
endcase

// Decoded immediate length
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         imm_sz = imm_imms;
		`I_TYPE_DATA_IMM_MOVE:          imm_sz = 6'h0F;
		`I_TYPE_DATA_IMM_BITFIELD:      imm_sz = imm_imms;
		default:                        imm_sz = 6'h3F;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_ADD_SUB_EXT:   imm_sz = {&ext_option[1:0],ext_option[1],|ext_option[1:0], 3'h7};
		default:                        imm_sz = 6'h3F;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_REG_OFFSET:        imm_sz = {ext_option[0],5'h1F};
		default:                        imm_sz = 6'h3F;
	endcase
	default:                            imm_sz = 6'h3F;
endcase

// Decoded immediate length 'N' bit
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         imm_n = imm_N;
		`I_TYPE_DATA_IMM_BITFIELD:      imm_n = imm_N;
		default:                        imm_n = FnH;
	endcase
	default:                            imm_n = FnH;
endcase

// Barrel shift amount
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         shamt = imm_immr;
		`I_TYPE_DATA_IMM_BITFIELD:      shamt = imm_immr;
		`I_TYPE_DATA_IMM_MOVE:          shamt = {mov_hw, 4'h0};
		`I_TYPE_DATA_IMM_EXTRACT:       shamt = imm_imms;
		default:                        shamt = 6'h00;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:   shamt = imm_imms;
		`I_TYPE_DATA_REG_ADD_SUB_SHIFT: shamt = imm_imms;
		`I_TYPE_DATA_REG_ADD_SUB_EXT:   shamt = {3'h0,imm_imms[2:0]};
		default:                        shamt = 6'h00;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_REG_OFFSET:        shamt = {2{ldst_shift}} & ldst_size;
		default:                        shamt = 6'h00;
	endcase
	default:                            shamt = 6'h00;
endcase

// Barrel shift operation
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         barrel_op = `BARREL_OP_ROR;
		`I_TYPE_DATA_IMM_BITFIELD:      barrel_op = `BARREL_OP_ROR;
		`I_TYPE_DATA_IMM_EXTRACT:       barrel_op = `BARREL_OP_ROR;
		default:                        barrel_op = `BARREL_OP_LSL;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:   barrel_op = reg_src_shift;
		`I_TYPE_DATA_REG_ADD_SUB_SHIFT: barrel_op = reg_src_shift;
		default:                        barrel_op = `BARREL_OP_LSL;
	endcase
	default:                            barrel_op = `BARREL_OP_LSL;
endcase

// Barrel lower operand
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_BITFIELD:      barrel_in_mux = `BARREL_IN_N;
		default:                        barrel_in_mux = `BARREL_IN_M;
	endcase
	default:                            barrel_in_mux = `BARREL_IN_M;
endcase

// Barrel upper operand
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_EXTRACT:       barrel_u_in_mux = `BARREL_U_IN_N;
		default:                        barrel_u_in_mux = `BARREL_U_IN_L;
	endcase
	default:                            barrel_u_in_mux = `BARREL_U_IN_L;
endcase

// Sign extend option
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_BITFIELD:      bitext_sign_ext = ~|imm_opc;
		default:                        bitext_sign_ext = 1'b0;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_ADD_SUB_EXT:   bitext_sign_ext = ext_option[2];
		default:                        bitext_sign_ext = 1'b0;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_REG_OFFSET:        bitext_sign_ext = ext_option[2];
		default:                        bitext_sign_ext = 1'b0;
	endcase
	default:                            bitext_sign_ext = 1'b0;
endcase

// ALU operand A mux
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_MOVE:          alu_op_a_mux = &imm_opc?   `ALU_OP_A_EXEC_A : `ALU_OP_A_ZERO;
		`I_TYPE_DATA_IMM_BITFIELD:      alu_op_a_mux = imm_opc[0]? `ALU_OP_A_EXEC_A : `ALU_OP_A_ZERO;
		`I_TYPE_DATA_IMM_EXTRACT:       alu_op_a_mux = `ALU_OP_A_ZERO;
		default:                        alu_op_a_mux = `ALU_OP_A_EXEC_N;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_COND_SEL:      alu_op_a_mux = `ALU_OP_A_ZERO;
		default:                        alu_op_a_mux = `ALU_OP_A_EXEC_N;
	endcase
	`I_TYPE_BRANCH_SYS:                 alu_op_a_mux = `ALU_OP_A_EXEC_A;
	default:                            alu_op_a_mux = `ALU_OP_A_EXEC_N;
endcase

// ALU operand B mux
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         alu_op_b_mux = `ALU_OP_B_WMASK;
		default:                        alu_op_b_mux = `ALU_OP_B_BITEXT;
	endcase
	`I_TYPE_BRANCH_SYS:                 alu_op_b_mux = `ALU_OP_B_WMASK;
	default:                            alu_op_b_mux = `ALU_OP_B_BITEXT;
endcase

// Bitield mask / clear
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_MOVE:          wt_mask = 1;
		`I_TYPE_DATA_IMM_BITFIELD:      wt_mask = 1;
		default:                        wt_mask = 0;
	endcase
	default:                            wt_mask = 0;
endcase

// ALU operand B invert
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_MOVE:          alu_invert_b = ~|imm_opc;
		`I_TYPE_DATA_IMM_ADD_SUB:       alu_invert_b = add_sub_op;
		default:                        alu_invert_b = 0;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:   alu_invert_b = logic_inv;
		default:                        alu_invert_b = add_sub_op;
	endcase
	default:                            alu_invert_b = 0;
endcase

// ALU operation (command)
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_LOGIC:         alu_cmd = {`ALU_CMD_LOGIC,imm_opc};
		`I_TYPE_DATA_IMM_ADD_SUB:       alu_cmd = {`ALU_CMD_ADD_I,add_sub_op};
		`I_TYPE_DATA_IMM_PC_REL_ADDR:   alu_cmd = {`ALU_CMD_ADD,`ALU_CMD_ADD_0};
		default:                        alu_cmd = {`ALU_CMD_LOGIC,`ALU_CMD_ORR};
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:   alu_cmd = {`ALU_CMD_LOGIC,imm_opc};
		`I_TYPE_DATA_REG_ADD_SUB_WC:    alu_cmd = {`ALU_CMD_ADD,`ALU_CMD_ADD_C};
		`I_TYPE_DATA_REG_COND_SEL:      alu_cmd = {`ALU_CMD_ADD_I,reg_src_op[0]};
		default:                        alu_cmd = {`ALU_CMD_ADD_I,add_sub_op};
	endcase
	`I_TYPE_LDST:                       alu_cmd = {`ALU_CMD_ADD,`ALU_CMD_ADD_0};
	default:                            alu_cmd = {`ALU_CMD_LOGIC,`ALU_CMD_AND};
endcase

// Execute output mux
always @ ( * ) casex (I)
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_COND_SEL:      out_mux = `EX_OUT_CND;
		default:                        out_mux = `EX_OUT_ALU;
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_REG_MOVE:    out_mux = `EX_OUT_CTRL;
		default:                        out_mux = `EX_OUT_PC_4;
	endcase
	default:                            out_mux = `EX_OUT_ALU;
endcase

// Execute stage register width (64/32 bits)
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_PC_REL_ADDR:   FnH = 1;
		default:                        FnH = reg_sf;
	endcase
	`I_TYPE_LDST:                       FnH = 1;
	default:                            FnH = reg_sf;
endcase

// Next PC mux
always @ ( * ) casex(I)
	`I_TYPE_BRANCH_SYS: casex(I)
		`I_TYPE_BRANCH_SYS_UCND_BR_REG: nextPC_mux = `NEXT_PC_RN;
		default:                        nextPC_mux = `NEXT_PC_ADD;
	endcase
	default:                            nextPC_mux = `NEXT_PC_ADD;
endcase

// PC addition operand mux
always @ ( * ) casex(I)
	`I_TYPE_BRANCH_SYS: casex(I)
		`I_TYPE_BRANCH_SYS_REG_MOVE:    PC_add_op_mux = `PC_OP_NEXT;
		`I_TYPE_BRANCH_SYS_HINTS:       PC_add_op_mux = `PC_OP_NEXT;
		default:                        PC_add_op_mux = `PC_OP_COND;
	endcase
	default:                            PC_add_op_mux = `PC_OP_NEXT;
endcase

// Conditional branch condition mux
always @ ( * ) casex(I)
	`I_TYPE_BRANCH_SYS: casex(I)
		`I_TYPE_BRANCH_SYS_CND_BR:      br_condition_mux = `BR_COND_PSTATE;
		default:                        br_condition_mux = `BR_COND_UNCOND;
	endcase
	default:                            br_condition_mux = 2'bxx;
endcase

// PSTATE source mux
always @ ( * ) casex (I)
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_COND_COMP_REG: pstate_mux = `PSTATE_IN_CND;
		default:                        pstate_mux = `PSTATE_IN_ALU;
	endcase
	`I_TYPE_BRANCH_SYS:                 pstate_mux = `PSTATE_IN_REG;
	default:                            pstate_mux = `PSTATE_IN_ALU;
endcase

// PSTATE enable
always @ ( * ) casex (I)
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_ADD_SUB:       pstate_en = flag_s;
		`I_TYPE_DATA_IMM_LOGIC:         pstate_en = &imm_opc;
		default:                        pstate_en = 0;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:   pstate_en = &imm_opc;
		default:                        pstate_en = flag_s;
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_REG_MOVE:    pstate_en = ~sys_mov_l;
		default:                        pstate_en = 0;
	endcase
	default:                            pstate_en = 0;
endcase

always @ ( * ) casex (I)
	`I_TYPE_DATA_REG:                   condition = I[15:12];
	default:                            condition = I[3:0];
endcase

// Memory load register width (64/32 bits)
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      load_FnH = |ldst_opc_lit;
		default:                        load_FnH = (ldst_opc == `LD_S_64) | (ldst_size == `LDST_SIZE_64);
	endcase
	default:                            load_FnH = 1'bx;
endcase

// Memory read
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      mem_read = 1;
		default:                        mem_read = |ldst_opc;
	endcase
	default:                            mem_read = 0;
endcase

// Memory write
always @ ( * ) casex (I)
	`I_TYPE_LDST:                       mem_write = ~mem_read;
	default:                            mem_write = 0;
endcase

// Memory size
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      mem_size = {1'b1,ldst_opc_lit[0]};
		default:                        mem_size = ldst_size;
	endcase
	default:                            mem_size = 2'bxx;
endcase

// Memory load sign extension
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:      mem_sign_ext = ldst_opc_lit[1];
		default:                        mem_sign_ext = ldst_opc[1];
	endcase
	default:                            mem_sign_ext = 1'bx;
endcase

// Memory address mux
always @ ( * ) casex (I)
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_POST_IDX:          mem_addr_mux = `MEM_ADDR_RN;
		default:                        mem_addr_mux = `MEM_ADDR_ALU;
	endcase
	default:                            mem_addr_mux = 1'bx;
endcase

always @ ( * ) begin
// No error unless one of the following is true
decode_err = `ERROR_OK;
casex (I)
	`I_TYPE_RESERVED:
		decode_err = `ERROR_UNDEFINED;
	`I_TYPE_DATA_IMM: casex (I)
		`I_TYPE_DATA_IMM_PC_REL_ADDR:
			decode_err = `ERROR_OK;
		`I_TYPE_DATA_IMM_ADD_SUB:
			decode_err = `ERROR_OK;
		`I_TYPE_DATA_IMM_LOGIC:
			if(~reg_sf & imm_N)
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_IMM_MOVE:
			if(~reg_sf & mov_hw[1])
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_IMM_BITFIELD:
			if((reg_sf ^ imm_N) | (~reg_sf & (imm_imms[5]|imm_immr[5])))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_IMM_EXTRACT:
			if((reg_sf ^ imm_N) | (~reg_sf & imm_imms[5]))
				decode_err = `ERROR_UNDEFINED;
		default:
			decode_err = `ERROR_UNDEFINED;
	endcase
	`I_TYPE_DATA_REG: casex (I)
		`I_TYPE_DATA_REG_LOGIC_SHIFT:
			if(~reg_sf & imm_imms[5])
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_REG_ADD_SUB_SHIFT:
			if(&reg_src_shift | (~reg_sf & imm_imms[5]))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_REG_ADD_SUB_EXT:
			if(|reg_src_shift | (imm_imms[2] & |imm_imms[1:0]))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_REG_ADD_SUB_WC:
			decode_err = `ERROR_OK;
		`I_TYPE_DATA_REG_COND_COMP_REG:
			if(I[4] | reg_src_op[0] | ~flag_s)
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_DATA_REG_COND_SEL:
			if(flag_s | reg_src_op[1])
				decode_err = `ERROR_UNDEFINED;
		default:
			decode_err = `ERROR_UNDEFINED;
	endcase
	`I_TYPE_LDST: casex (I)
		`I_TYPE_LDST_LOAD_LITERAL:
			if(I[26] | &ldst_opc_lit)
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_LDST_UNSCALED_IMM:
			if(I[26] | (ldst_size[1] & &ldst_opc) | (&ldst_size & ldst_opc[1]))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_LDST_POST_IDX:
			if(I[26] | (ldst_size[1] & &ldst_opc) | (&ldst_size & ldst_opc[1]))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_LDST_PRE_IDX:
			if(I[26] | (ldst_size[1] & &ldst_opc) | (&ldst_size & ldst_opc[1]))
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_LDST_REG_OFFSET:
			if(I[26] | (ldst_size[1] & &ldst_opc) | (&ldst_size & ldst_opc[1]) | ~ext_option[1])
				decode_err = `ERROR_UNDEFINED;
		default:
			decode_err = `ERROR_UNDEFINED;
	endcase
	`I_TYPE_BRANCH_SYS: casex (I)
		`I_TYPE_BRANCH_SYS_CND_BR:
			if(I[4] | I[24])
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_BRANCH_SYS_HINTS:
			if(|I[11:8] | |I[7:6])
				decode_err = `ERROR_UNDEFINED;
			else if(I[5])
				decode_err = `ERROR_YIELD;
		`I_TYPE_BRANCH_SYS_REG_MOVE:
			if(I[19:5] != 15'h5A10)
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_BRANCH_SYS_UCND_BR_REG:
			if(|I[4:0] | |I[15:10] | ~&I[20:16] | |I[24:23] | &I[22:21])
				decode_err = `ERROR_UNDEFINED;
		`I_TYPE_BRANCH_SYS_UCND_BR_IMM:
			decode_err = `ERROR_OK;
		default:
			decode_err = `ERROR_UNDEFINED;
	endcase
	default:
		decode_err = `ERROR_UNDEFINED;
endcase
end

endmodule
