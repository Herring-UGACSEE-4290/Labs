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

module Educore
(
	input         clk,    // Core clock
	input         clk_en, // Clock enable
	input         nreset, // Active low reset
	input  [31:0] instruction_memory_v, // Instruction memory read value
	input  [63:0] data_memory_in_v,     // Data memory read value

	output  [3:0] error_indicator,
	output [63:0] instruction_memory_a, // Instruction memory address
	output        instruction_memory_en,// Instruction memory read enable
	output [63:0] data_memory_a,        // Data memory address
	output [63:0] data_memory_out_v,    // Data memory write value
	output  [1:0] data_memory_s,        // Data memory read/write size
	output        data_memory_read,     // Data memory read control
	output        data_memory_write     // Data memory write control
);

/******************************************************************************/
// Register declaration

/* Instruction currently being decoded */
reg [31:0] instruction;
/* The program counter (pointing to the next instruction to be fetched) */
reg [61:0] PC;
/* Processor state register (Holds NZCV flags`) */
reg [3:0] PSTATE;
/* The program counter / address of currently decoded instruction (previous value of PC) */
reg [61:0] ID_PC;
/* Operand of the N datapath. This could be the ID_PC or the register value from port N */
reg [63:0] EX_fexec_n;
/* Operand of the M datapath. This could be an immediate value or a register value from port M */
reg [63:0] EX_fexec_m;
/* Operand of the A datapath. This is always the register value from port A */
reg [63:0] EX_fexec_a;

/* Signal indicating that operands from the respective ports could be subject to forwarding. */
/* (i.e Valid modifiable register value) */





/* Register indices from the respective ports to enable and check forwarding */







/* Barrel shifter immediate shift amount */
reg  [5:0] EX_shamt;
/* Barrel shifter operation (LSL, LSR, ASR, ROR) */
reg  [1:0] EX_barrel_op;
/* Select barrel shifter operand from N datapath or M datapath */
reg        EX_barrel_in_mux;
/* Select upper operand shifted in from N datapath or same as barrel operand. (effective with ROR only) */
reg        EX_barrel_u_in_mux;
/* ARM immediate decoder field size */
reg  [5:0] EX_imm_sz;
/* ARM immediate decoder 64-bit field size flag */
reg        EX_imm_n;
/* Execute stage operand & result size. (1->64-bit 'F'ull, 0->32-bit 'H'alf */
reg        EX_FnH;
/* Bit extender operation (1-> sign extend , 0-> zero extend) */
reg        EX_bitext_sign_ext;
/* Select operand A to the ALU. This could be ZERO, N datapath or A datapath */
reg  [1:0] EX_alu_op_a_mux;
/* Select operand B to the ALU. This could be bit extender output or ARM immediate mask */
reg        EX_alu_op_b_mux;
/* Active High: Apply bit clear on operand A and masking on operand B using the ARM immediate mask*/
reg        EX_wtmask;
/* Invert operand B after entering the ALU */
reg        EX_invert_b;
/* ALU operation (AND, OR, EOR, ADD) */
reg  [2:0] EX_alu_cmd;
/* select execute stage output from ALU, bit fiddler, conditional pass, PC+4 or System registers */
reg  [1:0] EX_out_mux;
/* Encoded condition to check agianst the PSTATE register */
reg  [3:0] EX_condition;
/* PSTATE write enable signal */
reg        EX_pstate_en;
/* Select source of next PSTATE. (ALU, Datapath A (register valu), immediate) */
reg  [1:0] EX_pstate_mux;
/* Select source of branch condition (PSTATE, unconditional) */
reg        EX_br_condition_mux;
/* Select source of next PC value (N datapath (Register value), PC addition) */
reg        EX_nextPC_mux;
/* Select operands for PC adder (A: PC or Executed instruction address, B: 4 or immediate value)*/
reg        EX_PC_add_op_mux;

// To memory stage
/* Memory write signal */
reg        EX_mem_write;
reg        MEM_write;
/* Select source of memory address. (Register (N-port) or Execute out (i.e ALU)) */
reg        EX_mem_addr_mux;
reg        MEM_addr_mux;
/* Base register for memory access (N port register value) */
reg [63:0] MEM_rn_value;
/* Register value to be stored in memory (A port register value) */
reg [63:0] MEM_rt_value;

// to writeback stage
/* Execute stage output */
reg [63:0] MEM_ex_out;
reg [63:0] WB_ex_out;
/* Memory transaction size (8b, 16b, 32b, 64b) */
reg  [1:0] EX_mem_size;
reg  [1:0] MEM_size;
reg  [1:0] WB_size;
/* Memory value sign extention (1-> sign extend, 0-> Zero extend) */
reg        EX_mem_sign_ext;
reg        MEM_sign_ext;
reg        WB_sign_ext;
/* Set size of loaded register after sign extention (1 -> 64-bit, 0-> 32-bit) */
reg        EX_load_FnH;
reg        MEM_load_FnH;
reg        WB_load_FnH;
/* Memory read signal */
reg        EX_mem_read;
reg        MEM_read;
reg		   WB_read;
/* Register index of register to be written with memory loaded value */
reg  [4:0] EX_rt_addr;
reg  [4:0] MEM_rt_addr;
reg  [4:0] WB_rt_addr;
/* Register index of register to be written with execute stage output */
reg  [4:0] EX_rd_addr;
reg  [4:0] MEM_rd_addr;
reg  [4:0] WB_rd_addr;
/* Write enable for memory loaded value */
reg        EX_wload_en;
reg        MEM_wload_en;
reg        WB_wload_en;
/* Write enable for execute stage output value */
reg        EX_write_en;
reg        MEM_write_en;
reg        WB_write_en;

/* Pipline register to hold memory loaded value */
reg [63:0] WB_memory_out;

/*****************************************************************************/
// Combintorial variables

/* Master enable of Educore (not processing element stop) */
wire npe_stop;
/* Values of each datapath input (n/m/a) */
wire [63:0] EX_exec_n;
wire [63:0] EX_exec_m;
wire [63:0] EX_exec_a;
/* If instruction is branch, this determines if branch is taken */
reg br_taken;
/* Operands of addition to calculate the next PC value (next PC = A + B) */
reg [63:0] PC_add_opA;
reg [63:0] PC_add_opB;
/* Signals if PSTATE matches instruction encoded condition */
reg pstate_match;
/* Flag output of alu (NZCV) */
wire [3:0] alu_flags;
/* Instruction embedded PSTATE flags (NZCV)*/
wire [3:0] imm_nzcv;
/*****************************************************************************/
// System-wide net assignment

reg nreset_sync;

reg	instruction_valid;
wire next_instruction_valid;
	
	always @ ( posedge clk or negedge nreset) begin
		if(~nreset) begin
			nreset_sync <= 0;
			instruction_valid <= 0;
		end
		else begin
			nreset_sync <= 1;
			instruction_valid <= next_instruction_valid;
		end
	end
	
	assign npe_stop = clk_en & ~|error_indicator & nreset_sync;
	
	// enable instruction read
	assign instruction_memory_en = nreset_sync;
	
	// instruction read valid as long as no reset
	assign next_instruction_valid = nreset_sync;
		
/*****************************************************************************/
// Instruction Fetch stage
// -------------------------- Program Counter Logic -------------------------- //	

	always @ ( * ) case (EX_br_condition_mux)
		`BR_COND_UNCOND: br_taken = 1;
		`BR_COND_PSTATE: br_taken = pstate_match;
		default:         br_taken = 1'bx;
	endcase

	wire [63:0] aligned_pc = {PC, 2'b00};
	always @ ( * ) case (EX_PC_add_op_mux)
		`PC_OP_NEXT: PC_add_opA = aligned_pc;
		`PC_OP_COND: PC_add_opA = br_taken? EX_exec_n : aligned_pc;
		default:     PC_add_opA = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	always @ ( * ) case (EX_PC_add_op_mux)
		`PC_OP_NEXT: PC_add_opB = 4;
		`PC_OP_COND: PC_add_opB = br_taken? EX_exec_m : 4;
		default:     PC_add_opB = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	wire [63:0] pc_add = PC_add_opA + PC_add_opB;
	always @ (posedge clk or negedge nreset_sync)
	if(~nreset_sync) PC <= 62'h0;
	else if(npe_stop) case (EX_nextPC_mux)
		`NEXT_PC_ADD: PC <= pc_add[63:2];
		`NEXT_PC_RN:  PC <= EX_exec_n[63:2];
	endcase

	assign instruction_memory_a = aligned_pc;

	
	// --------------------- ID pipeline registers --------------------- //
	always @ (*) begin
		if (~instruction_valid) instruction = `INST_NOP;
		else if(npe_stop) instruction = instruction_memory_v;
	end

	wire [63:0] aligned_id_pc = {ID_PC, 2'b00};
	always @ (posedge clk) if(npe_stop) begin
		ID_PC <= PC;
	end

/******************************************************************************/
// Decode stage

	wire  [4:0] ID_read_reg_an;
	wire  [4:0] ID_read_reg_am;
	wire  [4:0] ID_read_reg_aa;
	wire        ID_read_n_sp;
	wire  [1:0] ID_fexec_n_mux;
	wire        ID_fexec_m_mux;
	wire [63:0] ID_immediate;
	wire  [3:0] ID_decode_err;

	// To Execute stage
	wire        ID_read_n_valid;
	wire        ID_read_m_valid;
	wire        ID_read_a_valid;
	wire  [5:0] ID_shamt;
	wire  [5:0] ID_imm_sz;
	wire        ID_imm_n;
	wire        ID_FnH;
	wire  [1:0] ID_barrel_op;
	wire        ID_barrel_in_mux;
	wire        ID_barrel_u_in_mux;
	wire        ID_bitext_sign_ext;
	wire  [1:0] ID_alu_op_a_mux;
	wire        ID_alu_op_b_mux;
	wire        ID_wtmask;
	wire        ID_invert_b;
	wire  [2:0] ID_alu_cmd;
	wire  [1:0] ID_out_mux;
	wire  [3:0] ID_condition;
	wire        ID_pstate_en;
	wire  [1:0] ID_pstate_mux;
	wire        ID_br_condition_mux;
	wire        ID_nextPC_mux;
	wire        ID_PC_add_op_mux;

	// to memory stage
	wire [1:0] ID_mem_size;
	wire       ID_mem_sign_ext;
	wire       ID_mem_read;
	wire       ID_mem_write;
	wire       ID_mem_addr_mux;
	wire       ID_load_FnH;

	// to writeback stage
	wire [4:0] ID_rt_addr;
	wire [4:0] ID_rd_addr;
	wire       ID_wload_en;
	wire       ID_write_en;
	InstructionDecoder intruction_decoder (
		.I(instruction),

		.read_reg_an(ID_read_reg_an),
		.read_reg_am(ID_read_reg_am),
		.read_reg_aa(ID_read_reg_aa),
		.read_n_sp(ID_read_n_sp),
		.fexec_n_mux(ID_fexec_n_mux),
		.fexec_m_mux(ID_fexec_m_mux),
		.immediate(ID_immediate),

		// To Execute stage
		.read_n_valid(ID_read_n_valid),
		.read_m_valid(ID_read_m_valid),
		.read_a_valid(ID_read_a_valid),
		.shamt(ID_shamt),
		.imm_sz(ID_imm_sz),
		.imm_n(ID_imm_n),
		.FnH(ID_FnH),
		.barrel_op(ID_barrel_op),
		.barrel_in_mux(ID_barrel_in_mux),
		.barrel_u_in_mux(ID_barrel_u_in_mux),
		.bitext_sign_ext(ID_bitext_sign_ext),
		.alu_op_a_mux(ID_alu_op_a_mux),
		.alu_op_b_mux(ID_alu_op_b_mux),
		.wt_mask(ID_wtmask),
		.alu_invert_b(ID_invert_b),
		.alu_cmd(ID_alu_cmd),
		.out_mux(ID_out_mux),
		.condition(ID_condition),
		.pstate_en(ID_pstate_en),
		.pstate_mux(ID_pstate_mux),
		.br_condition_mux(ID_br_condition_mux),
		.nextPC_mux(ID_nextPC_mux),
		.PC_add_op_mux(ID_PC_add_op_mux),

		// to memory stage
		.mem_size(ID_mem_size),
		.mem_sign_ext(ID_mem_sign_ext),
		.mem_read(ID_mem_read),
		.mem_write(ID_mem_write),
		.mem_addr_mux(ID_mem_addr_mux),
		.load_FnH(ID_load_FnH),

		// to writeback stage
		.wload_addr(ID_rt_addr),
		.write_addr(ID_rd_addr),
		.wload_en(ID_wload_en),
		.write_en(ID_write_en),

		// Error detection
		.decode_err(ID_decode_err)
	);

	wire [63:0] ID_rn_value;
	wire [63:0] ID_rm_value;
	wire [63:0] ID_ra_value;
	RegisterFile register_file (
		.clk(clk),
		.clk_en(npe_stop),
		.read_n_sp(ID_read_n_sp),
		.read_reg_an(ID_read_reg_an),
		.read_reg_am(ID_read_reg_am),
		.read_reg_aa(ID_read_reg_aa),

		.write_en(WB_write_en),
		.write_reg_a(WB_rd_addr),
		.write_reg_v(WB_ex_out),

		.wload_en(WB_wload_en),
		.wload_reg_a(WB_rt_addr),
		.wload_reg_v(WB_memory_out),

		.read_reg_vn(ID_rn_value),
		.read_reg_vm(ID_rm_value),
		.read_reg_va(ID_ra_value)
	);

	assign error_indicator = ID_decode_err;

	// --------------------- EXE pipeline registers --------------------- //
	always @ (posedge clk or negedge nreset)
	if(~nreset) begin
		EX_write_en      <= 0;
		EX_wload_en      <= 0;
		EX_mem_write     <= 0;
		EX_mem_read      <= 0;
		EX_pstate_en     <= 0;
		EX_nextPC_mux    <= `NEXT_PC_ADD;
		EX_PC_add_op_mux <= `PC_OP_NEXT;
	end else if(npe_stop) begin
		EX_write_en      <= ID_write_en;
		EX_wload_en      <= ID_wload_en;
		EX_mem_write     <= ID_mem_write;
		EX_mem_read      <= ID_mem_read;
		EX_pstate_en     <= ID_pstate_en;
		EX_nextPC_mux    <= ID_nextPC_mux;
		EX_PC_add_op_mux <= ID_PC_add_op_mux;
	end

	always @ (posedge clk) if(npe_stop) begin
		// -------------------------- Decoded Instruction Logic -------------------------- //
		case (ID_fexec_n_mux)
			`FEXEC_N_PC:      EX_fexec_n <= aligned_id_pc;
			`FEXEC_N_PC_PAGE: EX_fexec_n <= {aligned_id_pc[63:12],12'h000};
			`FEXEC_N_RN:      EX_fexec_n <= ID_rn_value;
		endcase

		case (ID_fexec_m_mux)
			`FEXEC_M_IMM:     EX_fexec_m <= ID_immediate;
			`FEXEC_M_RM:      EX_fexec_m <= ID_rm_value;
		endcase

		// --------------------- EXE pipeline registers --------------------- //
		EX_fexec_a          <= ID_ra_value;
		






		
		EX_shamt            <= ID_shamt;
		EX_imm_sz           <= ID_imm_sz;
		EX_imm_n            <= ID_imm_n;
		EX_FnH              <= ID_FnH;
		EX_barrel_op        <= ID_barrel_op;
		EX_barrel_in_mux    <= ID_barrel_in_mux;
		EX_barrel_u_in_mux  <= ID_barrel_u_in_mux;
		EX_bitext_sign_ext  <= ID_bitext_sign_ext;
		EX_alu_op_a_mux     <= ID_alu_op_a_mux;
		EX_alu_op_b_mux     <= ID_alu_op_b_mux;
		EX_wtmask           <= ID_wtmask;
		EX_invert_b         <= ID_invert_b;
		EX_alu_cmd          <= ID_alu_cmd;
		EX_out_mux          <= ID_out_mux;
		EX_condition        <= ID_condition;
		EX_pstate_mux       <= ID_pstate_mux;
		EX_br_condition_mux <= ID_br_condition_mux;

		// to memory stage
		EX_mem_size     <= ID_mem_size;
		EX_mem_sign_ext <= ID_mem_sign_ext;
		EX_mem_addr_mux <= ID_mem_addr_mux;

		// to writeback stage
		EX_rt_addr  <= ID_rt_addr;
		EX_rd_addr  <= ID_rd_addr;
		EX_load_FnH <= ID_load_FnH;
	end

/******************************************************************************/
// Execute stage

	// Forward detection














	// -------------------------- Barrel shifter -------------------------- //
	reg [63:0] barrel_in;
	always @ ( * ) case (EX_barrel_in_mux)
		`BARREL_IN_N: barrel_in = EX_exec_n;
		`BARREL_IN_M: barrel_in = EX_exec_m;
		default:      barrel_in = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	reg [63:0] barrel_u_in;
	always @ ( * ) case (EX_barrel_u_in_mux)
		`BARREL_U_IN_N: barrel_u_in = EX_exec_n;
		`BARREL_U_IN_L: barrel_u_in = barrel_in;
		default:        barrel_u_in = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	wire  [5:0] barrel_rshamt;
	wire [63:0] barrel_out;
	BarrelShifter barrel_shifter (
		.shamt(EX_shamt),
		.FnH(EX_FnH),
		.barrel_op(EX_barrel_op),
		.barrel_in(barrel_in),
		.upper_in(barrel_u_in),

		.barrel_out(barrel_out),
		.right_shamt(barrel_rshamt)
	);

	// ------------------------- Immediate Decoder ------------------------- //
	wire [63:0] wmask, tmask;
	wire [63:0] wt_mask = wmask & tmask;
	wire  [5:0] immr = (EX_wtmask | ~&{EX_imm_n,EX_imm_sz})? barrel_rshamt : 6'h0;
	ImmediateDecoder immediate_decoder (
		.immr(immr),
		.imms(EX_imm_sz),
		.N(EX_imm_n),

		.wmask(wmask),
		.tmask(tmask)
	);

	// ---------------- Masking and Bit Extender --------------------------- //
	wire [63:0] bitext_out;
	wire [63:0] bitext_in = EX_wtmask? barrel_out & wt_mask : barrel_out;
	wire        sign_bit  = barrel_in[EX_imm_sz];

	assign bitext_out =
		(~tmask & {64{EX_bitext_sign_ext & sign_bit}}) | (tmask & bitext_in);

	// -------------------------------- Operand logic -------------------------------- //
	reg [63:0] alu_op_a;
	always @ ( * ) begin
		case (EX_alu_op_a_mux)
			`ALU_OP_A_EXEC_N: alu_op_a = EX_exec_n;
			`ALU_OP_A_EXEC_A: alu_op_a = EX_exec_a;
			`ALU_OP_A_ZERO:   alu_op_a = 64'h0;
			default: alu_op_a = 64'hxxxx_xxxx_xxxx_xxxx;
		endcase
		if(EX_wtmask)
			alu_op_a = alu_op_a & ~wt_mask;
	end

	reg [63:0] alu_op_b;
	always @ ( * ) begin
		case (EX_alu_op_b_mux)
			`ALU_OP_B_WMASK:  alu_op_b = wmask;
			`ALU_OP_B_BITEXT: alu_op_b = bitext_out;
			default:          alu_op_b = 64'hxxxx_xxxx_xxxx_xxxx;
		endcase
	end

	// -------------------------------- ALU -------------------------------- //	
	wire [63:0] alu_out;
	ArithmeticLogicUnit alu (
		.operand_a(alu_op_a),
		.operand_b(alu_op_b),
		.carry_in(PSTATE[1]),
		.invert_b(EX_invert_b),
		.FnH(EX_FnH),
		.cmd(EX_alu_cmd),

		.result(alu_out),
		.flags_nzcv(alu_flags)
	);

	// ------------------------------ PSTATE logic ------------------------- //
	always @ ( * ) begin
		case (EX_condition[3:1])
			3'b000: pstate_match = PSTATE[2];                // Z = 1 (!=)
			3'b001: pstate_match = PSTATE[1];                // C = 1
			3'b010: pstate_match = PSTATE[3];                // N = 1 (< 0)
			3'b011: pstate_match = PSTATE[0];                // V = 1
			3'b100: pstate_match = ~PSTATE[2] & PSTATE[1];   // Unsigned <=
			3'b101: pstate_match = PSTATE[3] == PSTATE[0];   // <
			3'b110: pstate_match = (PSTATE[3] == PSTATE[0]) & ~PSTATE[2]; // <=
			3'b111: pstate_match = 1'b1;
			default: pstate_match = 1'bx;
		endcase

		if(EX_condition[3:1] != 3'b111)
			pstate_match = EX_condition[0] ^ pstate_match;
	end

	// Flags are embedded in the same field as rd_addr
	assign imm_nzcv = EX_rd_addr[3:0];
	always @ (posedge clk) if(npe_stop) begin
		if(EX_pstate_en) case (EX_pstate_mux)
			`PSTATE_IN_ALU: PSTATE <= alu_flags;
			`PSTATE_IN_REG: PSTATE <= EX_exec_a[31:28];
			`PSTATE_IN_CND: PSTATE <= pstate_match? alu_flags : imm_nzcv;
			default:        PSTATE <= 4'hx;
		endcase
	end

	wire [63:0] cnd_out = pstate_match? EX_exec_n : alu_out;

	// ------------------------- Control registers ------------------------- //
	// We only have the PSTATE control register implemented
	wire [63:0] ctrl_out = {32'h0, PSTATE, 28'h0};

	// --------------------- Memory pipeline registers --------------------- //

	always @ (posedge clk or negedge nreset)
	if(~nreset) begin
		MEM_read     <= 1'b0;
		MEM_write    <= 1'b0;
		MEM_wload_en <= 1'b0;
		MEM_write_en <= 1'b0;
	end else if(npe_stop) begin
		MEM_read     <= EX_mem_read;
		MEM_write    <= EX_mem_write;
		MEM_wload_en <= EX_wload_en;
		MEM_write_en <= EX_write_en;
	end

	always @ (posedge clk) if(npe_stop) begin
		MEM_rn_value <= EX_exec_n;
		MEM_rt_value <= EX_exec_a;
		MEM_size     <= EX_mem_size;
		MEM_sign_ext <= EX_mem_sign_ext;
		MEM_addr_mux <= EX_mem_addr_mux;

		// ------------------------------ ALU result logic ------------------ //
		case (EX_out_mux)
			`EX_OUT_ALU:    MEM_ex_out <= EX_FnH? alu_out    : {{32{1'b0}},alu_out[31:0]};
			`EX_OUT_CND:    MEM_ex_out <= EX_FnH? cnd_out    : {{32{1'b0}},cnd_out[31:0]};
			`EX_OUT_CTRL:   MEM_ex_out <= ctrl_out;
			`EX_OUT_PC_4:   MEM_ex_out <= aligned_id_pc;
			default:        MEM_ex_out <= 64'hxxxx_xxxx_xxxx_xxxx;
		endcase

		// To writeback
		MEM_rt_addr  <= EX_rt_addr;
		MEM_rd_addr  <= EX_rd_addr;
		MEM_load_FnH <= EX_load_FnH;
	end

/******************************************************************************/
// Memory stage

assign data_memory_out_v =  MEM_rt_value;
	
assign data_memory_a = (MEM_addr_mux == `MEM_ADDR_RN)? MEM_rn_value : MEM_ex_out;
assign data_memory_s = MEM_size;
assign data_memory_read  = MEM_read;
assign data_memory_write = MEM_write;

	// --------------------- WriteBack pipeline registers --------------------- //
	always @ (posedge clk or negedge nreset)
	if(~nreset) begin
		WB_wload_en <= 1'b0;
		WB_write_en <= 1'b0;
	end else if(npe_stop) begin
		WB_wload_en   <= MEM_wload_en;
		WB_write_en   <= MEM_write_en;
	end

	always @ (posedge clk) if(npe_stop) begin
		WB_ex_out     <= MEM_ex_out;
		WB_rt_addr    <= MEM_rt_addr;
		WB_rd_addr    <= MEM_rd_addr;
		
		WB_read       <= MEM_read;
		WB_size       <= MEM_size;
		WB_sign_ext   <= MEM_sign_ext;
		WB_load_FnH   <= MEM_load_FnH;
	end
	
/******************************************************************************/
// WriteBack stage

reg [63:0] ld_ext_memory_out;
always @ ( * ) begin
	case (WB_size)
		`LDST_SIZE_08: ld_ext_memory_out = {{56{WB_sign_ext & data_memory_in_v[ 7]}}, data_memory_in_v[ 7:0]};
		`LDST_SIZE_16: ld_ext_memory_out = {{48{WB_sign_ext & data_memory_in_v[15]}}, data_memory_in_v[15:0]};
		`LDST_SIZE_32: ld_ext_memory_out = {{32{WB_sign_ext & data_memory_in_v[31]}}, data_memory_in_v[31:0]};
		default:       ld_ext_memory_out = data_memory_in_v;
	endcase

	if(~WB_load_FnH)
		ld_ext_memory_out = {{32{1'b0}}, ld_ext_memory_out[31:0]};
end


	
	always @ (*) begin
		WB_memory_out =  WB_read ? ld_ext_memory_out :
		64'hxxxx_xxxx_xxxx_xxxx;
	end


// Simply wraps to registers

endmodule
