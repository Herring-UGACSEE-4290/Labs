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
/* The program counter (pointing to the next instruction to be fetched) */
reg [61:0] PC;
/* Processor state register (Holds NZCV flags`) */
reg [3:0] PSTATE;

/*****************************************************************************/
// Combintorial variables

/* Master enable of Educore (not processing element stop) */
wire npe_stop;
/* Instruction currently being decoded */
reg  [31:0] instruction;
/* Operand of the N datapath. This could be the PC or the register value from port N */
reg  [63:0] exec_n;
/* Operand of the M datapath. This could be an immediate value or a register value from port M */
reg  [63:0] exec_m;
/* Operand of the A datapath. This is always the register value from port A */
reg  [63:0] exec_a;
/* Barrel shifter immediate shift amount */
wire  [5:0] shamt;
/* Barrel shifter operation (LSL, LSR, ASR, ROR) */
wire  [1:0] barrel_op;
/* Select barrel shifter operand from N datapath or M datapath */
wire        barrel_in_mux;
/* Select upper operand shifted in from N datapath or same as barrel operand. (effective with ROR only) */
wire        barrel_u_in_mux;
/* ARM immediate decoder field size */
wire  [5:0] imm_sz;
/* ARM immediate decoder 64-bit field size flag */
wire        imm_n;
/* Operand & result size. (1->64-bit 'F'ull, 0->32-bit 'H'alf */
wire        FnH;
/* Bit extender operation (1-> sign extend , 0-> zero extend) */
wire        bitext_sign_ext;
/* Select operand A to the ALU. This could be ZERO, N datapath or A datapath */
wire  [1:0] alu_op_a_mux;
/* Select operand B to the ALU. This could be bit extender output or ARM immediate mask */
wire        alu_op_b_mux;
/* Active High: Apply bit clear on operand A and masking on operand B using the ARM immediate mask*/
wire        wtmask;
/* Invert operand B after entering the ALU */
wire        alu_invert_b;
/* ALU operation (AND, OR, EOR, ADD) */
wire  [2:0] alu_cmd;
/* select output from ALU, bit fiddler, conditional pass, PC+4 or System registers */
wire  [1:0] ex_out_mux;
/* Encoded condition to check agianst the PSTATE register */
wire  [3:0] condition;
/* PSTATE write enable signal */
wire        pstate_en;
/* Select source of next PSTATE. (ALU, Datapath A (register value), immediate) */
wire  [1:0] pstate_mux;
/* Select source of branch condition (PSTATE, unconditional) */
wire        br_condition_mux;
/* Select source of next PC value (N datapath (Register value), PC addition) */
wire        nextPC_mux;
/* Select operands for PC adder (A: PC or Executed instruction address, B: 4 or immediate value)*/
wire        PC_add_op_mux;
/* Memory transaction size (8b, 16b, 32b, 64b) */
wire  [1:0] mem_size;
/* Memory value sign extention (1-> sign extend, 0-> Zero extend) */
wire        mem_sign_ext;
/* Set size of loaded register after sign extention (1 -> 64-bit, 0-> 32-bit) */
wire        mem_load_FnH;
/* Memory read signal */
wire        mem_read;
/* Memory write signal */
wire        mem_write;
/* Select source of memory address. (Register (N-port) or Execute out (i.e ALU)) */
wire        mem_addr_mux;
/* Base register for memory access (N port register value) */
wire [63:0] rn_value;
/* Arithmetic/logic computation output */
reg  [63:0] ex_out;
/* Register value to be stored in memory (A port register value) */
wire [63:0] rt_value;
/* Register index of register to be written with memory loaded value */
wire  [4:0] rt_addr;
/* Register index of register to be written with 'ex_out' output */
wire  [4:0] rd_addr;
/* Write enable for memory loaded value */
wire        wload_en;
/* Write enable for 'ex_out' output value */
wire        write_en;
/* Memory loaded value */
wire [63:0] memory_out;
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
	always @ ( posedge clk or negedge nreset) begin
		if(~nreset) nreset_sync <= 0;
		else nreset_sync <= 1; 
	end
	
	assign npe_stop = clk_en & ~|error_indicator & nreset_sync;

	// enable instruction read 
	assign instruction_memory_en = nreset_sync;

/*****************************************************************************/
// Instruction Fetch stage

// -------------------------- Program Counter Logic -------------------------- //				

	always @ ( * ) case (br_condition_mux)
		`BR_COND_UNCOND: br_taken = 1;
		`BR_COND_PSTATE: br_taken = pstate_match;
		default:         br_taken = 1'bx;
	endcase

	wire [63:0] aligned_pc = {PC, 2'b00};
	always @ ( * ) case (PC_add_op_mux)
		`PC_OP_NEXT: PC_add_opA = aligned_pc;
		`PC_OP_COND: PC_add_opA = br_taken? exec_n : aligned_pc;
		default:     PC_add_opA = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	always @ ( * ) case (PC_add_op_mux)
		`PC_OP_NEXT: PC_add_opB = 4;
		`PC_OP_COND: PC_add_opB = br_taken? exec_m : 4;
		default:     PC_add_opB = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	wire [63:0] pc_add = PC_add_opA + PC_add_opB;

	always @ (posedge clk or negedge nreset_sync) begin
		if(~nreset_sync) PC <= 62'h0;
		else if(npe_stop) case (nextPC_mux)
			`NEXT_PC_ADD: PC <= pc_add[63:2];
			`NEXT_PC_RN:  PC <= exec_n[63:2];
		endcase
	end

	assign instruction_memory_a = aligned_pc;

	always @ (*) begin
		if (~nreset_sync) instruction = `INST_NOP;
		else instruction = instruction_memory_v;
	end
	

/******************************************************************************/
// Decode stage

	wire  [4:0] read_reg_an;
	wire  [4:0] read_reg_am;
	wire  [4:0] read_reg_aa;
	wire        read_n_sp;
	wire  [1:0] exec_n_mux;
	wire        exec_m_mux;
	wire [63:0] immediate;
	wire  [3:0] decode_err;

	InstructionDecoder instruction_decoder (
		.I(instruction),

		.read_reg_an(read_reg_an),
		.read_reg_am(read_reg_am),
		.read_reg_aa(read_reg_aa),
		.read_n_sp(read_n_sp),
		.exec_n_mux(exec_n_mux),
		.exec_m_mux(exec_m_mux),
		.immediate(immediate),

		.shamt(shamt),
		.imm_sz(imm_sz),
		.imm_n(imm_n),
		.FnH(FnH),
		.barrel_op(barrel_op),
		.barrel_in_mux(barrel_in_mux),
		.barrel_u_in_mux(barrel_u_in_mux),
		.bitext_sign_ext(bitext_sign_ext),
		.alu_op_a_mux(alu_op_a_mux),
		.alu_op_b_mux(alu_op_b_mux),
		.wt_mask(wtmask),
		.alu_invert_b(alu_invert_b),
		.alu_cmd(alu_cmd),
		.out_mux(ex_out_mux),
		.condition(condition),
		.pstate_en(pstate_en),
		.pstate_mux(pstate_mux),
		.br_condition_mux(br_condition_mux),
		.nextPC_mux(nextPC_mux),
		.PC_add_op_mux(PC_add_op_mux),

		.mem_size(mem_size),
		.mem_sign_ext(mem_sign_ext),
		.mem_read(mem_read),
		.mem_write(mem_write),
		.mem_addr_mux(mem_addr_mux),
		.load_FnH(mem_load_FnH),

		.wload_addr(rt_addr),
		.write_addr(rd_addr),
		.wload_en(wload_en),
		.write_en(write_en),

		// Error detection
		.decode_err(decode_err)
	);

	wire [63:0] rm_value;
	wire [63:0] ra_value;
	RegisterFile register_file (
		.clk(clk),
		.clk_en(npe_stop),
		.read_n_sp(read_n_sp),
		.read_reg_an(read_reg_an),
		.read_reg_am(read_reg_am),
		.read_reg_aa(read_reg_aa),

		.write_en(write_en),
		.write_reg_a(rd_addr),
		.write_reg_v(ex_out),

		.wload_en(wload_en),
		.wload_reg_a(rt_addr),
		.wload_reg_v(memory_out),

		.read_reg_vn(rn_value),
		.read_reg_vm(rm_value),
		.read_reg_va(ra_value)
	);

	assign error_indicator = decode_err;


// -------------------------- Decoded Instruction Logic -------------------------- //

	always @ ( * ) begin
		case (exec_n_mux)
			`FEXEC_N_PC:      exec_n = aligned_pc;
			`FEXEC_N_PC_PAGE: exec_n = {aligned_pc[63:12],12'h000};
			`FEXEC_N_RN:      exec_n = rn_value;
			default:          exec_n = 64'hxxxx_xxxx_xxxx_xxxx;
		endcase

		case (exec_m_mux)
			`FEXEC_M_IMM:     exec_m = immediate;
			`FEXEC_M_RM:      exec_m = rm_value;
			default:          exec_m = 64'hxxxx_xxxx_xxxx_xxxx;
		endcase

		exec_a = ra_value;
	end




	
/******************************************************************************/
// Execute stage

	// -------------------------- Barrel shifter -------------------------- //
	reg [63:0] barrel_in;
	always @ ( * ) case (barrel_in_mux)
		`BARREL_IN_N: barrel_in = exec_n;
		`BARREL_IN_M: barrel_in = exec_m;
		default:      barrel_in = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

	reg [63:0] barrel_u_in;
	always @ ( * ) case (barrel_u_in_mux)
		`BARREL_U_IN_N: barrel_u_in = exec_n;
		`BARREL_U_IN_L: barrel_u_in = barrel_in;
		default:        barrel_u_in = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase
	
	wire  [5:0] barrel_rshamt;
	wire [63:0] barrel_out;
	BarrelShifter barrel_shifter (
		.shamt(shamt),
		.FnH(FnH),
		.barrel_op(barrel_op),
		.barrel_in(barrel_in),
		.upper_in(barrel_u_in),

		.barrel_out(barrel_out),
		.right_shamt(barrel_rshamt)
	);

	// ------------------------- Immediate Decoder ------------------------- //
	wire [63:0] wmask, tmask;
	wire [63:0] wt_mask = wmask & tmask;
	wire  [5:0] immr = (wtmask | ~&{imm_n,imm_sz})? barrel_rshamt : 6'h0;
	ImmediateDecoder immediate_decoder (
		.immr(immr),
		.imms(imm_sz),
		.N(imm_n),

		.wmask(wmask),
		.tmask(tmask)
	);

	// ---------------   Masking and Bit Extender --------------------------- //
	wire [63:0] bitext_out;
	wire [63:0] bitext_in = wtmask? barrel_out & wt_mask : barrel_out;
	wire        sign_bit  = barrel_in[imm_sz];

	assign bitext_out =
		(~tmask & {64{bitext_sign_ext & sign_bit}}) | (tmask & bitext_in);

	// -------------------------------- Operand logic -------------------------------- //
	reg [63:0] alu_op_a;
	always @ ( * ) begin
		case (alu_op_a_mux)
			`ALU_OP_A_EXEC_N: alu_op_a = exec_n;
			`ALU_OP_A_EXEC_A: alu_op_a = exec_a;
			`ALU_OP_A_ZERO:   alu_op_a = 64'h0;
			default: alu_op_a = 64'hxxxx_xxxx_xxxx_xxxx;
		endcase
		if(wtmask)
			alu_op_a = alu_op_a & ~wt_mask;
	end

	reg [63:0] alu_op_b;
	always @ ( * ) begin
		case (alu_op_b_mux)
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
		.invert_b(alu_invert_b),
		.FnH(FnH),
		.cmd(alu_cmd),

		.result(alu_out),
		.flags_nzcv(alu_flags)
	);

	// ------------------------------ PSTATE logic ------------------------- //
	always @ ( * ) begin
		case (condition[3:1])
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

		if(condition[3:1] != 3'b111)
			pstate_match = condition[0] ^ pstate_match;
	end

	// Flags are embedded in the same field as rd_addr
	assign imm_nzcv = rd_addr[3:0];
	always @ (posedge clk) if(npe_stop) begin
		if(pstate_en) case (pstate_mux)
			`PSTATE_IN_ALU: PSTATE <= alu_flags;
			`PSTATE_IN_REG: PSTATE <= exec_a[31:28];
			`PSTATE_IN_CND: PSTATE <= pstate_match? alu_flags : imm_nzcv;
			default:        PSTATE <= 4'hx;
		endcase
	end

	wire [63:0] cnd_out = pstate_match? exec_n : alu_out;

	// ------------------------- Control registers ------------------------- //
	// We only have the PSTATE control register implemented
	wire [63:0] ctrl_out = {32'h0, PSTATE, 28'h0};

	assign rt_value = exec_a;

// ------------------------------ ALU result logic ------------------------- //

	always @ ( * ) case (ex_out_mux)
		`EX_OUT_ALU:    ex_out = FnH? alu_out    : {{32{1'b0}},alu_out[31:0]};
		`EX_OUT_CND:    ex_out = FnH? cnd_out    : {{32{1'b0}},cnd_out[31:0]};
		`EX_OUT_CTRL:   ex_out = ctrl_out;
		`EX_OUT_PC_4:   ex_out = aligned_pc+4;
		default:        ex_out = 64'hxxxx_xxxx_xxxx_xxxx;
	endcase

/******************************************************************************/
// Memory stage

assign data_memory_out_v = rt_value;
assign data_memory_a = (mem_addr_mux == `MEM_ADDR_RN)? rn_value : ex_out;
assign data_memory_s = mem_size;
assign data_memory_read  = mem_read;
assign data_memory_write = mem_write;


/******************************************************************************/
// WriteBack stage

// Choose data to write back
reg [63:0] ld_ext_memory_out;
always @ ( * ) begin
	case (mem_size)
		`LDST_SIZE_08: ld_ext_memory_out = {{56{mem_sign_ext & data_memory_in_v[ 7]}}, data_memory_in_v[ 7:0]};
		`LDST_SIZE_16: ld_ext_memory_out = {{48{mem_sign_ext & data_memory_in_v[15]}}, data_memory_in_v[15:0]};
		`LDST_SIZE_32: ld_ext_memory_out = {{32{mem_sign_ext & data_memory_in_v[31]}}, data_memory_in_v[31:0]};
		default:       ld_ext_memory_out = data_memory_in_v;
	endcase

	if(~mem_load_FnH)
		ld_ext_memory_out = {{32{1'b0}}, ld_ext_memory_out[31:0]};
end

assign memory_out = mem_read? ld_ext_memory_out : 64'hxxxx_xxxx_xxxx_xxxx;

// Simply wraps to registers
endmodule
