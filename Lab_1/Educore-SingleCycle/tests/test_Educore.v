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
`define TEST_LOG_PREFIX "[EDUCORE LOG]: "
`define TEST_ERR_PREFIX "[EDUCORE ERR]: "

`timescale 10ms/1ms


module test_Educore();

    // dump waveform event
	event START_LOG;
	initial begin
		@(START_LOG);
		$dumpvars(0, test_Educore);
	end


	// Clocks: main clock, memory clock and core clock. 
	// Core clock feeds to Educore.
	// Memory clock feeds to unified memory. 
	reg main_clk = 0;
	always #1 main_clk = ~main_clk;
	reg core_clk = 0;
	reg mem_clk = 0;
	
	// Clock ratio for single-cycle Educore
	always #2 mem_clk = ~mem_clk;
	always #4 core_clk = ~core_clk;
	
	// Clock ratio for pipelined EDUCORE. mem_clk and core_clk must not be inverts of each other.
	//always #2 mem_clk = ~mem_clk;
	//always #2 core_clk = ~core_clk;
	
	
	wire clk_en = 1;				  // Clock enable signal
	reg         nreset;               // Active low reset
	reg [31:0] instruction_memory_v;  // Instruction memory read value

	reg [63:0] data_memory_in_v;      // Data memory read value

	wire [63:0] data_memory_out_v;    // Data memory write value
	wire  [3:0] error_indicator;
	wire [63:0] instruction_memory_a; // Instruction memory address
	wire        instruction_memory_en;// Instruction memory read enable
	wire [63:0] data_memory_a;        // Data memory address
	wire  [1:0] data_memory_s;        // Data memory read/write size
	wire        data_memory_read;     // Data memory read control
	wire        data_memory_write;    // Data memory write control


	// Educore instantiation
	Educore educore (
		.clk(core_clk),
		.clk_en(clk_en),
		.nreset(nreset),

		.data_memory_a(data_memory_a),
		.data_memory_s(data_memory_s),
		.data_memory_read(data_memory_read),
		.data_memory_write(data_memory_write),
		.data_memory_in_v(data_memory_in_v),
		.data_memory_out_v(data_memory_out_v),

		.instruction_memory_en(instruction_memory_en),
		.instruction_memory_a(instruction_memory_a),
		.instruction_memory_v(instruction_memory_v),

		.error_indicator(error_indicator)
	);

	// Unified Memory for testbench (memory[]).
	// Instruction and Data are placed in this unified memory.
	// There is no memory protection, so please ensure that instructions do not overlap with data // memory in your Assembly code. 
	reg  [7:0]  memory [0:(2**16)-1];
	wire [7:0]  wb_en;


	assign wb_en =
		{{4{&data_memory_s}}, {2{data_memory_s[1]}}, |data_memory_s, 1'b1};

	always @ (posedge mem_clk or negedge nreset) begin
		if (~nreset) instruction_memory_v <= `INST_NOP; 
		else if(instruction_memory_en) begin
			instruction_memory_v[ 7: 0] <= memory[instruction_memory_a[15:0]];
			instruction_memory_v[15: 8] <= memory[instruction_memory_a[15:0]+1];
			instruction_memory_v[23:16] <= memory[instruction_memory_a[15:0]+2];
			instruction_memory_v[31:24] <= memory[instruction_memory_a[15:0]+3];
		end
	end

	always @ (posedge mem_clk) begin
		if(data_memory_read) begin
			data_memory_in_v[ 7: 0] <= memory[data_memory_a[15:0]];
			data_memory_in_v[15: 8] <= memory[data_memory_a[15:0]+1];
			data_memory_in_v[23:16] <= memory[data_memory_a[15:0]+2];
			data_memory_in_v[31:24] <= memory[data_memory_a[15:0]+3];
			data_memory_in_v[39:32] <= memory[data_memory_a[15:0]+4];
			data_memory_in_v[47:40] <= memory[data_memory_a[15:0]+5];
			data_memory_in_v[55:48] <= memory[data_memory_a[15:0]+6];
			data_memory_in_v[63:56] <= memory[data_memory_a[15:0]+7];
		end

		if(data_memory_write) begin
			if(wb_en[0]) memory[data_memory_a[15:0]+0] <= data_memory_out_v[ 7: 0];
			if(wb_en[1]) memory[data_memory_a[15:0]+1] <= data_memory_out_v[15: 8];
			if(wb_en[2]) memory[data_memory_a[15:0]+2] <= data_memory_out_v[23:16];
			if(wb_en[3]) memory[data_memory_a[15:0]+3] <= data_memory_out_v[31:24];
			if(wb_en[4]) memory[data_memory_a[15:0]+4] <= data_memory_out_v[39:32];
			if(wb_en[5]) memory[data_memory_a[15:0]+5] <= data_memory_out_v[47:40];
			if(wb_en[6]) memory[data_memory_a[15:0]+6] <= data_memory_out_v[55:48];
			if(wb_en[7]) memory[data_memory_a[15:0]+7] <= data_memory_out_v[63:56];
		end
	end

	// Signal instantiation to view memory for STRCPY exercise
	wire [8:0]  mem50;
	wire [8:0]  mem51;
	assign mem50 = memory[80];
	assign mem51 = memory[81];


	// Display, instruction memory initialization, error indication
	reg [8*128:0] TEST_CASE;
	initial begin
		nreset <= 0;
		if(!$value$plusargs("TEST_CASE=%s",TEST_CASE)) begin
			$display("%sPlease specify the test case file", `TEST_ERR_PREFIX);
			$finish;
		end
		$display("%sTest case: %0s", `TEST_LOG_PREFIX, TEST_CASE);
		$readmemh(TEST_CASE, memory);

		-> START_LOG;
		repeat (6) @(posedge main_clk);
		nreset <= 1;

		forever @(posedge core_clk)
			if(error_indicator == `ERROR_YIELD) begin
				$display("%sApollo has landed", `TEST_LOG_PREFIX);
				$finish;
			end
			else if(error_indicator == `ERROR_UNDEFINED) begin
				$display("%sHouston, we got a problem", `TEST_ERR_PREFIX);
				$finish;
			end
	end
endmodule
