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

module ArithmeticLogicUnit
(
	input [63:0] operand_a,
	input [63:0] operand_b,
	input        carry_in,
	input        invert_b,
	input        FnH,
	input  [2:0] cmd,

	output reg [63:0] result,
	output      [3:0] flags_nzcv
);

wire [63:0] noperand_b = invert_b? ~operand_b : operand_b;
wire flag_n64, flag_n32;
wire flag_z64, flag_z32;
wire flag_c64, flag_c32;
wire flag_v64, flag_v32;

reg internal_cin;
reg cout_32, cout_64;

assign flags_nzcv = FnH?
	{flag_n64,flag_z64,flag_c64,flag_v64} :
	{flag_n32,flag_z32,flag_c32,flag_v32};

always @ ( * ) begin

	result       = 64'hxxxx_xxxx_xxxx_xxxx;
	internal_cin = 1'bx;
	cout_32      = 1'bx;
	cout_64      = 1'bx;

	case (cmd[2])
	`ALU_CMD_LOGIC:  case (cmd[1:0])
		`ALU_CMD_AND,
		`ALU_CMD_ANDS: result = operand_a & noperand_b;
		`ALU_CMD_ORR : result = operand_a | noperand_b;
		`ALU_CMD_EOR : result = operand_a ^ noperand_b;
		endcase
	`ALU_CMD_ADD: begin
		case(cmd[1:0])
		`ALU_CMD_ADD_0: internal_cin = 0;
		`ALU_CMD_ADD_1: internal_cin = 1;
		`ALU_CMD_ADD_C: internal_cin = carry_in;
		endcase

		{cout_32,result[31: 0]} = operand_a[31: 0] + noperand_b[31: 0] + internal_cin;
		{cout_64,result[63:32]} = operand_a[63:32] + noperand_b[63:32] + cout_32;

		end
	endcase
end

assign flag_n32 = result[31];
assign flag_n64 = result[63];

assign flag_z32 = ~|result[31: 0];
assign flag_z64 = ~|result[63:32] & flag_z32;

assign flag_c32 = cmd[2] & cout_32;
assign flag_c64 = cmd[2] & cout_64;

assign flag_v32 = cmd[2] & (operand_a[31] == noperand_b[31]) & (operand_a[31] != result[31]);
assign flag_v64 = cmd[2] & (operand_a[63] == noperand_b[63]) & (operand_a[63] != result[63]);

endmodule
