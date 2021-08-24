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

module BarrelShifter (
	input   [5:0] shamt,
	input         FnH,
	input   [1:0] barrel_op,
	input  [63:0] barrel_in,
	input  [63:0] upper_in,

	output  [5:0] right_shamt,
	output [63:0] barrel_out
);

assign right_shamt = {FnH,{5{1'b1}}} &
	((barrel_op == `BARREL_OP_LSL)? ~shamt + 1'b1 : shamt);

wire [63:0] right_bits =
	(barrel_op == `BARREL_OP_LSL)? {64{1'b0}} : barrel_in;

// LSR shift causes this bus to be ZERO
wire [63:0] left_bits =
	({64{barrel_op == `BARREL_OP_LSL}} & barrel_in)
|	({64{barrel_op == `BARREL_OP_ASR}} & {64{FnH?barrel_in[63]:barrel_in[31]}})
|	({64{barrel_op == `BARREL_OP_ROR}} & upper_in);

wire [127:0] concat = FnH?
	{left_bits,right_bits} : {{64{1'b0}}, left_bits[31:0], right_bits[31:0]};

// This is usually done using generator for-loop
wire [127:0] barrel_lv0 = right_shamt[0]?
	{concat[0],concat[127:1]} : concat;

wire [127:0] barrel_lv1 = right_shamt[1]?
	{barrel_lv0[ 1:0],barrel_lv0[127: 2]} : barrel_lv0;

wire [127:0] barrel_lv2 = right_shamt[2]?
	{barrel_lv1[ 3:0],barrel_lv1[127: 4]} : barrel_lv1;

wire [127:0] barrel_lv3 = right_shamt[3]?
	{barrel_lv2[ 7:0],barrel_lv2[127: 8]} : barrel_lv2;

wire [127:0] barrel_lv4 = right_shamt[4]?
	{barrel_lv3[15:0],barrel_lv3[127:16]} : barrel_lv3;

wire [127:0] barrel_lv5 = right_shamt[5]?
	{barrel_lv4[31:0],barrel_lv4[127:32]} : barrel_lv4;

assign barrel_out = ~|right_shamt? barrel_in : barrel_lv5[63:0];

endmodule
