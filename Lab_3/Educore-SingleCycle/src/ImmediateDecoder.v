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
module ImmediateDecoder
(
	input [5:0] immr,
	input [5:0] imms,
	input       N,

	output [63:0] wmask,
	output [63:0] tmask
);

wire [6:0] n_imms;
wire [5:0] levels;
wire [6:0] diff;

assign n_imms = {N,~imms};
assign levels = {
	 n_imms[6],
	|n_imms[6:5],
	|n_imms[6:4],
	|n_imms[6:3],
	|n_imms[6:2],
	|n_imms[6:1]
};

assign diff = (imms&levels) + ~(immr&levels) + 1'b1;

/*****************************************************************************/
wire [5:0] tmask_and = diff[5:0] | ~levels;
wire [5:0] tmask_or  = diff[5:0] &  levels;

wire [63:0] tmask_a1 = {32{     tmask_and[0]  ,    1'b1   }};
wire [63:0] tmask_a2 = {16{ { 2{tmask_and[1]}},{ 2{1'b1}} }};
wire [63:0] tmask_a3 = { 8{ { 4{tmask_and[2]}},{ 4{1'b1}} }};
wire [63:0] tmask_a4 = { 4{ { 8{tmask_and[3]}},{ 8{1'b1}} }};
wire [63:0] tmask_a5 = { 2{ {16{tmask_and[4]}},{16{1'b1}} }};
wire [63:0] tmask_a6 = {    {32{tmask_and[5]}},{32{1'b1}}  };

wire [63:0] tmask_o2 = {16{ { 2{1'b0}},{ 2{tmask_or[1]}} }};
wire [63:0] tmask_o3 = { 8{ { 4{1'b0}},{ 4{tmask_or[2]}} }};
wire [63:0] tmask_o4 = { 4{ { 8{1'b0}},{ 8{tmask_or[3]}} }};
wire [63:0] tmask_o5 = { 2{ {16{1'b0}},{16{tmask_or[4]}} }};
wire [63:0] tmask_o6 = {    {32{1'b0}},{32{tmask_or[5]}}  };

assign tmask =
	(tmask_a1 & tmask_a2 & tmask_a3 & tmask_a4 & tmask_a5 & tmask_a6)
|	(tmask_o2 & tmask_a3 & tmask_a4 & tmask_a5 & tmask_a6)
|	(tmask_o3 & tmask_a4 & tmask_a5 & tmask_a6)
|	(tmask_o4 & tmask_a5 & tmask_a6)
|	(tmask_o5 & tmask_a6)
|	tmask_o6;

wire [5:0] wmask_and = immr | ~levels;
wire [5:0] wmask_or  = immr &  levels;

wire [63:0] wmask_a2 = {16{ { 2{1'b1}},{ 2{wmask_and[1]}} }};
wire [63:0] wmask_a3 = { 8{ { 4{1'b1}},{ 4{wmask_and[2]}} }};
wire [63:0] wmask_a4 = { 4{ { 8{1'b1}},{ 8{wmask_and[3]}} }};
wire [63:0] wmask_a5 = { 2{ {16{1'b1}},{16{wmask_and[4]}} }};
wire [63:0] wmask_a6 = {    {32{1'b1}},{32{wmask_and[5]}}  };

wire [63:0] wmask_o1 = {32{     wmask_or[0]  ,    1'b0   }};
wire [63:0] wmask_o2 = {16{ { 2{wmask_or[1]}},{ 2{1'b0}} }};
wire [63:0] wmask_o3 = { 8{ { 4{wmask_or[2]}},{ 4{1'b0}} }};
wire [63:0] wmask_o4 = { 4{ { 8{wmask_or[3]}},{ 8{1'b0}} }};
wire [63:0] wmask_o5 = { 2{ {16{wmask_or[4]}},{16{1'b0}} }};
wire [63:0] wmask_o6 = {    {32{wmask_or[5]}},{32{1'b0}}  };

wire [63:0] pre_wmask =
	(wmask_o1 & wmask_a2 & wmask_a3 & wmask_a4 & wmask_a5 & wmask_a6)
|	(wmask_o2 & wmask_a3 & wmask_a4 & wmask_a5 & wmask_a6)
|	(wmask_o3 & wmask_a4 & wmask_a5 & wmask_a6)
|	(wmask_o4 & wmask_a5 & wmask_a6)
|	(wmask_o5 & wmask_a6)
|	wmask_o6;

assign wmask = diff[6]? pre_wmask & tmask : pre_wmask | tmask;

endmodule
