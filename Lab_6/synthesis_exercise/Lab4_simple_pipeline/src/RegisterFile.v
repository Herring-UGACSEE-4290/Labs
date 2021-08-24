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
module RegisterFile
(
	input        clk,
	input        clk_en,
	input        read_n_sp,
	input  [4:0] read_reg_an,
	input  [4:0] read_reg_am,
	input  [4:0] read_reg_aa,

	input        write_en,
	input  [4:0] write_reg_a,
	input [63:0] write_reg_v,

	input        wload_en,
	input  [4:0] wload_reg_a,
	input [63:0] wload_reg_v,

	output [63:0] read_reg_vn,
	output [63:0] read_reg_vm,
	output [63:0] read_reg_va
);

// Ensure the tools synthesize the correct components "flip-flops" not memory.
reg [63:0] rX00, rX01, rX02, rX03, rX04, rX05, rX06, rX07;
reg [63:0] rX08, rX09, rX10, rX11, rX12, rX13, rX14, rX15;
reg [63:0] rX16, rX17, rX18, rX19, rX20, rX21, rX22, rX23;
reg [63:0] rX24, rX25, rX26, rX27, rX28, rX29, rX30, rX31; // <- SP

wire [31:0] an_read_s;
wire [31:0] am_read_s;
wire [31:0] aa_read_s;
wire [31:0] write_s;
wire [31:0] wload_s;

genvar i;
generate for(i = 0; i < 32; i = i+1) begin: gen_registerfile
	if(i < 31) begin
		assign an_read_s[i] = (read_reg_an == i);
		assign am_read_s[i] = (read_reg_am == i);
		assign aa_read_s[i] = (read_reg_aa == i);
		assign write_s[i]   = (write_reg_a == i) & write_en;
		assign wload_s[i]   = (wload_reg_a == i) & wload_en;
	end else begin
		assign an_read_s[i] = (read_reg_an == i) & read_n_sp;
		assign am_read_s[i] = 1'b0;
		assign aa_read_s[i] = 1'b0;
		assign write_s[i]   = (write_reg_a == i) & write_en;
		assign wload_s[i]   = 1'b0;
	end
end
endgenerate

assign read_reg_vn =
	( (rX00 & {64{an_read_s[ 0]}}) | (rX01 & {64{an_read_s[ 1]}})
	| (rX02 & {64{an_read_s[ 2]}}) | (rX03 & {64{an_read_s[ 3]}})
	| (rX04 & {64{an_read_s[ 4]}}) | (rX05 & {64{an_read_s[ 5]}})
	| (rX06 & {64{an_read_s[ 6]}}) | (rX07 & {64{an_read_s[ 7]}})
	| (rX08 & {64{an_read_s[ 8]}}) | (rX09 & {64{an_read_s[ 9]}})
	| (rX10 & {64{an_read_s[10]}}) | (rX11 & {64{an_read_s[11]}})
	| (rX12 & {64{an_read_s[12]}}) | (rX13 & {64{an_read_s[13]}})
	| (rX14 & {64{an_read_s[14]}}) | (rX15 & {64{an_read_s[15]}})
	| (rX16 & {64{an_read_s[16]}}) | (rX17 & {64{an_read_s[17]}})
	| (rX18 & {64{an_read_s[18]}}) | (rX19 & {64{an_read_s[19]}})
	| (rX20 & {64{an_read_s[20]}}) | (rX21 & {64{an_read_s[21]}})
	| (rX22 & {64{an_read_s[22]}}) | (rX23 & {64{an_read_s[23]}})
	| (rX24 & {64{an_read_s[24]}}) | (rX25 & {64{an_read_s[25]}})
	| (rX26 & {64{an_read_s[26]}}) | (rX27 & {64{an_read_s[27]}})
	| (rX28 & {64{an_read_s[28]}}) | (rX29 & {64{an_read_s[29]}})
	| (rX30 & {64{an_read_s[30]}}) | (rX31 & {64{an_read_s[31]}}));

assign read_reg_vm =
	( (rX00 & {64{am_read_s[ 0]}}) | (rX01 & {64{am_read_s[ 1]}})
	| (rX02 & {64{am_read_s[ 2]}}) | (rX03 & {64{am_read_s[ 3]}})
	| (rX04 & {64{am_read_s[ 4]}}) | (rX05 & {64{am_read_s[ 5]}})
	| (rX06 & {64{am_read_s[ 6]}}) | (rX07 & {64{am_read_s[ 7]}})
	| (rX08 & {64{am_read_s[ 8]}}) | (rX09 & {64{am_read_s[ 9]}})
	| (rX10 & {64{am_read_s[10]}}) | (rX11 & {64{am_read_s[11]}})
	| (rX12 & {64{am_read_s[12]}}) | (rX13 & {64{am_read_s[13]}})
	| (rX14 & {64{am_read_s[14]}}) | (rX15 & {64{am_read_s[15]}})
	| (rX16 & {64{am_read_s[16]}}) | (rX17 & {64{am_read_s[17]}})
	| (rX18 & {64{am_read_s[18]}}) | (rX19 & {64{am_read_s[19]}})
	| (rX20 & {64{am_read_s[20]}}) | (rX21 & {64{am_read_s[21]}})
	| (rX22 & {64{am_read_s[22]}}) | (rX23 & {64{am_read_s[23]}})
	| (rX24 & {64{am_read_s[24]}}) | (rX25 & {64{am_read_s[25]}})
	| (rX26 & {64{am_read_s[26]}}) | (rX27 & {64{am_read_s[27]}})
	| (rX28 & {64{am_read_s[28]}}) | (rX29 & {64{am_read_s[29]}})
	| (rX30 & {64{am_read_s[30]}}) | ({64{1'b0}}));

assign read_reg_va =
	( (rX00 & {64{aa_read_s[ 0]}}) | (rX01 & {64{aa_read_s[ 1]}})
	| (rX02 & {64{aa_read_s[ 2]}}) | (rX03 & {64{aa_read_s[ 3]}})
	| (rX04 & {64{aa_read_s[ 4]}}) | (rX05 & {64{aa_read_s[ 5]}})
	| (rX06 & {64{aa_read_s[ 6]}}) | (rX07 & {64{aa_read_s[ 7]}})
	| (rX08 & {64{aa_read_s[ 8]}}) | (rX09 & {64{aa_read_s[ 9]}})
	| (rX10 & {64{aa_read_s[10]}}) | (rX11 & {64{aa_read_s[11]}})
	| (rX12 & {64{aa_read_s[12]}}) | (rX13 & {64{aa_read_s[13]}})
	| (rX14 & {64{aa_read_s[14]}}) | (rX15 & {64{aa_read_s[15]}})
	| (rX16 & {64{aa_read_s[16]}}) | (rX17 & {64{aa_read_s[17]}})
	| (rX18 & {64{aa_read_s[18]}}) | (rX19 & {64{aa_read_s[19]}})
	| (rX20 & {64{aa_read_s[20]}}) | (rX21 & {64{aa_read_s[21]}})
	| (rX22 & {64{aa_read_s[22]}}) | (rX23 & {64{aa_read_s[23]}})
	| (rX24 & {64{aa_read_s[24]}}) | (rX25 & {64{aa_read_s[25]}})
	| (rX26 & {64{aa_read_s[26]}}) | (rX27 & {64{aa_read_s[27]}})
	| (rX28 & {64{aa_read_s[28]}}) | (rX29 & {64{aa_read_s[29]}})
	| (rX30 & {64{aa_read_s[30]}}) | ({64{1'b0}}));

// ALU write value takes priority over memory loads
always @ (posedge clk) if(clk_en) begin
//always @ (*) begin
	if(write_s[ 0]|wload_s[ 0]) rX00 <= write_s[ 0]? write_reg_v : wload_reg_v;
	if(write_s[ 1]|wload_s[ 1]) rX01 <= write_s[ 1]? write_reg_v : wload_reg_v;
	if(write_s[ 2]|wload_s[ 2]) rX02 <= write_s[ 2]? write_reg_v : wload_reg_v;
	if(write_s[ 3]|wload_s[ 3]) rX03 <= write_s[ 3]? write_reg_v : wload_reg_v;
	if(write_s[ 4]|wload_s[ 4]) rX04 <= write_s[ 4]? write_reg_v : wload_reg_v;
	if(write_s[ 5]|wload_s[ 5]) rX05 <= write_s[ 5]? write_reg_v : wload_reg_v;
	if(write_s[ 6]|wload_s[ 6]) rX06 <= write_s[ 6]? write_reg_v : wload_reg_v;
	if(write_s[ 7]|wload_s[ 7]) rX07 <= write_s[ 7]? write_reg_v : wload_reg_v;
	if(write_s[ 8]|wload_s[ 8]) rX08 <= write_s[ 8]? write_reg_v : wload_reg_v;
	if(write_s[ 9]|wload_s[ 9]) rX09 <= write_s[ 9]? write_reg_v : wload_reg_v;
	if(write_s[10]|wload_s[10]) rX10 <= write_s[10]? write_reg_v : wload_reg_v;
	if(write_s[11]|wload_s[11]) rX11 <= write_s[11]? write_reg_v : wload_reg_v;
	if(write_s[12]|wload_s[12]) rX12 <= write_s[12]? write_reg_v : wload_reg_v;
	if(write_s[13]|wload_s[13]) rX13 <= write_s[13]? write_reg_v : wload_reg_v;
	if(write_s[14]|wload_s[14]) rX14 <= write_s[14]? write_reg_v : wload_reg_v;
	if(write_s[15]|wload_s[15]) rX15 <= write_s[15]? write_reg_v : wload_reg_v;
	if(write_s[16]|wload_s[16]) rX16 <= write_s[16]? write_reg_v : wload_reg_v;
	if(write_s[17]|wload_s[17]) rX17 <= write_s[17]? write_reg_v : wload_reg_v;
	if(write_s[18]|wload_s[18]) rX18 <= write_s[18]? write_reg_v : wload_reg_v;
	if(write_s[19]|wload_s[19]) rX19 <= write_s[19]? write_reg_v : wload_reg_v;
	if(write_s[20]|wload_s[20]) rX20 <= write_s[20]? write_reg_v : wload_reg_v;
	if(write_s[21]|wload_s[21]) rX21 <= write_s[21]? write_reg_v : wload_reg_v;
	if(write_s[22]|wload_s[22]) rX22 <= write_s[22]? write_reg_v : wload_reg_v;
	if(write_s[23]|wload_s[23]) rX23 <= write_s[23]? write_reg_v : wload_reg_v;
	if(write_s[24]|wload_s[24]) rX24 <= write_s[24]? write_reg_v : wload_reg_v;
	if(write_s[25]|wload_s[25]) rX25 <= write_s[25]? write_reg_v : wload_reg_v;
	if(write_s[26]|wload_s[26]) rX26 <= write_s[26]? write_reg_v : wload_reg_v;
	if(write_s[27]|wload_s[27]) rX27 <= write_s[27]? write_reg_v : wload_reg_v;
	if(write_s[28]|wload_s[28]) rX28 <= write_s[28]? write_reg_v : wload_reg_v;
	if(write_s[29]|wload_s[29]) rX29 <= write_s[29]? write_reg_v : wload_reg_v;
	if(write_s[30]|wload_s[30]) rX30 <= write_s[30]? write_reg_v : wload_reg_v;
	if(write_s[31]|wload_s[31]) rX31 <= write_s[31]? write_reg_v : wload_reg_v;
end

endmodule
