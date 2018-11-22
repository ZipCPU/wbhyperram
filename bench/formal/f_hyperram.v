////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	bench/formal/f_hyperram.v
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	Provides a set of formal properties for testing a HyperRAM
//		module against.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2018, Gisselquist Technology, LLC
//
// This file is part of the WB-HyperRAM controller project.
//
// The WB-HyperRAM controller project is free software (firmware): you can
// redistribute it and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// The WB-HyperRAM controller project is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
// General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  (It's in the $(ROOT)/doc directory.  Run make
// with no target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	LGPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/lgpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
//
//
`default_nettype none
//
`define	FORMAL
`ifdef	FORMAL
//
module f_hyperram(i_clk, 
		i_reset_n, i_cke,    i_csn,
		i_rwctrl,  i_rw_out, i_rw_in,
		i_dq_we,   i_dq_out, i_dq_in,
		o_fv_cmd, o_fv_addr,  o_fv_data,
		o_vcs_count, o_rp_count, o_csm_count,
		o_cfgword);
	parameter	CLOCK_SPEED_HZ =   100_000_000;
	parameter	AW = 22;
	parameter	IODELAY = 22;
	//
	input	wire	i_clk;
	input	wire	i_reset_n, i_cke, i_csn, i_rwctrl;
	input	wire	[1:0]	i_rw_out, i_rw_in;
	input	wire		i_dq_we;
	input	wire	[15:0]	i_dq_out, i_dq_in;
	output	reg	[47:0]	o_fv_cmd;
	output	reg	[AW-1:0] o_fv_addr;
	output	reg	[15:0]	o_fv_data;
	output	reg	[31:0]	o_vcs_count, o_rp_count, o_csm_count;
	output	reg	[15:0]	o_cfgword;

	(* anyconst *) wire [AW-1:0]	fv_addr;
	always @(*)
		o_fv_addr = fv_addr;
	
	//////
	// Round faster
	localparam	CLOCK_SPEED_NS = 1_000_000_000 / (CLOCK_SPEED_HZ);
	// Tming Criteria
	localparam	CK_RP  = (200 + (CLOCK_SPEED_NS-1)) / CLOCK_SPEED_NS,
			CK_VCS = (150_000 / CLOCK_SPEED_NS),
			CK_CSM = (4_000_000 / CLOCK_SPEED_NS);

	initial assert(CLOCK_SPEED_HZ < 166_000_000);

	reg	[2:0]	latency;
	reg		fixed_latency;
	reg	[4:0]	start_count;

	always @(*)
	begin
		if (CLOCK_SPEED_HZ > 133_000_000)
			assert(latency == 6);
		else if (CLOCK_SPEED_HZ > 100_000_000)
			assert(latency >= 5);
		else if (CLOCK_SPEED_HZ >  83_000_000)
			assert(latency >= 4);
		else if (CLOCK_SPEED_HZ <=  83_000_000)
			assert(latency >= 3);
	end

	reg	f_past_valid = 0;
	always @(posedge i_clk)
		f_past_valid = 1'b1;

	/////////////////////////////////////////////
	//
	// Delay the high speed IO lines
	//
	reg		dly_cke;
	reg	[1:0]	dly_rw_out;
	reg	[15:0]	dly_dq_out;
	generate if (IODELAY == 0)
	begin

		always @(*)
		begin
			dly_cke     = i_cke;
			dly_rw_out  = i_rw_out;
			dly_dq_out  = i_dq_out;
		end

	end else if (IODELAY == 1)
	begin

		always @(posedge i_clk)
		begin
			dly_cke    <= i_cke;
			dly_rw_out <= i_rw_out;
			dly_dq_out <= i_dq_out;
		end

	end else begin

		reg	[IODELAY-2:0]		pipe_cke;
		reg	[2*(IODELAY-1):0]	pipe_rw_out;
		reg	[16*(IODELAY-1):0]	pipe_dq_out;

		always @(posedge i_clk)
		begin
			{ dly_cke, pipe_cke } <= { pipe_cke, i_cke };
			{ dly_rw_out, pipe_rw_out} <= { pipe_rw_out, i_rw_out };
			{ dly_dq_out, pipe_dq_out }<= { pipe_dq_out, i_dq_out };
		end

	end endgenerate

	/////////////////////////////////////////////
	//
	// Start with the reset
	//

	//
	// Insist on a minimum pulse width
	initial	o_rp_count = 0;
	always @(posedge i_clk)
	if (i_reset_n)
		o_rp_count <= 0;
	else if (!(&o_rp_count))
		o_rp_count <= o_rp_count + 1;

	always @(posedge i_clk)
	if ((f_past_valid)&&($rose(i_reset_n)))
		assert(o_rp_count >= CK_RP);

	always @(posedge i_clk)
	if (!i_reset_n)
		assert(i_csn);

	//
	// Insist on known delay from reset rising edge to the first CS#
	initial	o_vcs_count = 0;
	always @(posedge i_clk)
	if (!i_reset_n)
		o_vcs_count <= 0;
	else if ((i_reset_n)&&(!(&o_vcs_count)))
		o_vcs_count <= o_vcs_count + 1'b1;

	always @(posedge i_clk)
	if ((o_vcs_count < CK_VCS)||(!i_reset_n))
		assert(i_csn);

	/////////////////////////////////////////////
	//
	// Count the write recovery
	//
	// Not necessary in our implementation.
	/*
	reg	[2:0]	rwr_count;
	always @(posedge i_clk)
	if (!i_csn)
		rwr_count <= 0;
	else if (!(&rwr_count))
		rwr_count <= rwr_count + 1'b1;

	always @(*)
	if ((!i_csn)&&(rwr_count > 0))
		assert(rwr_count >= CK_RWR-2);
	*/

	/////////////////////////////////////////////
	//
	// Count the number of clock cycles with i_csn low
	//
	initial	o_csm_count = 0;
	always @(posedge i_clk)
	if (i_csn)
		o_csm_count <= 1'b0;
	else if (!(&o_csm_count))
		o_csm_count <= o_csm_count + 1'b1;


	always @(*)
		assert(o_csm_count < CK_CSM);

	/////////////////////////////////////////////
	//
	// Count cycles since starting
	//
	initial	start_count = 0;
	always @(posedge i_clk)
	if (i_csn)
		start_count <= 0;
	else if ((!i_csn)&&(dly_cke)&&(!(&start_count)))
		start_count <= start_count + 1'b1;

	/////////////////////////////////////////////
	//
	// Grab the command address
	//
	reg	[47:0]	fv_cmd;
	reg		double_latency;

	always @(posedge i_clk)
	if ((dly_cke)&&(!i_csn))
	begin
		if (start_count == 0)
		begin
			fv_cmd[47:32] <= dly_dq_out;

			// While the chip supports wrapped burst mode, this
			// property set only includes linear burst mode
			assert(dly_dq_out[13]);
		end
		if (start_count == 1)
			fv_cmd[31:16] <= dly_dq_out;
		if (start_count == 2)
			fv_cmd[15: 0] <= dly_dq_out;

		if (start_count < 3)
			assert((i_dq_we)&&(!i_rwctrl));

		if ((start_count > 0)&&(start_count < 3))
		begin
			assume(($stable(i_rw_in))&&(i_rw_in[0] == i_rw_in[1]));
			if (fixed_latency)
				assume(i_rw_in == 2'b11);
		end
		// else if ((start_count == 3)&&(!i_rwctrl))
		//	assume(($stable(i_rw_in))&&(i_rw_in[0] == i_rw_in[1]));

		double_latency <= (fixed_latency)||(i_rw_in);
	end

	always @(*)
	if (i_rwctrl)
		assume(i_rw_in == dly_rw_out);

	assign	o_fv_cmd = fv_cmd;
	/////////////////////////////////////////////
	//
	// Handle the latency writes
	//
	reg	devwrite;

	initial	o_cfgword = 16'b1000_1111_0001_1111;
	always @(posedge i_clk)
	if (!i_reset_n)
		o_cfgword <= 16'b1000_1111_0001_1111;
	else if ((dly_cke)&&(!i_csn)&&(start_count == 3))
	begin
		devwrite = (fv_cmd[47:46] == 2'b01);
		devwrite = (devwrite) && (fv_cmd[44:0] == 0);

		if (devwrite)
			assert(i_dq_we);

		if (devwrite)
		begin
			o_cfgword <= dly_dq_out;
			assert(dly_dq_out[11:8] == 4'hf);
		end

		if (AW > 22)
			o_cfgword[3] <= 1'b1;

		// fixed_latency <= (AW <= 22) ? dly_dq_out[3] : 1'b1;
	end

	always @(*)
	begin
		latency = 6;
		case(o_cfgword[7:4])
		4'b0000: latency = 5;
		4'b0001: latency = 6;
		4'b1110: latency = 3;
		4'b1111: latency = 4;
		default: assert(0);
		endcase
	end

	always @(*)
		fixed_latency = (o_cfgword[3]);


	/////////////////////////////////////////////
	//
	// Grab the initial address
	//
	wire	[31:0]		cmd_addr;
	reg	[AW-1:0]	mem_addr;
	wire			cmd_read, cmd_write, cmd_dev;
	reg	[3:0]	counts_till_active;
	reg		active;

	assign	cmd_addr = { fv_cmd[44:16], fv_cmd[2:0] };
	assign	cmd_read =   fv_cmd[47];
	assign	cmd_write = !cmd_read;
	assign	cmd_dev = !cmd_read;
	always @(posedge i_clk)
	if (start_count == 3)
		mem_addr <= cmd_addr[AW-1:0];
	else if (active)
		mem_addr <= mem_addr + 1;

	always @(posedge i_clk)
	if (start_count > 2)
	begin
		assert(cmd_addr[31:AW] == 0);
		assert(fv_cmd[15:3] == 0);
	end

	/////////////////////////////////////////////
	//
	// Active transaction
	//
	reg		read_stall;
	reg	[2:0]	stall_count;

	initial	counts_till_active = 12;
	always @(posedge i_clk)
	if (i_csn)
		counts_till_active = 2*latency;
	else if (start_count == 1)
	begin
		if (fv_cmd[47:46]==2'b00)
			counts_till_active <= 3;
		else if (double_latency)
			counts_till_active <= latency * 2;
		else
			counts_till_active <= latency;
	end else if (counts_till_active > 0)
		counts_till_active <= counts_till_active - 1;

	always @(posedge i_clk)
	if ((!i_csn)&&(counts_till_active == 1)&&(cmd_write))
		assert((i_rwctrl)&&(dly_rw_out == 2'b00));

	always @(*)
	if ((counts_till_active == 0)&&(!i_csn))
		assume((i_rwctrl)||(i_rw_in[0] == 0));


	always @(*)
		read_stall = (!i_csn)&&(cmd_read)&&(!i_rwctrl)&&(!i_rw_in[1]);

	always @(posedge i_clk)
	if ((f_past_valid)&&($past(read_stall))&&(!i_csn))
		assume(i_rw_in[1]);

	always @(*)
		active = (counts_till_active == 0)
			&&(!i_csn)&&(!read_stall)&&(dly_cke);

	initial	stall_count = 0;
	always @(posedge i_clk)
	if ((i_csn)||(cmd_write))
		stall_count <= 0;
	else if ((counts_till_active == 0)&&(dly_cke)&&(i_rw_in==2'b00))
		stall_count <= stall_count + 1'b1;

	always @(*)
	if ((&stall_count)&&(!i_csn)&&(counts_till_active == 0)&&(!cmd_write))
		assume(i_rw_in == 2'b10);

	always @(*)
	if (active)
		assert(i_rwctrl == cmd_write);

	always @(*)
	if ((active)&&(cmd_read)&&(!cmd_dev)&&(mem_addr == o_fv_addr))
		assume(i_dq_in == o_fv_data);

	initial	o_fv_data = 0;
	always @(posedge i_clk)
	if ((active)&&(cmd_write)&&(!cmd_dev)&&(mem_addr == o_fv_addr))
	begin
		if (!dly_rw_out[0])
			o_fv_data[15: 8] <= dly_dq_out[15:8];
		if (!dly_rw_out[1])
			o_fv_data[ 7: 0] <= dly_dq_out[ 7:0];
	end

endmodule
//
`endif	// FORMAL
