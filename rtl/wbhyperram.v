////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/wbhyperram.v
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	
//
//	Check	tRWR (Read-write recovery)
//	Host may stop RWDS to handle crossing memory boundaries
//	Must the clock idle before CS# high? (sec 3.3)
//	Can the clock be made to be free running?
//	Check maximum write burst length (configuration register 0?)
//	Check DPDOUT as part of the reset sequence
//	tCMD, or rather CK_CMS, is 4us -- not 1us
//	Check tCSM? (maximum CS# active time)
//	Check tVCS (reset to standby?)
//
//	RESET pulse must be low for tRP > 200ns
//	Following RESET#, CS must be high for tVCS = 150us
//
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
module wbhyperram(i_clk, i_reset,
		i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data, i_wb_sel,
			o_wb_stall, o_wb_ack, o_wb_data,
		o_hram_reset_n, o_hram_cke, o_hram_csn, o_hram_rwctrl,
			o_hram_rwds,
		i_hram_rwds, o_hram_drive_data, i_hram_data, o_hram_data);
	localparam	DEFAULT_LATENCY_COUNT = 6;
	parameter	AW= 23-2; // 8MB
	parameter	CLOCK_RATE_HZ = 100_000_000;
	//
	parameter [0:0]	OPT_PIPE  = 1'b1;
	parameter [0:0]	OPT_FIXED_LATENCY = 1'b0;
	//
	parameter	LATENCY_COUNT = DEFAULT_LATENCY_COUNT;
	//
	localparam	RWDS_HIGH = 2'b00,
			RWDS_LOW  = 2'b01,
			RWDS_LCL  = 2'b10,
			RWDS_INPT = 2'b11;
	localparam	DW = 32;
	localparam	MIN_LATENCY_COUNT = (CLOCK_RATE_HZ <= 83_000_000) ? 3
			: (CLOCK_RATE_HZ <= 100_000_000) ? 4
			: (CLOCK_RATE_HZ <= 133_000_000) ? 5 : 6;
	localparam	CLOCK_PERIOD_NS = 1000_000_000 / CLOCK_RATE_HZ;
	localparam	CK_VCS = 150_000 / CLOCK_PERIOD_NS,
			CK_RWR = ((40 +CLOCK_PERIOD_NS-1) / CLOCK_PERIOD_NS),
			// Minimum reset pulse width
			CK_RP = ((200+CLOCK_PERIOD_NS-1)/CLOCK_PERIOD_NS),
			// Maximum time CS# can be active
			CK_CSM = 4_000 / CLOCK_PERIOD_NS;
	localparam	CK_RWR_STALL = (CK_RWR>3) ? (CK_RWR-3) : 0;
	localparam	CSM_BITS = (CK_CSM > 255) ? 9
				: ((CK_CSM > 127) ? 8
				: ((CK_CSM > 63) ? 7
				: ((CK_CSM > 31) ? 6 : 5))),
			VCS_BITS = (CK_VCS > 2047) ? 12
				: ((CK_VCS > 1023) ? 11
				: ((CK_VCS >  511) ? 10
				: ((CK_VCS >  255) ?  9
				: ((CK_VCS >  127) ?  8
				: ((CK_VCS >   63) ?  7
				: ((CK_VCS >   31) ? 6 : 5)))))),
			RP_BITS = 5;
	input	wire	i_clk, i_reset;
	//
	// WB interface
	input	wire	i_wb_cyc, i_wb_stb, i_wb_we;
	input	wire	[AW:0]		i_wb_addr; // Top bit selects dev regs
	input	wire	[DW-1:0]	i_wb_data;
	input	wire	[DW/8-1:0]	i_wb_sel;
	//
	output	reg			o_wb_stall, o_wb_ack;
	output	reg	[DW-1:0]	o_wb_data;
	//
	// HyperRAM interface
	output	reg		o_hram_reset_n, o_hram_cke, o_hram_csn;
	output	reg	[1:0]	o_hram_rwctrl, o_hram_rwds;
	input	wire		i_hram_rwds;
	output	reg		o_hram_drive_data;
	input	wire	[15:0]	i_hram_data;
	output	reg	[15:0]	o_hram_data;
	//

	reg	[47:0]	cmd_reg;
	reg	[31:0]	data_reg;
	reg	[3:0]	data_mask;
	//
	reg	[3:0]	state_ctr;
	reg	[3:0]	read_ctr, write_ctr;
	reg	[1:0]	cmd_ctr;

	reg	cmd_output, pipe_req;
	wire	dev_addr, mem_addr, write_stb, read_stb, start_stb,
			pipe_stb, bus_stb;
	reg	cti_dev, cti_write;	// Cycle type indication


	//
	//
	assign	dev_addr  = !i_wb_addr[AW];
	assign	mem_addr  =  i_wb_addr[AW];
	assign	bus_stb   = (i_wb_stb)&&(!o_wb_stall);
	assign	start_stb = (bus_stb)&&( o_hram_csn);
	assign	pipe_stb  = (bus_stb)&&(pipe_req);
	assign	read_stb  = (bus_stb)&&(!i_wb_we);
	assign	write_stb = (bus_stb)&&( i_wb_we);


	reg	[CSM_BITS-1:0]	chip_select_count;
	reg			chip_select_warning;

	/////////////////////////////////////////////////////////////
	//
	// Handle reset
	//
	//
	reg	[RP_BITS-1:0]	reset_clock;
	reg	[VCS_BITS-1:0]	reset_recovery;
	reg			rwr_stall, maintenance_stall;

	initial	reset_clock = CK_RP[RP_BITS-1:0];
	always @(posedge i_clk)
	if (i_reset)
		reset_clock <= CK_RP[RP_BITS-1:0];
	else if (reset_clock > 0)
		reset_clock <= reset_clock - 1'b1;

	initial	o_hram_reset_n = 1'b0;
	always @(posedge i_clk)
	if (i_reset)
		o_hram_reset_n <= 1'b0;
	else
		o_hram_reset_n <= (reset_clock == 0);

	initial	reset_recovery = CK_VCS[VCS_BITS-1:0];
	always @(posedge i_clk)
	if (!o_hram_reset_n)
		reset_recovery <= CK_VCS[VCS_BITS-1:0];
	else if (reset_recovery > 0)
		reset_recovery <= reset_recovery - 1'b1;

	generate if (CK_RWR_STALL == 0)
	begin : NO_RWR_STALL

		always @(*)
			rwr_stall =  1'b0;

	end else if (CK_RWR_STALL == 1)
	begin : SINGLE_RWR_STALL

		always @(posedge i_clk)
			rwr_stall <= !o_hram_csn;

	end else begin : LONG_RWR_STALL
		reg	[1:0]	rwr_recovery;

		always @(posedge i_clk)
		if (!o_hram_csn)
			rwr_recovery <= CK_RWR -1;
		else if (rwr_recovery > 0)
			rwr_recovery <= rwr_recovery - 1;

		always @(*)
		if (!o_hram_csn)
			rwr_stall <= 1'b1;
		else
			rwr_stall <= (rwr_recovery > 1);

	end endgenerate

	always @(posedge i_clk)
	if ((i_reset)||(!o_hram_reset_n))
		maintenance_stall <= 1'b1;
	else if (reset_recovery > 0)
		maintenance_stall <= 1'b1;
	else
		maintenance_stall <= 1'b1;

		

	/////////////////////////////////////////////////////////////
	//
	// Recognize ongoing transactions for the next word
	//
	//
	generate if (OPT_PIPE)
	begin : PIPED_REQUESTS
		reg	[AW-1:0]	next_addr;

		initial	chip_select_count = 0;
		always @(posedge i_clk)
		if (o_hram_csn)
			chip_select_count <= 0;
		else
			chip_select_count <= chip_select_count + 1'b1;

		always @(posedge i_clk)
		if (o_hram_csn)
			chip_select_warning <= 1'b0;
		else
			chip_select_warning
				<= (chip_select_count > CK_CSM[CSM_BITS-1:0]-6);

		always @(posedge i_clk)
		if (bus_stb)
			next_addr <= i_wb_addr[AW-1:0] + 1'b1;

		initial	pipe_req = 1'b0;
		always @(posedge i_clk)
			pipe_req <= (i_wb_stb)&&(o_wb_stall)
				&&(!o_hram_csn)
				&&(!chip_select_warning)
				&&(i_wb_we == cti_write)
				&&(mem_addr)&&(!cti_dev)
				&&(!cmd_output)&&(state_ctr == 2)
				&&(i_wb_addr[AW-1:0] == next_addr);
//
// STB
// PIP
// (STB)&&(!STALL)
// 
	end else begin : NO_PIPE

		always @(*)
		begin
			chip_select_count   = 0;
			chip_select_warning = 0;
			pipe_req = 0;
		end

		// Verilator lint_off UNUSED
		wire	[6:0]	unused_pipe;
		assign	unused_pipe = chip_select_count;
		// Verilator lint_on UNUSED
		
	end endgenerate

	always @(posedge i_clk)
	if (start_stb)
		{ cti_dev, cti_write } <= { dev_addr, i_wb_we };

	initial	o_hram_csn = 1'b1;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		o_hram_csn <= 1'b1;
	else if ((bus_stb)||(cmd_output))
		o_hram_csn <= 1'b0;
	else if (state_ctr == 1)
		o_hram_csn <= 1'b1;

	//
	//
	//
	//
	initial	cmd_ctr = 0;
	initial	cmd_output = 1'b0;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
	begin
		cmd_ctr    <= 0;
		cmd_output <= 0;
	end else if (start_stb)
	begin
		cmd_ctr <= 3;
		cmd_output <= 1'b1;
	end else if (cmd_ctr != 0)
	begin
		cmd_ctr <= cmd_ctr - 1'b1;
		cmd_output <= (cmd_ctr > 1);
	end

	reg	r_stall;
	initial	r_stall = 1'b1;
	always @(posedge i_clk)
	if ((i_reset)||(maintenance_stall))
		r_stall <= 1'b1;
	else if (!i_wb_cyc)
		r_stall <= (!o_hram_csn)&&(CK_RWR_STALL > 0);
	else if ((o_hram_csn)&&(rwr_stall))
		r_stall <= 1'b1;
	else if (chip_select_warning)
		r_stall <= 1'b1;
	else if (bus_stb)
		r_stall <= 1'b1;
	else if ((pipe_req)&&(state_ctr == 2))
		r_stall <= 1'b0;
	else if ((cmd_ctr>0)||(state_ctr > 1))
		r_stall <= 1'b1;
	else
		r_stall <= 1'b0;

	always @(*)
		o_wb_stall = (r_stall)&&(!pipe_req);

	always @(posedge i_clk)
	if (bus_stb)
		cmd_reg <= { (!i_wb_we), dev_addr, 1'b1,
			{ {(29-(AW-2)){1'b0}}, i_wb_addr[AW-1:2] },
			13'h0, i_wb_addr[1:0], 1'b0 };
	else
		cmd_reg <= { cmd_reg[31:0], 16'h00 };

	//
	//
	//
	//
	initial	state_ctr = 0;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		state_ctr <= 0;
	else if (start_stb)
		state_ctr <= 1;
	else if (cmd_output)
	begin
		casez({(i_hram_rwds||OPT_FIXED_LATENCY), cti_dev, cti_write})
		3'b0?0: state_ctr <= 2 + LATENCY_COUNT;
		3'b001: state_ctr <= 1 + LATENCY_COUNT;
		3'b?11: state_ctr <= 1;
		3'b1?0: state_ctr <= 2 + 2*LATENCY_COUNT;
		3'b101: state_ctr <= 1 + 2*LATENCY_COUNT;
		default: state_ctr <= 1;
		endcase
	end else if (pipe_stb)
		state_ctr <= 2;
	else if (state_ctr > 0)
		state_ctr <= state_ctr - 1'b1;

	initial	write_ctr = 0;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		write_ctr <= 0;
	else if ((start_stb)&&(write_stb))
		write_ctr <= 1;
	else if ((OPT_PIPE)&&(write_stb))
		write_ctr <= 2;
	else if (!cti_write)
		write_ctr <= 0;
	else if ((cmd_output)&&(!cti_dev))
		write_ctr <= 1 + (((OPT_FIXED_LATENCY)||(i_hram_rwds))
				? 2 * LATENCY_COUNT : LATENCY_COUNT);
	else if ((!cmd_output)&&(write_ctr > 0))
		write_ctr <= write_ctr - 1'b1;

	reg	write_data_shift;
	always @(posedge i_clk)
		write_data_shift <= (!o_hram_csn)&&(state_ctr < 4)
			&&(!cmd_output)&&(cti_write);

	always @(posedge i_clk)
	if (bus_stb)
	begin
		data_reg  <= i_wb_data;
		data_mask <= i_wb_sel;
		if (dev_addr)
			data_reg[31:16] <= i_wb_data[15:0];
	end else if (write_data_shift)
	begin
		data_reg  <= { data_reg[15:0], 16'h0 };
		data_mask <= { data_mask[1:0], 2'b00 };
	end

	initial	o_hram_drive_data = 1'b1;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		o_hram_drive_data <= 1'b1;
	else if ((start_stb)||(cmd_output))
		o_hram_drive_data <= 1'b1;
	else if ((cti_write)&&((bus_stb)||(state_ctr>1)))
		o_hram_drive_data <= 1'b1;
	else if ((bus_stb)||(state_ctr>1))
		o_hram_drive_data <= 1'b0;
	else
		// Bus cycle is over
		o_hram_drive_data <= 1'b1;

	//
	//
	// read_ctr is a count down timer counting down until
	// a read is complete.  The actual read takes place when
	// read_ctr == 2 or read_ctr == 1
	//
	initial	read_ctr = 0;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		read_ctr <= 0;
	else if ((start_stb)&&(read_stb))
		read_ctr <= 1;
	else if ((cmd_output)&&(!cti_write))
	begin
		// Need to wait LATENCY_COUNT-1, possibly LATENCY_COUNT*2-1
		// clocks from the end of the cmd_output before the first
		// read, at read_ctr == 2
		if ((OPT_FIXED_LATENCY)||(i_hram_rwds))
			read_ctr <= 2 + 2*LATENCY_COUNT;
		else
			read_ctr <= 2 + LATENCY_COUNT;
	end else if (read_stb)
		read_ctr <= 2;
	else if (read_ctr > 0)
		read_ctr <= read_ctr - 1'b1;

	initial	o_wb_ack = 1'b0;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		o_wb_ack <= 1'b0;
	else
		o_wb_ack <= ((!cmd_output)&&(state_ctr == 1));

	always @(*)
	if (cmd_output)
		o_hram_data = cmd_reg[47:32];
	else
		o_hram_data = data_reg[31:16];

	//
	// Control the direction of the rwds signal
	//
	initial	o_hram_rwctrl = RWDS_LCL;
	always @(posedge i_clk)
	if ((i_reset)||(!i_wb_cyc))
		o_hram_rwctrl <= RWDS_LCL;
	else if ((start_stb)||(cmd_output)||(!cti_write)||(cti_dev))
		o_hram_rwctrl <= RWDS_INPT;
	else
		o_hram_rwctrl <= RWDS_LCL;

	always @(*)
		o_hram_rwds = (write_data_shift) ? (~data_mask[3:2]) : (2'b00);

	always @(*)
		o_hram_cke = (!o_hram_csn);
	always @(posedge i_clk)
		o_wb_data <= { o_wb_data[15:0], i_hram_data };

	// Verilator lint_off UNUSED
	wire	[1:0]	unused;
	assign	unused = { mem_addr, cti_dev };
	// Verilator lint_on  UNUSED

`ifdef	FORMAL
	parameter [0:0]	F_OPT_COVER = 1'b0;

	initial	assert(LATENCY_COUNT >= MIN_LATENCY_COUNT);
	initial	assert(LATENCY_COUNT <= DEFAULT_LATENCY_COUNT);

	reg	f_past_valid;
	initial	f_past_valid = 1'b0;
	always @(posedge i_clk)
		f_past_valid <= 1'b1;

	always @(posedge i_clk)
	if ((f_past_valid)&&($past(i_wb_cyc))&&(!$past(i_reset))
			&&(!$past(cmd_output))&&($past(state_ctr)==1))
		assert(o_wb_ack);

	localparam	F_LGDEPTH = 4;

	wire	[(F_LGDEPTH-1):0]	f_nreqs, f_nacks, f_outstanding;

	fwb_slave #(.AW(AW+1), .DW(DW), .F_MAX_STALL(26), .F_MAX_ACK_DELAY(25),
			.F_LGDEPTH(3), .F_MAX_REQUESTS(0))
		busproperties(i_clk, i_reset,
			i_wb_cyc, i_wb_stb, i_wb_we, i_wb_addr, i_wb_data,
				i_wb_sel,
			o_wb_ack, o_wb_stall, o_wb_data, 1'b0,
			f_nreqs, f_nacks, f_outstanding);

	always @(*)
	if ((i_wb_cyc)&&(!o_hram_csn))
		assert(f_outstanding > 0);
	else
		assert((f_outstanding == 0)||(o_wb_ack));

always @(*)
assert(cmd_output == (cmd_ctr != 0));
always @(*)
if ((cti_write)&&(write_ctr == 0))
assert(o_hram_csn);

always @(*)
if ((pipe_stb)&&(read_stb))
assert(read_ctr == 1);

always @(*)
if ((pipe_stb)&&(write_stb))
assert(write_ctr == 1);



	always @(*)
	if (o_wb_ack)
		assert((read_ctr == 0)||(read_ctr == 2)
			||(write_ctr == 0)||(write_ctr == 2));

	always @(posedge i_clk)
	if (state_ctr == 0)
	begin
		assert(write_ctr == 0);
		assert(read_ctr  == 0);
	end else if (state_ctr < 4)
	begin
		assert((read_ctr == state_ctr)
			||(write_ctr == state_ctr));
	end

	always @(posedge i_clk)
	if (i_wb_cyc)
	begin
		if (!o_hram_csn)
			assert(f_outstanding > 0);
		if (OPT_PIPE)
			assert(f_outstanding <= 2);
		else
			assert(f_outstanding <= 1);

		if (f_outstanding == 2)
			assert((OPT_PIPE)&&(o_wb_ack));

		if ((read_ctr == 0)&&(write_ctr == 0))
			assert((o_wb_ack)||(f_outstanding == 0));

		if ((read_ctr == 1)||(write_ctr == 1))
			assert(f_outstanding == 1);
	end

	always @(*)
	if (!cmd_output)
		assert((write_ctr == 0)||(read_ctr == 0));


	generate if (F_OPT_COVER)
	begin : GEN_COVER

		always @(posedge i_clk)
		if ((f_past_valid)&&($past(i_wb_cyc)&&(i_wb_cyc)&&(!i_wb_stb)))
			assume($stable(i_wb_we));

	end endgenerate

	always @(posedge i_clk)
	if ((f_past_valid)&&(!$past(o_hram_csn))&&(cmd_ctr > 0))
		assume($stable(i_hram_rwds));

	always @(*)
	if (o_hram_rwctrl == RWDS_LCL)
		assume(i_hram_rwds == o_hram_rwds);

	always @(*)
	if ((!o_hram_csn)&&(cti_dev))
		assert(o_hram_rwctrl == RWDS_INPT);

	always @(posedge i_clk)
	if ((f_past_valid)&&(($past(i_reset))||($past(!i_wb_cyc))))
		assert(o_hram_drive_data);

	always @(*)
	if (o_hram_csn)
		assert(o_hram_drive_data);

	always @(posedge i_clk)
	if ((f_past_valid)&&(($past(i_reset))||($past(!i_wb_cyc))))
		assert(o_hram_csn);

	always @(posedge i_clk)
	if ((f_past_valid)&&(($past(i_reset))||($past(!i_wb_cyc))))
		assert(!o_hram_cke);

	reg	[47:0]	fv_cmd;
	always @(posedge i_clk)
	if (start_stb)
		fv_cmd <= { (!i_wb_we), dev_addr, 1'b1,
			{ {(29-(AW-2)){1'b0}}, i_wb_addr[AW-1:2] },
			13'h0, i_wb_addr[1:0], 1'b0 };

	always @(*)
	if (!o_hram_csn)
	begin
		assert(cti_write != fv_cmd[47]);
		assert(cti_dev   == fv_cmd[46]);
		assert(fv_cmd[45]);
		assert(fv_cmd[15:3] == 0);
		assert(fv_cmd[0] == 0);
	end

	always @(*)
		assert(chip_select_count < CK_CSM);

	always @(*)
	if ((!OPT_PIPE)&&((write_stb)||(read_stb)))
		assert(start_stb);

	always @(posedge i_clk)
	if (o_wb_ack)
	begin
		cover(!cti_write);
		cover({cti_write, cti_dev } == 2'b11);

	end else if ((f_past_valid)&&($past(o_wb_ack)))
	begin
		cover((!i_wb_cyc)&&($past(o_wb_ack))&&($past(i_wb_we)));
		cover((!i_wb_cyc)&&($past(o_wb_ack))&&($past(!i_wb_we)));
	end

	generate if (OPT_PIPE)
	begin : COVER_PIPE
		always @(posedge i_clk)
		if (f_past_valid)
		begin
		cover(($past(pipe_stb,3))&&($past(pipe_stb)) &&  cti_write);
		cover(($past(pipe_stb,3))&&($past(pipe_stb)) && !cti_write);
		cover((i_wb_stb)&&(chip_select_warning));
		cover(chip_select_count == 50);
		cover(pipe_req);
		cover(pipe_stb);
		end
	end endgenerate

`ifdef	VERIFIC
	reg	[31:0]	fv_data;
	reg	[3:0]	fv_sel;

	always @(posedge i_clk)
	if (write_stb)
		{ fv_sel, fv_data } <= { i_wb_sel, i_wb_data };

	sequence	COMMAND_SEQ;
		((o_wb_stall)&&(!o_hram_csn)&&(cmd_output)
			&&(o_hram_drive_data)&&(o_hram_cke)
			&&(o_hram_rwctrl==RWDS_INPT) throughout
		(o_hram_data == fv_cmd[47:32])&&(cmd_ctr == 3)
		##1 (o_hram_data == fv_cmd[31:16])&&(cmd_ctr == 2)
		##1 (o_hram_data == fv_cmd[15: 0])&&(cmd_ctr == 1));
	endsequence
		
	sequence	DEV_WRITE_SEQ;
		((!o_hram_csn)&&(cti_write)&&(cti_dev)&&(!cmd_output)
			&&(o_hram_drive_data)&&(o_hram_cke)
			&&(o_hram_rwctrl==RWDS_INPT)
		&&(o_hram_data == fv_data[15:0])
			&&(write_ctr == 1)&&(o_wb_stall))
		##1 (o_wb_ack)&&(o_hram_csn);
	endsequence

	sequence	WRITE_WORD_SEQ;
		(((!o_hram_csn)&&(cti_write)&&(!cti_dev)&&(!cmd_output)
			&&(o_hram_drive_data)&&(o_hram_cke)
			&&(o_hram_rwctrl==RWDS_LCL)) throughout
		(o_hram_data == fv_data[31:16])
			&&(o_hram_rwds == (~fv_sel[3:2]))&&(write_ctr == 2)
			&&(o_wb_stall)
		##1 (o_hram_data == fv_data[15:0])
			&&(o_hram_rwds == (~fv_sel[1:0]))&&(write_ctr == 1)
			&&((o_wb_stall)||(OPT_PIPE)));
	endsequence

	sequence	READ_WORD_SEQ;
		(((!o_hram_csn)&&(!cti_write)&&(!cmd_output)
			&&(!o_hram_drive_data)&&(o_hram_cke)
			&&(o_hram_rwctrl==RWDS_INPT)) throughout
		(read_ctr == 2)&&(o_wb_stall)
		##1 (o_wb_data[15:0] == $past(i_hram_data[15:0]))
			&&(read_ctr == 1)
			&&((o_wb_stall)||(OPT_PIPE)));
	endsequence

	// Command write
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		(start_stb) |=> COMMAND_SEQ);

	// Device register write
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		(start_stb)&&(dev_addr)&&(write_stb)
		|=> COMMAND_SEQ
		##1 DEV_WRITE_SEQ);

	//
	// Memory write
	//
	// Memory write: Double latency count
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		(start_stb)&&(write_stb)&&(!dev_addr)
		##1(((i_hram_rwds)||(OPT_FIXED_LATENCY)) throughout COMMAND_SEQ)
		|=> (o_hram_rwctrl == RWDS_INPT)
		##1 (o_hram_rwds == 2'b11)&&(o_hram_rwctrl == RWDS_LCL)
			&&(data_reg  == fv_data)
			&&(data_mask == fv_sel)
			[*2*LATENCY_COUNT-2]
		##1 WRITE_WORD_SEQ
		##1 (o_wb_ack)&&(
			(o_hram_csn)
			||((OPT_PIPE)&&($past(write_stb)))));

	// Memory write: Single latency count
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		(start_stb)&&(write_stb)&&(!dev_addr)
		##1 (((!i_hram_rwds)&&(!OPT_FIXED_LATENCY))
				throughout COMMAND_SEQ)
		|=> (!o_hram_csn)&&(o_hram_rwctrl == RWDS_INPT)
		##1 (!o_hram_csn)&&(o_hram_rwctrl == RWDS_LCL)
			&&(o_hram_rwds == 2'b11)
			[*(LATENCY_COUNT-2)]
		##1 ((!o_wb_ack) throughout WRITE_WORD_SEQ)
		##1 (o_wb_ack)&&(
			(o_hram_csn)
			||((OPT_PIPE)&&($past(write_stb)))));

	// Memory write, pipelined
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		(pipe_stb)&&(write_stb)
		|-> (write_ctr == 1)&&(!dev_addr)
		##1 WRITE_WORD_SEQ
		##1 (o_wb_ack)&&(
			(o_hram_csn)
			||((OPT_PIPE)&&($past(write_stb)))));

	// 
	// Memory read, single latency
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		((start_stb)&&(read_stb))
		##1(((!i_hram_rwds)&&(!OPT_FIXED_LATENCY)) throughout COMMAND_SEQ)
		|=> (!o_hram_csn)&&(o_hram_cke)
			&&(o_hram_rwctrl == RWDS_INPT)
		##1 (!o_hram_csn)&&(o_hram_cke)&&(!o_hram_drive_data)
			&&(read_ctr > 3)
			&&(o_hram_rwctrl == RWDS_INPT) [*(LATENCY_COUNT-3+1)]
		##1 (!o_hram_csn)&&(o_hram_cke)&&(!o_hram_drive_data)
			&&(read_ctr == 3)
			&&(o_hram_rwctrl == RWDS_INPT)
		##1 ((!o_wb_ack) throughout READ_WORD_SEQ)
		##1 (o_wb_ack)&&(o_wb_data[31:16] == $past(i_hram_data,2))
			&&(o_wb_data[15:0] == $past(i_hram_data))
			&&((o_hram_csn)||((OPT_PIPE)&&($past(read_stb)))));

	// 
	// Memory read, double latency
	assert property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		((start_stb)&&(read_stb))
		##1 ((i_hram_rwds)||(OPT_FIXED_LATENCY) throughout COMMAND_SEQ)
		|=> (!o_hram_csn)&&(o_hram_cke)
			&&(o_hram_rwctrl == RWDS_INPT)
		##1 (!o_hram_csn)&&(o_hram_cke)&&(!o_hram_drive_data)
			&&(read_ctr > 3)
			&&(o_hram_rwctrl == RWDS_INPT) [*(2*LATENCY_COUNT-3+1)]
		##1 (!o_hram_csn)&&(o_hram_cke)&&(!o_hram_drive_data)
			&&(read_ctr == 3)
			&&(o_hram_rwctrl == RWDS_INPT) 
		##1 ((!o_wb_ack) throughout READ_WORD_SEQ)
		##1 (o_wb_ack)&&(o_wb_data[31:16] == $past(i_hram_data,2))
			&&(o_wb_data[15:0] == $past(i_hram_data))
			&&((o_hram_csn)||((OPT_PIPE)&&($past(read_stb)))));

	// 
	// Memory read, pipelined
	assert	property (@(posedge i_clk)
		disable iff ((i_reset)||(!i_wb_cyc))
		((pipe_stb)&&(read_stb))
		|-> (read_ctr == 1)
		##1 READ_WORD_SEQ
		##1 (o_wb_ack)&&(o_wb_data[31:16] == $past(i_hram_data,2))
			&&(o_wb_data[15:0] == $past(i_hram_data))
			&&((o_hram_csn)||((OPT_PIPE)&&($past(read_stb)))));

`endif	// VERIFIC
`endif	// FORMAL
endmodule
