////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	xoddrck.v
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	When a clock needs to be regenerated, Xilinx recommends using
//		an ODDR primitive.  This tries to simplify that process.
//
//	This particular ODDR primitive is unique for the clock enable, and
//	for being followed by an OBUFDS.  If the clock enable is high,
//	the clock will alternate.  If not, the clock will idle high.
//
// Creator:	Dan Gisselquist, Ph.D.
//		Gisselquist Technology, LLC
//
////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2018-2019, Gisselquist Technology, LLC
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
module	xoddrck(i_clk, i_en, o_pin);
	input	wire		i_clk;
	input	wire [1:0]	i_en;
	output	wire [1:0]	o_pin;

	wire	w_internal;

	ODDR #(
		.DDR_CLK_EDGE("SAME_EDGE"),
		.INIT(1'b0),
		.SRTYPE("SYNC")
	) ODDRi(
		.Q(w_internal),
		.C(i_clk),
		.CE(1'b1),
		.D1(1'b1),	// Negative clock edge (goes first)
		.D2(!i_en));	// Positive clock edge

	OBUFDS dsbuf(.I(w_internal), .O(o_pin[1]), .OB(o_pin[0]));

endmodule
