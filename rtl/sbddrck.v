////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	rtl/sbddrck.v
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	Creates the final clock output, using an iCE40 specific
//		I/O primitive.
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
`default_nettype	none
//
module	sbddrck(i_clk, i_ddr, o_pin);
	input	wire		i_clk;
	input	wire		i_en;
	output	wire	[1:0]	o_pin;

	SB_IO	#(.PIN_TYPE(6'b0100_01)
	   ) oddr(
		.OUTPUT_CLK(i_clk),
		.CLOCK_ENABLE(1'b1),
		.D_OUT_0(!i_en),
		.D_OUT_1(1'b1),
		.OUTPUT_ENABLE(1),
		.PACKAGE_PIN(o_pin[1]));


	SB_IO	#(.PIN_TYPE(6'b0100_01)
	   ) oddr(
		.OUTPUT_CLK(i_clk),
		.CLOCK_ENABLE(1'b1),
		.D_OUT_0(i_en),
		.D_OUT_1(1'b0),
		.OUTPUT_ENABLE(1),
		.PACKAGE_PIN(o_pin[0]));

endmodule
