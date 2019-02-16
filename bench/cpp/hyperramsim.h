////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	bench/cpp/hyperramsim.h
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	Create a Verilator based HyperRAM simulation
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
#ifndef	HYPERRAMSIM_H
#define	HYPERRAMSIM_H

#include <stdint.h>

class	HYPERRAMSIM {
private:
	uint16_t	*m_mem;
	unsigned	m_len, m_mask, m_cfgword;
	unsigned	m_startctr, m_addr, m_latency;
	bool		m_fixed_latency, m_zero;

	unsigned	m_dly_cke, m_dly_rwctrl, m_dly_data, m_dly_rwds,
			m_dly_drive, m_dly_result,
			m_countdown;
	unsigned	m_cmd[3];

	void	zero(void);
public:
	HYPERRAMSIM(unsigned lgsize) {
		m_latency = 6;
		m_len  = (1<<lgsize);
		m_mask = m_len - 1;
		m_mem  = new uint16_t[(1<<lgsize)];
		zero();
	}

	void	load(const unsigned int addr, const char *buf,const size_t len);
	uint16_t &operator[](const unsigned addr) {
		return m_mem[addr & m_mask];
	}

	unsigned apply(int reset_n, int cke, int csn, int rwctrl, int rwds,
			int drive_data, int data);

	unsigned operator()(int reset_n, int cke, int csn, int rwctrl, int rwds,
			int drive_data, int data) {
		return apply(reset_n, cke, csn, rwctrl, rwds, drive_data, data);
	}

	static inline unsigned DAT(unsigned vl) { return (vl&0x0ffff); }
	static inline unsigned RWS(unsigned vl) { return (vl >> 16)&3; }
};

#endif
