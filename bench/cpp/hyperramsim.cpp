////////////////////////////////////////////////////////////////////////////////
//
// Filename: 	bench/cpp/hyperramsim.cpp
//
// Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
//
// Purpose:	Provides the internal guts of the HyperRAM Verilator simulation
//		component.
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
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "hyperramsim.h"

void	HYPERRAMSIM::zero(void) {
	for(unsigned i=0; i<m_len; i++)
		m_mem[i] = 0;

	m_zero = true;
}

void	HYPERRAMSIM::load(const unsigned int addr,
		const char *buf, const size_t len) {
	// Not allowed to load this (currently) so just zero everything
	zero();
}


unsigned HYPERRAMSIM::operator()(int reset_n,
	int cke, int csn, int rwctrl, int rwds, int drive_data, int data) {
	unsigned	result;

	rwds &= 3;
	data &= 0x0ffff;
	result = m_dly_result;
	m_dly_result = rand() & 0x03ffff;


	if ((!reset_n)&&(!m_zero))
		zero();

	if ((!reset_n)||(csn)) {
		m_startctr = 0;
		m_countdown= 12;

		m_dly_cke    =  cke;
		m_dly_rwctrl = !rwctrl;
		m_dly_data   =  data;
		m_dly_rwds   =  rwds;
		m_dly_drive  =  drive_data;

		assert(rwctrl);
		assert(drive_data);

		if (!reset_n) {
			m_latency = 6;
			m_fixed_latency = true;
		}

		if (m_dly_drive)
			m_dly_result = (m_dly_result & ~0x0ffff) | (m_dly_data & 0x0ffff);
		if (m_dly_rwctrl)
			m_dly_result = (m_dly_result & ~0x30000) | (m_dly_rwds << 16);

		return m_dly_result;
	}

	if (m_dly_cke)
		m_startctr++;
	if (m_startctr == 1) {
		assert(!m_dly_rwctrl);

		if (m_fixed_latency)
			m_dly_result = (m_dly_result & 0x0ffff)| (3<<16);
		else
			m_dly_result = (m_dly_result & 0x0ffff)|(((rand()&7)==7)?0x30000:0);

		m_countdown = (m_dly_result&0x30000) ? 2*m_latency : m_latency;
		m_countdown+=2;

		m_cmd[0] = m_dly_data;

		assert((m_cmd[0]&0x2000)==0);
	} else if (m_startctr == 2)
		m_cmd[1] = m_dly_data;
	else if (m_startctr == 3) {
		m_cmd[2] == m_dly_data;
		assert((m_cmd[2]&0xfff8)==0);

		m_addr  = (m_cmd[0]&0x01fff) << (16+3);
		m_addr |=  m_cmd[1] << 3;
		m_addr |=  m_cmd[2] &7;

		assert((m_addr & ~m_mask)==0);
	} else if (m_startctr == 4) {
		// Check for a data write
		if (((m_cmd[0] & 0xc000) == 0x4000)
			&&(m_cmd[1] == 0)
			&&(m_cmd[2] == 0)) {
			m_cfgword = m_dly_data & 0x0ffff;

			if (m_cfgword&0x0f0 == 0x010)
				m_latency = 5;
			else if (m_cfgword&0x0f0 == 0x020)
				m_latency = 6;
			else if (m_cfgword&0x0f0 == 0x0e0)
				m_latency = 3;
			else if (m_cfgword&0x0f0 == 0x0f0)
				m_latency = 4;
			else
				assert((0)&&("Invalid latency"));

			m_fixed_latency = (m_cfgword&4)?1:0;
		}
	} else if (m_startctr == 5) {
		assert((m_cmd[0] & 0xc000) != 0x4000);
	}

	if ((m_startctr != 1)&&(m_countdown > 0))
		m_countdown--;

	if ((m_countdown == 0)&&(m_dly_cke)) {
		bool read = (m_cmd[0]&0x8000);
		bool dev  = (m_cmd[0]&0x4000);

		m_dly_result = (m_dly_result & 0x30000) | (m_mem[m_addr] & 0x0ffff);
		if (!read) {
			assert(m_dly_rwctrl);
			if (m_dly_rwds == 0)
				m_mem[m_addr] = m_dly_data;
			else {
				if ((m_dly_rwds&2)==0)
					m_mem[m_addr] = (m_mem[m_addr]&0x0ff)
							|(m_dly_data&0x0ff00);
				if ((m_dly_rwds&1)==0)
					m_mem[m_addr] = (m_mem[m_addr]&0x0ff00)
							|(m_dly_data&0x0ff);
			}
			if (m_dly_rwds != 3)
				m_zero = false;
		} else {
			assert(!m_dly_rwctrl);
			m_dly_result = (m_dly_result & 0x0ffff) | (2 << 16);
		}
		m_addr++;
	}

	//
	//
	if (!m_dly_rwctrl)
		m_dly_result = (m_dly_result & 0x0ffff)
				|| (m_dly_rwds << 16);
	if (m_dly_drive)
		m_dly_result = (m_dly_result & 0x30000)
				|| (m_dly_data & 0x0ffff);

	return result;
}
