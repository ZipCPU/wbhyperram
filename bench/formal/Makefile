################################################################################
##
## Filename: 	bench/formal/Makefile
##
## Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
##
## Purpose:	To make it easy to run all of my SymbiYosys proofs in this
##		repository from running only one command line program: make..
##
## Creator:	Dan Gisselquist, Ph.D.
##		Gisselquist Technology, LLC
##
################################################################################
##
## Copyright (C) 2018, Gisselquist Technology, LLC
##
## This file is part of the WB-HyperRAM controller project.
##
## The WB-HyperRAM controller project is free software (firmware): you can
## redistribute it and/or modify it under the terms of the GNU Lesser General
## Public License as published by the Free Software Foundation, either version
## 3 of the License, or (at your option) any later version.
##
## The WB-HyperRAM controller project is distributed in the hope that it will
## be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
## General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this program.  (It's in the $(ROOT)/doc directory.  Run make
## with no target there if the PDF file isn't present.)  If not, see
## <http://www.gnu.org/licenses/> for a copy.
##
## License:	LGPL, v3, as defined and found on www.gnu.org,
##		http://www.gnu.org/licenses/lgpl.html
##
##
################################################################################
##
##
all: wbhyperram
RTL := ../../rtl
HRAM := wbhyperram

.PHONY: $(HRAM)
$(HRAM): $(HRAM)_pipe_prf/PASS  $(HRAM)_pipe_cvr/PASS
$(HRAM): $(HRAM)_plain_prf/PASS $(HRAM)_plain_cvr/PASS
$(HRAM)_pipe_prf/PASS:  $(HRAM).sby $(RTL)/$(HRAM).v
	sby -f $(HRAM).sby pipe_prf
$(HRAM)_pipe_cvr/PASS:  $(HRAM).sby $(RTL)/$(HRAM).v
	sby -f $(HRAM).sby pipe_cvr
$(HRAM)_plain_prf/PASS: $(HRAM).sby $(RTL)/$(HRAM).v
	sby -f $(HRAM).sby plain_prf
$(HRAM)_plain_cvr/PASS: $(HRAM).sby $(RTL)/$(HRAM).v
	sby -f $(HRAM).sby plain_cvr

