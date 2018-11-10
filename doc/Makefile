################################################################################
##
## Filename: 	doc/Makefile
##
## Project:	WB-HyperRAM, a wishbone controller for a hyperRAM interface
##
## Purpose:	To coordinate the build of documentation PDFs from their
##		LaTeX sources.  Requires LaTeX and ps2pdf to build.
##
##	Targets include:
##		all		Builds all documents
##
##		lgpl-3.0.pdf	Builds the LGPL license that everything else is
##				released under.
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
## The WB-HyperRAM project is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
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
.PHONY: all pdf lgpl
all:	pdf
pdf:	lgpl
DSRC := src

.PHONY: lgpl
lgpl: lgpl-3.0.pdf

lgpl-3.0.pdf: $(DSRC)/lgpl-3.0.tex
	latex $(DSRC)/lgpl-3.0.tex
	latex $(DSRC)/lgpl-3.0.tex
	dvips -q -z -t letter -P pdf -o lgpl-3.0.ps lgpl-3.0.dvi
	ps2pdf -dAutoRotatePages=/All lgpl-3.0.ps lgpl-3.0.pdf
	rm lgpl-3.0.dvi lgpl-3.0.log lgpl-3.0.aux lgpl-3.0.ps

.PHONY: clean
clean:
	rm -f lgpl-3.0.pdf
