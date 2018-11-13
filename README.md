## WB Controlled HyperRAM controller

This repository is a place-holder for a WB controlled HyperRAM controller.
The goal of this controller is to be able to match the HyperRAM's speed
from within an FPGA.  As such, some extra hardware support is required
(SB_IO, IDDR, ODDR, OBUF, etc.).  This support is dependent upon the
FPGA environment it will be applied within.  I expect to provide support for
both Xilinx and iCE40, while leaving support for other platforms as a very
real possibility.

The controller is currently a work in progress.

You may be able to find parts and pieces of the (not-yet-working) controller
on the [dev branch](tree/dev)
