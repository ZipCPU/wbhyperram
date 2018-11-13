The basic HyperRAM controller core can be found in [wbhyperram.v].

Other cores in this directory are support cores for specific hardware.
Items prefixed with an 'x' are for Xilinx architectures, and those prefixed
with an 'sb' are for iCE40.  The "top level" which pulls these other cores
together is intended to be the top level of the design, as composed by
[AutoFPGA](https://github.com/ZipCPU/autofpga).
