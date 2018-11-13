The wbhyperram core is fairly sparse.  Other than following a WB interface
(B4 spec, pipelined, 32-bit, trimmed of extraneous wires), the core supports
two addressing spaces depending upon the top bit of the address given to it.
If this top bit is a '0', reads from and writes to configuration addresses
are supported.  if the top bit is a '1', reads and writes from the memory
are supported.

The design should be able to read or write many words in succession--up to
a burst lasting four microseconds.

Individual reads and writes will read (or write) 32-bits at a time, in order
to support a 32-bit WB bus.
