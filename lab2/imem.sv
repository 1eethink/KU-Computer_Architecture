/* ********************************************
 *	COSE222 Lab #2
 *
 *	Module: instruction memory (imem.sv)
 *	- 1 address input port
 *	- 32-bit 1 data output port
 *	- A single entry size is 32 bit, which is equivalent to the RISC-V instruction size
 *
 *	Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

module imem
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10 )
(
    input   [IMEM_ADDR_WIDTH-1:0]   addr,
    output  [31:0]  dout
);


endmodule