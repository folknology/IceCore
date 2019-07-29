////////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2019, Gisselquist Technology, LLC
//
// This program is free software (firmware): you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTIBILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program.  (It's in the $(ROOT)/doc directory.  Run make with no
// target there if the PDF file isn't present.)  If not, see
// <http://www.gnu.org/licenses/> for a copy.
//
// License:	GPL, v3, as defined and found on www.gnu.org,
//		http://www.gnu.org/licenses/gpl.html
//
//
////////////////////////////////////////////////////////////////////////////////
module sfifo(i_clk, i_wr, i_data, o_full, o_fill, i_rd, o_data, o_empty);
	parameter	BW=8;	// Byte/data width
	parameter 	LGFLEN=4;
	//
	//
	input	wire		i_clk;
	//
	// Write interface
	input	wire		i_wr;
	input	wire [(BW-1):0]	i_data;
	output	reg 		o_full;
	output	reg [LGFLEN:0]	o_fill;
	//
	// Read interface
	input	wire		i_rd;
	output	reg [(BW-1):0]	o_data;
	output	reg		o_empty;	// True if FIFO is empty
	// 

	reg	[(BW-1):0]	fifo_mem[0:(1<<LGFLEN)-1];
	reg	[LGFLEN:0]	wr_addr, rd_addr;
	reg	[LGFLEN-1:0]	rd_next;


	wire	w_wr = (i_wr && !o_full);
	wire	w_rd = (i_rd && !o_empty);

	//
	// Write a new value into our FIFO
	//
	initial	wr_addr = 0;
	always @(posedge i_clk)
	if (w_wr)
		wr_addr <= wr_addr + 1'b1;

	always @(posedge i_clk)
	if (w_wr)
		fifo_mem[wr_addr[(LGFLEN-1):0]] <= i_data;

	//
	// Read a value back out of it
	//
	initial	rd_addr = 0;
	always @(posedge i_clk)
	if (w_rd)
		rd_addr <= rd_addr + 1;

	always @(*)
		o_data = fifo_mem[rd_addr[LGFLEN-1:0]];

	//
	// Return some metrics of the FIFO, it's current fill level,
	// whether or not it is full, and likewise whether or not it is
	// empty
	//
	always @(*)
		o_fill = wr_addr - rd_addr;
	always @(*)
		o_full = o_fill == { 1'b1, {(LGFLEN){1'b0}} };
	always @(*)
		o_empty = (o_fill  == 0);


	always @(*)
		rd_next = rd_addr[LGFLEN-1:0] + 1;

endmodule