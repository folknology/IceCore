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
module chip (
  input	wire		i_clk,
	input	wire		i_uart_rx,
	output	wire		o_uart_tx,
	output	wire	[1:0]	o_led,
		);
	// 115200 Baud, if clk @ 100MHz
  parameter CLOCK_RATE_HZ = 25_000_000;
  parameter BAUD_RATE = 115_200;
  parameter CLOCKS_PER_BAUD = (CLOCK_RATE_HZ/BAUD_RATE);

	wire		rx_stb, tx_busy, fifo_empty, fifo_valid,
			fifo_full;
	wire	[7:0]	rx_data, tx_data;
	wire	[8:0]	fifo_fill;
	reg	[7:0]	line_count;
	reg		run_tx, tx_stb, fifo_rd;


	rxuart	#(.CLOCKS_PER_BAUD(CLOCKS_PER_BAUD))
		receiver(i_clk, i_uart_rx, rx_stb, rx_data);

	reg	[24:0] rx_counter;
	initial rx_counter= 0;
	always @(posedge i_clk)
	if (rx_stb)
		rx_counter <= -1;
	else if (rx_counter != 0)
		rx_counter <= rx_counter - 1;
	assign	o_led[0] = ~rx_counter[24];

	sfifo	#(.BW(8), .LGFLEN(8))
		fifo(i_clk, rx_stb, rx_data, fifo_full, fifo_fill,
			fifo_rd, tx_data, fifo_empty);

	reg	[25:0] fifo_counter;
	initial fifo_counter= 0;
	always @(posedge i_clk)
	if (!fifo_empty)
		fifo_counter <= -1;
	else if (fifo_counter != 0)
		fifo_counter <= fifo_counter - 1;
	assign	o_led[1] = ~fifo_counter[25];


	assign	fifo_valid = !fifo_empty;

	// Here's the guts of the algorithm--setting run_tx.  Once set, the
	// buffer will flush.  Here, we set it on one of two conditions: 1)
	// a newline is received, or 2) the line is now longer than 80
	// characters.
	//
	// Once the line has ben transmitted (separate from emptying the buffer)
	// we stop transmitting.
	initial	run_tx = 0;
	initial	line_count = 0;
	always @(posedge i_clk)
	if (rx_stb && (rx_data == 8'ha || rx_data == 8'hd))
	begin
		run_tx <= 1'b1;
		line_count <= fifo_fill[7:0];
	end else if (!run_tx)
	begin
		if (fifo_fill >= 9'd80)
		begin
			run_tx <= 1'b1;
			line_count <= 80;
		end else if (fifo_valid && (tx_data == 8'ha || tx_data == 8'hd))
		begin
			run_tx <= 1'b1;
			line_count <= 1;
		end
	end else if (!fifo_empty && !tx_busy) begin // if (run_tx)
		line_count <= line_count - 1;
		if (line_count == 1)
			run_tx <= 0;
	end

	always @(*)
		fifo_rd = (tx_stb && !tx_busy);

	// When do we wish to transmit?
	//
	// Any time run_tx is true--but we'll give it an extra clock.
	always @(*)
		tx_stb = (run_tx && !fifo_empty && fifo_valid);

	txuart	#(.CLOCKS_PER_BAUD(CLOCKS_PER_BAUD))
		transmitter(i_clk, tx_stb, tx_data, o_uart_tx, tx_busy);

endmodule
