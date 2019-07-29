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
module rxuart(
    input	wire		i_clk,
	input	wire		i_uart_rx,
	output	reg		o_wr,
    output	reg	[7:0]	o_data
	);

	parameter  [15:0]	CLOCKS_PER_BAUD = 868;
	//
	localparam	[3:0]	IDLE      = 4'h0;
	localparam	[3:0]	BIT_ZERO  = 4'h1;
	// localparam	[3:0]	BIT_ONE   = 4'h2;
	// localparam	[3:0]	BIT_TWO   = 4'h3;
	// localparam	[3:0]	BIT_THREE = 4'h4;
	// localparam	[3:0]	BIT_FOUR  = 4'h5;
	// localparam	[3:0]	BIT_FIVE  = 4'h6;
	// localparam	[3:0]	BIT_SIX   = 4'h7;
	// localparam	[3:0]	BIT_SEVEN = 4'h8;
	localparam	[3:0]	STOP_BIT  = 4'h9;

	reg	[3:0]		state;
	reg	[15:0]		baud_counter;
	reg			zero_baud_counter;

	// 2FF Synchronizer
	//
	reg		ck_uart;
	reg		q_uart;
	initial	{ ck_uart, q_uart } = -1;
	always @(posedge i_clk)
		{ ck_uart, q_uart } <= { q_uart, i_uart_rx };

	initial	state = IDLE;
	initial	baud_counter = 0;
	always @(posedge i_clk)
	if (state == IDLE)
	begin
		state <= IDLE;
		baud_counter <= 0;
		if (!ck_uart)
		begin
			state <= BIT_ZERO;
			baud_counter <= CLOCKS_PER_BAUD+CLOCKS_PER_BAUD/2-1'b1;
		end
	end else if (zero_baud_counter)
	begin
		state <= state + 1;
		baud_counter <= CLOCKS_PER_BAUD-1'b1;
		if (state == STOP_BIT)
		begin
			state <= IDLE;
			baud_counter <= 0;
		end
	end else
		baud_counter <= baud_counter - 1'b1;

	always @(*)
		zero_baud_counter = (baud_counter == 0);

	always @(posedge i_clk)
	if ((zero_baud_counter)&&(state != STOP_BIT))
		o_data <= { ck_uart, o_data[7:1] };

	initial	o_wr = 1'b0;
	always @(posedge i_clk)
		o_wr <= ((zero_baud_counter)&&(state == STOP_BIT));

endmodule