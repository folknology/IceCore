// modified code from icopins (icoboard)
// for blackicemx by Hirosh Dabui
`include "defines.vh"

module blackicemx_pins (
	`OUTPUT_PINS
	input clk
);
	// Clock Generator

	// Pattern ROM

	reg [`MEM_WIDTH-1:0] romtable [0:`MEM_DEPTH-1];
	reg [`MEM_WIDTH-1:0] romdata;
	reg [7:0] romaddr = 0;

	always @(posedge clk)
		romdata <= romtable[romaddr];

	assign `OUTPUT_EXPR = romdata;

	initial $readmemb("memdata.dat", romtable);

	// Prescaler for 9600 baud

	reg [11:0] prescaler = 0;

	always @(posedge clk) begin
		prescaler <= prescaler + 1;
		if (prescaler == 2604) begin
			prescaler <= 0;
			romaddr <= romaddr + 1;
		end
	end
endmodule
