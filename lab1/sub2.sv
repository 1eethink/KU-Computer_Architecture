module sub2(a, b, c);

	input [7:0] a, b;
	output signed [8:0] c;

assign c = a-b;

endmodule