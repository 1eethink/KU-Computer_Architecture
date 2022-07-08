`timescale 1ns/1ps

module tb_xor2();

reg [7:0] a, b;
reg [7:0] c;
reg [7:0] x;

initial 
begin
a=0; b=255;

for (x=0; x<255; x=x+1)
begin
#(10) a=a+1; b=b-1;
end

end

xor2 u0(a,b,c);


endmodule