module program_counter(clk,reset,in,out);
    input clk, reset;
    input [63:0] in;
    output reg [63:0] out;

    always @(posedge clk or posedge reset)
    begin
        if(reset)
            out <= 64'b0;
        else 
            out <= in;
    end

endmodule

module instruction_memory (
    input [63:0] addr,
    output reg [31:0] inst
);

  reg [31:0] mem[63:0];

  initial begin
    mem[0] = 32'b00000000001100010111001000110011;
    mem[4] = 32'b00000000001100010110001010110011;
  end

  always @(*) begin
    inst <= mem[addr];
  end

endmodule

module instruction_fetch (
  input clk,
  input rst,
  input [63:0] in,
  output [63:0] out,
  output [31:0] inst,
  output [63:0] PC4
);

  program_counter p1(.clk(clk), .reset(rst), .in(in), .out(out));
  adder a1(.A(out), .B(64'd4), .M(1'b0), .S(PC4));
  instruction_memory im1(.addr(out), .inst(inst));
  
endmodule