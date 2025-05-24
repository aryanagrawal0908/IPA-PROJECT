// `include "alu.v"



module program_counter(clk,reset,PCnotWrite,in,out);
    input clk, reset;
    input PCnotWrite;
    input [63:0] in;
    output reg [63:0] out;

    always @(posedge clk or posedge reset)
    begin
        if(reset)
            out <= 64'b0;
        else
          if(!PCnotWrite) begin 
            out <= in;
          end
    end

endmodule

module instruction_memory (
    input [63:0] addr,
    output reg [31:0] inst
);

  reg [31:0] mem[63:0];


  always @(*) begin
    inst <= mem[addr];
  end

endmodule

module instruction_fetch (
  input clk,
  input reset,
  input PCnotWrite,
  input PCSrc,
  input [63:0] PCbranch,
  output [63:0] PC,
  output [31:0] inst
);

  wire [63:0] in, PC4;

  mux m1(.a(PC4), .b(PCbranch), .sel(PCSrc), .out(in));
  adder a1(.A(PC), .B(64'd4), .M(1'b0), .S(PC4));
  program_counter p1(.clk(clk), .reset(reset), .PCnotWrite(PCnotWrite), .in(in), .out(PC));
  instruction_memory im1(.addr(PC), .inst(inst));
  
  
endmodule