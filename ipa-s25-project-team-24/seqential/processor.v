`include "instruction_fetch.v"
`include "instruction_decode.v"
`include "execute.v"
`include "data_memory.v"
`include "write_back.v"

`include "alu.v"

module processor (
    input clk,
    input reset
);

  wire [63:0] in, out, PC4;
  wire signed [63:0] write_data, imm_out, read_data1, read_data2, read_data, alu_result;
  wire [31:0] inst;
  wire [1:0] ALUOp;
  wire ALUSrc, Branch, MemWrite, MemRead, MemtoReg;


  instruction_fetch u1(.clk(clk), .rst(reset), .in(in), .out(out), .inst(inst), .PC4(PC4));
  instruction_decode u2(.clk(clk), .inst(inst), .write_data(write_data), .ALUOp(ALUOp), .imm_out(imm_out), .read_data1(read_data1), .read_data2(read_data2),
  .ALUSrc(ALUSrc), .Branch(Branch), .MemWrite(MemWrite), .MemRead(MemRead), .MemtoReg(MemtoReg));

  execute u3(.read_data1(read_data1), .read_data2(read_data2), .imm_out(imm_out), .inst(inst), .ALUOp(ALUOp), .ALUSrc(ALUSrc),
  .Branch(Branch), .out(out), .in(in), .PC4(PC4), .alu_result(alu_result));

  data_memory u4(.MemRead(MemRead), .MemWrite(MemWrite), .address(alu_result), .write_data(read_data2), .read_data(read_data));
  write_back u5(.read_data(read_data), .alu_result(alu_result), .MemtoReg(MemtoReg), .write_data(write_data));
  
  
endmodule

module testbench;

  reg clk, rst;
  processor p1(.clk(clk), .reset(rst));

  always #10 clk = ~clk;

  initial begin
    clk = 1'b1;
    rst = 1'b1;
    #5
    rst = 1'b0;

    #2000
    $finish;
  end

  // initial begin
  //   $monitor("in = %d, out = %d, inst = %b, time = %t", p1.u1.in, p1.u1.out, p1.u1.inst, $time);
  // end

  // initial begin
  //   $monitor("rs1 = %d, rs2 = %d, rd = %d, imm_out = %d, RegWrite = %b, read_data1 = %d, read_data2 = %d, time = %t", p1.inst[19:15], p1.inst[24:20], p1.inst[11:7], p1.u2.imm_out, p1.u2.RegWrite, p1.u2.read_data1, p1.u2.read_data2, $time);
  // end
  initial begin
    $display("and and or result of 10 and 5");
  end
  
  initial begin
    $monitor("and_result = %d, or_result = %d, time = %t", p1.u2.r1.registers[4], p1.u2.r1.registers[5], $time);
  end

  // initial begin
  //   $monitor("rs1 = %d, rs2 = %d, rd = %d, Regwrite = %b, read_data1 = %d, read_data2 = %d, time = %t", p1.u2.inst[19:15], p1.u2.inst[24:20], p1.u2.inst[11:7], p1.u2.RegWrite, p1.u2.read_data1, p1.u2.read_data2, $time);
  // end
  // initial begin
  //   $monitor("in = %d, imm_out = %d, time = %t", p1.u1.out, p1.u2.imm_out, $time);
  // end

  // initial begin
  //   $monitor("rd = %d, time = %t, alu_control = %b", p1.u2.r1.registers[9], $time, p1.u3.control);
  // end

endmodule

