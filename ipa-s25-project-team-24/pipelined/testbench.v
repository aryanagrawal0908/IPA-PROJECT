`include "core.v"

module tb_core;
  reg clk;
  reg reset;

  processor p1(.reset (reset),.clk (clk));

  localparam CLK_PERIOD = 20;
  always #(CLK_PERIOD/2) clk=~clk;

  initial begin
    $dumpfile("tb_core.vcd");
    $dumpvars(0, tb_core);

    #1200;
    $finish;
  end

  initial begin
    // register values
    p1.id1.r1.registers[1] = 64'd1;
    p1.id1.r1.registers[4] = 64'd13;

    // memory values
    p1.dm1.memory[20] = 64'd16;
    


    //instructions
    p1.if1.im1.mem[0] = 32'b00000000000000000000001001100011; // beq x0, x0, 4
    p1.if1.im1.mem[8] = 32'b00000000010000001000000010110011; // add x1 x1 x4
    

    // p1.if1.im1.mem[0] = 32'b00000000000000000011000010000011; // ld x1 0(x0)
    // p1.if1.im1.mem[4] = 32'b00000000000000001011100110000011; // ld x19 0(x1)
    // p1.if1.im1.mem[8] = 32'b00000000000000000000010100110011; // add x10 x0 x0
    // p1.if1.im1.mem[12] = 32'b00000000000010011000010001100011; // beq x19 x0 8
    // p1.if1.im1.mem[0] = 32'b00000001001101010000010100110011; // add x10 x10 x19
    // p1.if1.im1.mem[20] = 32'b01000000000110011000100110110011; // sub x19 x19 x1
    // p1.if1.im1.mem[24] = 32'b11111110000000000000110111100011; // beq x0 x0 -6
    // p1.if1.im1.mem[28] = 32'b00000000101000010011000000100011; // sd x10 0(x2)

  end

  initial begin
    clk = 1'b1;
    reset = 1'b1;
    #5
    reset = 1'b0;


    // $monitor("PC_if = %d, instruction_if = %b, PCnotWrite = %d, time = %t", p1.PC_if, p1.inst_if, p1.if1.p1.PCnotWrite ,$time);

    // $monitor("PC_id = %d, rs1 = %d, rs2 = %d, rd = %d, inst_id = %b, inst_ex = %b, time = %t", p1.PC_id, p1.rs1_id, p1.rs2_id, p1.rd_id, p1.inst_id, p1.inst_ex, $time);

    // $monitor("rd_1 = %d, rd_2 = %d, alu_result = %d, imm_out = %d, time = %t", p1.ex1.A1, p1.ex1.B1, p1.ex1.ALU_result_ex, p1.ex1.imm_out_ex, $time);
    
    // $monitor("rd_1 = %d, res2 = %d, alu_result = %d, write_data = %d, time = %t", p1.ex1.B1, p1.ex1.res2, p1.ex1.ALU_result_ex, p1.ex1.write_data_ex, $time);

    // $monitor("write_data = %d, memory = %d, address = %d, reg_result = %d, MemWrite = %b, MemRead = %b, time = %t", p1.dm1.write_data, p1.dm1.memory[11], p1.dm1.address, p1.id1.r1.registers[26], p1.dm1.MemWrite_mem, p1.dm1.MemRead_mem, $time);

    // $monitor("register value = %d, memory loc = %d, time = %t", p1.id1.r1.registers[10], p1.dm1.memory[11], $time);

    //$monitor("result = %d,  PC_if = %d, PC_id = %d,  time = %t", p1.ex1.ALU_result_ex, p1.PC_if, p1.PC_id, $time);

    $monitor("result = %d, time = %t", p1.id1.r1.registers[1], $time);
  end

endmodule
