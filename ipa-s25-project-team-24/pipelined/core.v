`include "alu.v"

`include "instruction_fetch.v"
`include "instruction_decode.v"
`include "execute.v"
`include "data_memory.v"
`include "write_back.v"

`include "pipeline_registers.v"


module processor (
  input clk,
  input reset
);
  wire [63:0] PC_if, PC_id, PCbranch_id;

  wire [31:0] inst_if, inst_id, inst_ex;

  wire [63:0] Result_wb;
  wire [63:0] ALU_result_ex, ALU_result_mem, ALU_result_wb;

  wire [63:0] imm_out_id, imm_out_ex;

  wire [63:0] read_data1_id, read_data1_ex;
  wire [63:0] read_data2_id, read_data2_ex;
  wire [63:0] read_data_mem, read_data_wb;
  wire [63:0] write_data_ex, write_data_mem;

  wire RegWrite_id, RegWrite_ex, RegWrite_mem, RegWrite_wb;
  wire MemRead_id, MemRead_ex, MemRead_mem;
  wire ALUSrc_id, ALUSrc_ex;
  wire Branch_id;
  wire MemWrite_id, MemWrite_ex, MemWrite_mem;
  wire MemtoReg_id, MemtoReg_ex, MemtoReg_mem, MemtoReg_wb;
  wire [1:0] ALUOp_id, ALUOp_ex, ALUOp_mem;

  wire [4:0] rd_id, rd_ex, rd_mem, rd_wb;
  wire [4:0] rs1_id, rs1_ex;
  wire [4:0] rs2_id, rs2_ex;

  assign rs1_id = inst_id[19:15];
  assign rs2_id = inst_id[24:20];
  assign rd_id = inst_id[11:7];

  wire PCnotWrite, IFIDnotWrite, IFIDflush;
  wire PCSrc;

  instruction_fetch   if1(.clk(clk), .reset(reset), .PCnotWrite(PCnotWrite), .PCSrc(PCSrc), .PCbranch(PCbranch_id), .PC(PC_if), .inst(inst_if));
  IF_ID               r1(.clk(clk), .reset(reset), .IFIDnotWrite(IFIDnotWrite), .instr_in(inst_if), .instr_out(inst_id), .pc_in(PC_if), .pc_out(PC_id), .flush(IFIDflush));

  instruction_decode  id1(.clk(clk), .inst_id(inst_id), .Result_wb(Result_wb), .RegWrite_wb(RegWrite_wb), .MemRead_ex(MemRead_ex), 
                          .ALUOp_id(ALUOp_id), .rd_wb(rd_wb), .imm_out_id(imm_out_id), .read_data1_id(read_data1_id), .rd_ex(rd_ex), .PCSrc(PCSrc),
                          .read_data2_id(read_data2_id), .ALUSrc_id(ALUSrc_id), .Branch_id(Branch_id), .MemWrite_id(MemWrite_id), .MemRead_id(MemRead_id),
                          .RegWrite_id(RegWrite_id), .MemtoReg_id(MemtoReg_id), .PCnotWrite(PCnotWrite), .IFIDnotWrite(IFIDnotWrite), .PC_id(PC_id), .PCbranch_id(PCbranch_id), .IFIDflush(IFIDflush));
  
  ID_EX               r2(.clk(clk), .reset(reset), .reg_data1_in(read_data1_id), .reg_data2_in(read_data2_id), .imm_in(imm_out_id), .rd_in(rd_id),
                          .rs1_in(rs1_id), .rs2_in(rs2_id), .alu_ctrl_in(ALUOp_id), .mem_read_in(MemRead_id), .mem_write_in(MemWrite_id), .reg_write_in(RegWrite_id),
                          .mem_to_reg_in(MemtoReg_id), .reg_data1_out(read_data1_ex), .reg_data2_out(read_data2_ex), .imm_out(imm_out_ex), .rd_out(rd_ex),
                          .rs1_out(rs1_ex), .rs2_out(rs2_ex), .alu_ctrl_out(ALUOp_ex), .mem_read_out(MemRead_ex), .mem_write_out(MemWrite_ex), .reg_write_out(RegWrite_ex),
                          .mem_to_reg_out(MemtoReg_ex), .ALUSrc_id(ALUSrc_id), .ALUSrc_ex(ALUSrc_ex), .inst_id(inst_id), .inst_ex(inst_ex));

  execute             ex1(.imm_out_ex(imm_out_ex), .read_data1_ex(read_data1_ex), .read_data2_ex(read_data2_ex), .ALU_result_mem(ALU_result_mem), .Result_wb(Result_wb),
                          .rs1_ex(rs1_ex), .rs2_ex(rs2_ex), .rd_mem(rd_mem), .rd_wb(rd_wb), .RegWrite_mem(RegWrite_mem), .RegWrite_wb(RegWrite_wb), .inst_ex(inst_ex), .ALUOp_ex(ALUOp_ex),
                          .ALUSrc_ex(ALUSrc_ex), .ALU_result_ex(ALU_result_ex), .write_data_ex(write_data_ex));
  
  EX_MEM              r3(.clk(clk), .reset(reset), .RegWrite_in(RegWrite_ex), .MemRead_in(MemRead_ex), .MemWrite_in(MemWrite_ex), .MemtoReg_in(MemtoReg_ex), .ALUOp_in(ALUOp_ex),
                          .ALU_result_in(ALU_result_ex), .write_data_ex(write_data_ex), .rd_in(rd_ex), .RegWrite_out(RegWrite_mem), .MemRead_out(MemRead_mem), .MemWrite_out(MemWrite_mem),
                          .MemtoReg_out(MemtoReg_mem), .ALUOp_out(ALUOp_mem), .ALU_result_out(ALU_result_mem), .write_data_mem(write_data_mem), .rd_out(rd_mem));
  
  data_memory         dm1(.MemRead_mem(MemRead_mem), .MemWrite_mem(MemWrite_mem), .address(ALU_result_mem), .write_data(write_data_mem), .read_data_mem(read_data_mem));

  MEM_WB              r4(.clk(clk), .reset(reset), .RegWrite_in(RegWrite_mem), .MemtoReg_in(MemtoReg_mem), .ALU_result_in(ALU_result_mem), .mem_data_in(read_data_mem), .rd_in(rd_mem), .RegWrite_out(RegWrite_wb),
                          .MemtoReg_out(MemtoReg_wb), .ALU_result_out(ALU_result_wb), .mem_data_out(read_data_wb), .rd_out(rd_wb));

  write_back          wb1(.read_data(read_data_wb), .alu_result(ALU_result_wb), .MemtoReg(MemtoReg_wb), .write_data(Result_wb));

endmodule

