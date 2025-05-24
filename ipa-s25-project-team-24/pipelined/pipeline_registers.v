module IF_ID (
    input clk,
    input reset,
    input flush,
    input IFIDnotWrite,                
    input [31:0] instr_in,     
    input [63:0] pc_in,         
    output reg [31:0] instr_out,
    output reg [63:0] pc_out    
);

always @(posedge clk or posedge reset) begin
    if (reset || flush) begin
        instr_out <= 64'b0;
        pc_out <= 64'b0;
    end else begin
        if (!IFIDnotWrite) begin
            instr_out <= instr_in;
            pc_out <= pc_in;
        end
    end
end

endmodule

module ID_EX (
    input clk,
    input reset,
    input [31:0] inst_id,
    input [63:0] reg_data1_in,  
    input [63:0] reg_data2_in,  
    input [63:0] imm_in,       
    input [4:0] rd_in,         
    input [4:0] rs1_in,         
    input [4:0] rs2_in,         
    input [1:0] alu_ctrl_in,    
    input mem_read_in,          
    input mem_write_in,         
    input reg_write_in,         
    input mem_to_reg_in,
    input ALUSrc_id,
    output reg [31:0] inst_ex,        
    output reg [63:0] reg_data1_out,
    output reg [63:0] reg_data2_out,
    output reg [63:0] imm_out,
    output reg [4:0] rd_out,
    output reg [4:0] rs1_out,
    output reg [4:0] rs2_out,
    output reg [1:0] alu_ctrl_out,
    output reg mem_read_out,
    output reg mem_write_out,
    output reg reg_write_out,
    output reg mem_to_reg_out,
    output reg ALUSrc_ex
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        reg_data1_out <= 0;
        reg_data2_out <= 0;
        imm_out <= 0;
        rd_out <= 0;
        rs1_out <= 0;
        rs2_out <= 0;
        alu_ctrl_out <= 0;
        mem_read_out <= 0;
        mem_write_out <= 0;
        reg_write_out <= 0;
        mem_to_reg_out <= 0;
        ALUSrc_ex <= 0;
        inst_ex <= 0;
    end else begin
        reg_data1_out <= reg_data1_in;
        reg_data2_out <= reg_data2_in;
        imm_out <= imm_in;
        rd_out <= rd_in;
        rs1_out <= rs1_in;
        rs2_out <= rs2_in;
        alu_ctrl_out <= alu_ctrl_in;
        mem_read_out <= mem_read_in;
        mem_write_out <= mem_write_in;
        reg_write_out <= reg_write_in;
        mem_to_reg_out <= mem_to_reg_in;
        ALUSrc_ex <= ALUSrc_id;
        inst_ex <= inst_id;
    end
end

endmodule

module EX_MEM (
    input clk,
    input reset,
    input RegWrite_in,
    input MemRead_in,
    input MemWrite_in,
    input MemtoReg_in,
    input [1:0] ALUOp_in,
    input [63:0] ALU_result_in,
    input [63:0] write_data_ex, 
    input [4:0] rd_in,

    output reg RegWrite_out,
    output reg MemRead_out,
    output reg MemWrite_out,
    output reg MemtoReg_out,
    output reg [1:0] ALUOp_out,
    output reg [63:0] ALU_result_out,
    output reg [63:0] write_data_mem, 
    output reg [4:0] rd_out
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        RegWrite_out   <= 0;
        MemRead_out    <= 0;
        MemWrite_out   <= 0;
        MemtoReg_out   <= 0;
        ALUOp_out      <= 2'b00;
        ALU_result_out <= 64'b0;
        write_data_mem <= 64'b0;
        rd_out         <= 5'b0;
    end else begin
        RegWrite_out   <= RegWrite_in;
        MemRead_out    <= MemRead_in;
        MemWrite_out   <= MemWrite_in;
        MemtoReg_out   <= MemtoReg_in;
        ALUOp_out      <= ALUOp_in;
        ALU_result_out <= ALU_result_in;
        write_data_mem <= write_data_ex;
        rd_out         <= rd_in;
    end
end

endmodule

module MEM_WB (
    input clk,
    input reset,
    input RegWrite_in,
    input MemtoReg_in,
    input [63:0] ALU_result_in,
    input [63:0] mem_data_in,
    input [4:0] rd_in,

    output reg RegWrite_out,
    output reg MemtoReg_out,
    output reg [63:0] ALU_result_out,
    output reg [63:0] mem_data_out,
    output reg [4:0] rd_out
);

always @(posedge clk or posedge reset) begin
    if (reset) begin
        RegWrite_out   <= 0;
        MemtoReg_out   <= 0;
        ALU_result_out <= 64'b0;
        mem_data_out   <= 64'b0;
        rd_out         <= 5'b0;
    end else begin
        RegWrite_out   <= RegWrite_in;
        MemtoReg_out   <= MemtoReg_in;
        ALU_result_out <= ALU_result_in;
        mem_data_out   <= mem_data_in;
        rd_out         <= rd_in;
    end
end

endmodule
