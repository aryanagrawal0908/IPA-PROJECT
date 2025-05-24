// `include "alu.v"

module PCBranch_calculator (
    input [63:0] imm_out,
    input [63:0] PC_id,
    output [63:0] PCbranch_id
);
    wire [63:0] w1;

    SL1 s1(.A(imm_out), .C(w1));
    adder a1(.A(PC_id), .B(w1), .M(1'b0), .S(PCbranch_id));
    
endmodule

module flusher (
    input Branch_id,
    input [63:0] read_data1_id,
    input [63:0] read_data2_id,

    output reg IFIDflush
);
    initial begin
        IFIDflush = 1'b0;
    end

    always @(*) begin
        if(Branch_id && (read_data1_id == read_data2_id)) begin
            IFIDflush = 1'b1;
        end
        else begin
            IFIDflush = 1'b0;
        end
    end
       
endmodule

module Hazard_Detection_Unit (
    input [4:0] rs1,           
    input [4:0] rs2,           
    input [4:0] ex_rd,         
    input mem_read_ex,         
    output reg mux_sel,          
    output reg IFIDnotWrite,
    output reg PCnotWrite        
);

initial begin
    PCnotWrite = 1'b0;
    mux_sel = 1'b0;
    IFIDnotWrite = 1'b0;
end

always @(*) begin
    if (mem_read_ex && ((ex_rd == rs1) || (ex_rd == rs2))) begin
        IFIDnotWrite <= 1'b1;
        PCnotWrite <= 1'b1;
        mux_sel <= 1'b1;
    end else begin
        IFIDnotWrite <= 1'b0;
        PCnotWrite <= 1'b0;
        mux_sel <= 1'b0;
    end
end

endmodule

module reg_file (
    input clk,
    input reg_write,
    input [4:0] rs1, rs2, rd,
    input [63:0] write_data,
    output [63:0] read_data1, read_data2
);

    reg [63:0] registers [31:0];  

    initial begin
        registers[0] = 64'b0; 
    end

    always @(negedge clk) begin
        if (reg_write && rd != 0) begin
            registers[rd] <= write_data;
        end      
    end

    assign read_data1 = registers[rs1];
    assign read_data2 = registers[rs2];

endmodule


module immediate_generation (
    input  [31:0] instruction,
    output reg signed [63:0] imm_out
);

  wire [6:0] opcode = instruction[6:0];

    always @(*) begin
        case (opcode) 
            7'b0000011: // I-Type (Load instructions like ld)
                imm_out = {{52{instruction[31]}}, instruction[31:20]};

            7'b0100011: // S-Type (Store instructions like sd)
                imm_out = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};

            7'b1100011: // B-Type (Branch instructions like beq, bne)
                imm_out = {{51{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};

            default:
                imm_out = 64'b0; // Default case for unknown opcodes
        endcase
    end

endmodule


module control_unit(
    input [6:0] instruction,
    input mux_sel,              // inserted mux_sel instead of mux
    output reg MemWrite,
    output reg MemRead,
    output reg MemtoReg,
    output reg [1:0] ALUOp,
    output reg Branch,
    output reg ALUSrc,
    output reg RegWrite
);

    always @(*) 
    begin
        if (mux_sel) begin 
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            RegWrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00; 
        end
        else begin

            case (instruction)
                7'b0110011: // R-type
                begin
                    ALUSrc = 1'b0;
                    MemtoReg = 1'b0;
                    RegWrite = 1'b1;
                    MemRead = 1'b0;
                    MemWrite = 1'b0;
                    Branch = 1'b0;
                    ALUOp = 2'b10; 
                end
                
                7'b0000011: // ld (Load)
                begin
                    ALUSrc = 1'b1; 
                    MemtoReg = 1'b1;
                    RegWrite = 1'b1; 
                    MemRead = 1'b1; 
                    MemWrite = 1'b0; 
                    Branch = 1'b0;
                    ALUOp = 2'b00; 
                end
                
                7'b0100011: // sd (Store)
                begin
                    ALUSrc = 1'b1; 
                    MemtoReg = 1'b0; 
                    RegWrite = 1'b0; 
                    MemRead = 1'b0; 
                    MemWrite = 1'b1; 
                    Branch = 1'b0; 
                    ALUOp = 2'b00;
                end
                
                7'b1100011: // beq (Branch if Equal)
                begin
                    ALUSrc = 1'b0; 
                    MemtoReg = 1'b0; 
                    RegWrite = 1'b0;
                    MemRead = 1'b0; 
                    MemWrite = 1'b0;
                    Branch = 1'b1; 
                    ALUOp = 2'b01; 
                end

                default:
                begin
                    ALUSrc = 1'b0;
                    MemtoReg = 1'b0;
                    RegWrite = 1'b0;
                    MemRead = 1'b0;
                    MemWrite = 1'b0;
                    Branch = 1'b0;
                    ALUOp = 2'b00;
                end
            endcase
        end
    end

endmodule

module instruction_decode (
  input clk,
  input [31:0] inst_id,
  input [63:0] Result_wb,
  input RegWrite_wb,
  input [4:0] rd_wb,
  input [4:0] rd_ex,
  input MemRead_ex,
  input [63:0] PC_id,
  output [1:0] ALUOp_id,
  output [63:0] imm_out_id,
  output [63:0] read_data1_id,
  output [63:0] read_data2_id,
  output [63:0] PCbranch_id,
  output ALUSrc_id,
  output Branch_id,
  output MemWrite_id,
  output MemRead_id,
  output RegWrite_id,
  output MemtoReg_id,
  output PCnotWrite,
  output IFIDnotWrite,
  output IFIDflush,
  output PCSrc
);
  
  wire mux_sel;
  wire zero;

  assign zero = (read_data1_id == read_data2_id);
  and(PCSrc, zero, Branch_id);
  

  PCBranch_calculator pb1(.imm_out(imm_out_id), .PC_id(PC_id), .PCbranch_id(PCbranch_id));

  control_unit  c1(.instruction(inst_id[6:0]), .mux_sel(mux_sel), .MemWrite(MemWrite_id),
                    .MemRead(MemRead_id), .MemtoReg(MemtoReg_id), .ALUOp(ALUOp_id), .Branch(Branch_id),
                    .ALUSrc(ALUSrc_id), .RegWrite(RegWrite_id));

  immediate_generation ig1(.instruction(inst_id), .imm_out(imm_out_id));

  reg_file      r1(.clk(clk), .reg_write(RegWrite_wb), .rs1(inst_id[19:15]), .rs2(inst_id[24:20]), .rd(rd_wb),
                    .write_data(Result_wb), .read_data1(read_data1_id), .read_data2(read_data2_id));

  Hazard_Detection_Unit h1(.rs1(inst_id[19:15]), .rs2(inst_id[24:20]), .ex_rd(rd_ex), .mem_read_ex(MemRead_ex),
                            .IFIDnotWrite(IFIDnotWrite), .PCnotWrite(PCnotWrite), .mux_sel(mux_sel));

  flusher f1(.Branch_id(Branch_id), .read_data1_id(read_data1_id), .read_data2_id(read_data2_id), .IFIDflush(IFIDflush));


endmodule
