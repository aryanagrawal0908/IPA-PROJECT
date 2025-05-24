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
        registers[2] = 64'd10;
        registers[3] = 64'd5;
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

endmodule

module instruction_decode (
  input clk,
  input [31:0] inst,
  input [63:0] write_data,
  output [1:0] ALUOp,
  output [63:0] imm_out,
  output [63:0] read_data1,
  output [63:0] read_data2,
  output ALUSrc,
  output Branch,
  output MemWrite,
  output MemRead,
  output MemtoReg
);

  wire RegWrite;

  control_unit c1(.instruction(inst[6:0]), .MemRead(MemRead), .MemWrite(MemWrite), .Branch(Branch), .MemtoReg(MemtoReg), .ALUOp(ALUOp), .ALUSrc(ALUSrc), .RegWrite(RegWrite));
  immediate_generation ig1(.instruction(inst), .imm_out(imm_out));
  reg_file r1(.clk(clk), .rs1(inst[19:15]), .rs2(inst[24:20]), .rd(inst[11:7]), .reg_write(RegWrite), .read_data1(read_data1), .read_data2(read_data2), .write_data(write_data));
endmodule
