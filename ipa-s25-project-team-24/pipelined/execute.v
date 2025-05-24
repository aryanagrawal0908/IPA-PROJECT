`include "alu.v"

module mux(a,b,sel,out);
    input [63:0] a,b;
    input sel;
    output [63:0] out;
    reg [63:0] out;
    always @(*)
    begin
        if(sel)
            out <= b;
        else
            out <= a;
    end

endmodule

module mux3 (
    input [63:0] A,
    input [63:0] B,
    input [63:0] C,
    input [1:0] sel,
    output reg [63:0] D
);

    always @(*) begin
        case (sel)
            2'b00: D <= A;
            2'b01: D <= B;
            2'b10: D <= C;
        endcase
    end
    
endmodule

module forwarding_unit (
    input [4:0] rs1_ex, rs2_ex,  
    input [4:0] rd_mem,            
    input [4:0] rd_wb,
    input RegWrite_mem,             
    input RegWrite_wb,             

    output reg [1:0] ForwardA,         
    output reg [1:0] ForwardB         
);

always @(*) begin
    ForwardA = 2'b00;
    ForwardB = 2'b00;

    if (RegWrite_mem && (rd_mem != 0) && (rd_mem == rs1_ex)) begin
        ForwardA = 2'b10; 
    end else if (RegWrite_wb && (rd_wb != 0) &&
                 !(RegWrite_mem && (rd_mem != 0) && (rd_mem != rs1_ex)) &&
                 (rd_wb == rs1_ex)) begin
        ForwardA = 2'b01; 
    end

    if (RegWrite_mem && (rd_mem != 0) && (rd_mem == rs2_ex)) begin
        ForwardB = 2'b10; 
    end else if (RegWrite_wb && (rd_wb != 0) &&
                 !(RegWrite_mem && (rd_mem != 0) && (rd_mem != rs2_ex)) &&
                 (rd_wb == rs2_ex)) begin
        ForwardB = 2'b01;
    end
end

endmodule

module alu_control(
    input [31:0] instruction, 
    input [1:0] ALUOp,         
    output reg [3:0] control_output 
);

    wire [6:0] funct7;  
    wire [2:0] funct3;  

    assign funct7 = instruction[31:25];  
    assign funct3 = instruction[14:12];  

    always @(*) begin
        case (ALUOp)
            2'b00: control_output = 4'b0010; // Load (ld) & Store (sd) -> ADD
            2'b01: control_output = 4'b0110; // Branch (beq) -> SUBTRACT
            2'b10: // R-type instructions
                case ({funct7, funct3}) 
                    10'b0000000_000: control_output = 4'b0010; // ADD
                    10'b0100000_000: control_output = 4'b0110; // SUBTRACT
                    10'b0000000_111: control_output = 4'b0000; // AND
                    10'b0000000_110: control_output = 4'b0001; // OR
                    default: control_output = 4'b0000; 
                endcase
            default: control_output = 4'b0000; 
        endcase
    end

endmodule


module execute (
  input [63:0] imm_out_ex,
  input [63:0] read_data1_ex,
  inout [63:0] read_data2_ex,
  input [63:0] ALU_result_mem,
  input [63:0] Result_wb,
  input [4:0] rs1_ex, rs2_ex,
  input [4:0] rd_mem,
  input [4:0] rd_wb,
  input RegWrite_mem,
  input RegWrite_wb,
  input [31:0] inst_ex,
  input [1:0] ALUOp_ex,
  input ALUSrc_ex,
  output [63:0] ALU_result_ex,
  output [63:0] write_data_ex
);

  wire [1:0] ForwardA, ForwardB;
  wire [63:0] res2;
  wire [63:0] A1, B1;

  forwarding_unit f1(.rs1_ex(rs1_ex), .rs2_ex(rs2_ex), .rd_mem(rd_mem), .rd_wb(rd_wb), .RegWrite_mem(RegWrite_mem), .RegWrite_wb(RegWrite_wb), .ForwardA(ForwardA), .ForwardB(ForwardB));

  mux m1(.a(B1), .b(imm_out_ex), .sel(ALUSrc_ex), .out(res2));

  mux3 A(.A(read_data1_ex), .C(ALU_result_mem), .B(Result_wb), .sel(ForwardA), .D(A1));
  mux3 B(.A(read_data2_ex), .C(ALU_result_mem), .B(Result_wb), .sel(ForwardB), .D(B1));

  assign write_data_ex = B1;

  wire [3:0] control;

  alu_control ac1(.instruction(inst_ex), .ALUOp(ALUOp_ex), .control_output(control));
  alu         al1(.rs1(A1), .rs2(res2), .ALUcontrol(control), .out(ALU_result_ex), .zero(zero));

  
endmodule