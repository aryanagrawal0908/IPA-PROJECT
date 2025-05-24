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
  input signed [63:0] read_data1,
  input signed [63:0] read_data2,
  input signed [63:0] imm_out,
  input [31:0] inst,
  input [1:0] ALUOp,
  input ALUSrc,
  input Branch,
  input [63:0] out,
  input [63:0] PC4,
  output [63:0] in,
  output signed [63:0] alu_result
);
  wire [63:0] rs2, as, ar;
  wire [3:0] control;
  wire zero, muxs;

  mux m1(.a(read_data2), .b(imm_out), .sel(ALUSrc), .out(rs2));
  alu_control ac1(.instruction(inst), .ALUOp(ALUOp), .control_output(control));
  alu a1(.rs1(read_data1), .rs2(rs2), .ALUcontrol(control), .out(alu_result), .zero(zero));

  SL1 l1(.A(imm_out), .C(as));
  adder ad1(.A(out), .B(as), .M(1'b0), .S(ar));
  and (muxs, Branch, zero);

  mux m2(.a(PC4), .b(ar), .sel(muxs), .out(in));
  
endmodule