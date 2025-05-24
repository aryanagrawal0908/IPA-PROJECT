// not in included in alu, but still included it here:

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

// cool proceed


module PFA (
  input a,
  input b,
  input c,
  output s,
  output p,
  output g
);

  xor(p, a, b);
  and(g, a, b);
  xor(s, p, c); 
endmodule

module CLA (
  input [3:0] P,
  input [3:0] G,
  input Cin,
  output [3:0] C
);

  wire t1;
  and(t1, P[0], Cin);
  or(C[0], G[0], t1);

  wire t2, t3;
  and(t2, P[1], P[0], Cin);
  and(t3, P[1], G[0]);
  or(C[1], G[1], t2, t3);

  wire t4, t5, t6;
  and(t4, P[2], P[1], P[0], Cin);
  and(t5, P[2], P[1], G[0]);
  and(t6, P[2], G[1]);
  or(C[2], G[2], t4, t5, t6);

  wire t7, t8, t9, t10;
  and(t7, P[3], P[2], P[1], P[0], Cin);
  and(t8, P[3], P[2], P[1], G[0]);
  and(t9, P[3], P[2], G[1]);
  and(t10, P[3], G[2]);
  or(C[3], G[3], t7, t8, t9, t10);
  
endmodule

module adder4 (
  input signed [3:0] A,
  input signed [3:0] B,
  input Cin,
  output signed [3:0] S,
  output Cout
);
  wire [3:0] P;
  wire [3:0] G;
  wire [3:0] C;

  PFA p1(.a(A[0]), .b(B[0]), .c(Cin), .s(S[0]), .p(P[0]), .g(G[0]));
  PFA p2(.a(A[1]), .b(B[1]), .c(C[0]), .s(S[1]), .p(P[1]), .g(G[1]));
  PFA p3(.a(A[2]), .b(B[2]), .c(C[1]), .s(S[2]), .p(P[2]), .g(G[2]));
  PFA p4(.a(A[3]), .b(B[3]), .c(C[2]), .s(S[3]), .p(P[3]), .g(G[3]));

  CLA c1(.P(P), .G(G), .Cin(Cin), .C(C));
  assign Cout = C[3]; 
endmodule


module adder16 (
  input signed [15:0] A,
  input signed [15:0] B, 
  input Cin,
  output signed [15:0] S,
  output Cout
);

  wire [3:0] C;

  adder4 a1(.A(A[3:0]), .B(B[3:0]), .Cin(Cin), .S(S[3:0]), .Cout(C[0]));
  adder4 a2(.A(A[7:4]), .B(B[7:4]), .Cin(C[0]), .S(S[7:4]), .Cout(C[1]));
  adder4 a3(.A(A[11:8]), .B(B[11:8]), .Cin(C[1]), .S(S[11:8]), .Cout(C[2]));
  adder4 a4(.A(A[15:12]), .B(B[15:12]), .Cin(C[2]), .S(S[15:12]), .Cout(C[3]));


  assign Cout = C[3]; 
endmodule

module adder (
  input signed [63:0] A,
  input signed [63:0] B,
  input M,
  output signed [63:0] S,
  output Cout
);
  
  wire [3:0] C;

  wire [63:0] BB;
  genvar i;
  for (i = 0; i < 64; i = i + 1) begin
    xor(BB[i], B[i], M);
  end

  adder16 a1(.A(A[15:0]), .B(BB[15:0]), .Cin(M), .S(S[15:0]), .Cout(C[0]));
  adder16 a2(.A(A[31:16]), .B(BB[31:16]), .Cin(C[0]), .S(S[31:16]), .Cout(C[1]));
  adder16 a3(.A(A[47:32]), .B(BB[47:32]), .Cin(C[1]), .S(S[47:32]), .Cout(C[2]));
  adder16 a4(.A(A[63:48]), .B(BB[63:48]), .Cin(C[2]), .S(S[63:48]), .Cout(C[3]));
  assign Cout = C[3];
endmodule

module ander (
  input [63:0] A,
  input [63:0] B,
  output [63:0] C
);
  genvar  i;
  generate
    for (i = 0; i < 64; i = i + 1) begin
      and(C[i], A[i], B[i]);
    end
  endgenerate

endmodule


module orer (
  input [63:0] A,
  input [63:0] B,
  output [63:0] C
);
  genvar  i;
  generate
    for (i = 0; i < 64; i = i + 1) begin
      or(C[i], A[i], B[i]);
    end
  endgenerate

endmodule

module SL1 (
  input [63:0] A,
  output [63:0] C
);

  assign C = {A[62:0], 1'b0};

endmodule

module alu (
  input [63:0] rs1,
  input [63:0] rs2,
  input [3:0] ALUcontrol,
  output reg [63:0] out,
  output reg zero 
);

  wire [63:0] add_result, or_result, and_result, sub_result;

  adder A1(.A(rs1), .B(rs2), .M(1'b0), .S(add_result), .Cout());
  adder A2(.A(rs1), .B(rs2), .M(1'b1), .S(sub_result), .Cout());
  orer O1(.A(rs1), .B(rs2), .C(or_result));
  ander D1(.A(rs1), .B(rs2), .C(and_result));

  always @(*) begin
    case (ALUcontrol)  // Using ALUcontrol directly
      4'b0010: out = add_result;  // ADD
      4'b0110: out = sub_result;  // SUBTRACT
      4'b0000: out = and_result;  // AND
      4'b0001: out = or_result;   // OR
    endcase

    zero = (sub_result == 64'b0) ? 1'b1 : 1'b0;
  end

endmodule



