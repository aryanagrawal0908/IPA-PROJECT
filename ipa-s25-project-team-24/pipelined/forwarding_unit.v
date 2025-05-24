module forwarding_unit (
    input [4:0] ID_EX_rs1, ID_EX_rs2,  // Source registers from ID/EX stage
    input [4:0] EX_MEM_rd,             // Destination register from EX/MEM stage
    input [4:0] MEM_WB_rd,             // Destination register from MEM/WB stage
    input EX_MEM_RegWrite,             // RegWrite signal from EX/MEM stage
    input MEM_WB_RegWrite,             // RegWrite signal from MEM/WB stage

    output reg [1:0] ForwardA,         // Forwarding control for ALU operand A
    output reg [1:0] ForwardB          // Forwarding control for ALU operand B
);

always @(*) begin
    // Default values: No forwarding (take from ID/EX register file)
    ForwardA = 2'b00;
    ForwardB = 2'b00;

    // Forwarding for rs1 (ALU operand A)
    if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs1)) begin
        ForwardA = 2'b10; // Forward from EX/MEM (ALU result of prior instruction)
    end else if (MEM_WB_RegWrite && (MEM_WB_rd != 0) &&
                 !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd != ID_EX_rs1)) &&
                 (MEM_WB_rd == ID_EX_rs1)) begin
        ForwardA = 2'b01; // Forward from MEM/WB (Data memory or earlier ALU result)
    end

    // Forwarding for rs2 (ALU operand B)
    if (EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd == ID_EX_rs2)) begin
        ForwardB = 2'b10; // Forward from EX/MEM (ALU result of prior instruction)
    end else if (MEM_WB_RegWrite && (MEM_WB_rd != 0) &&
                 !(EX_MEM_RegWrite && (EX_MEM_rd != 0) && (EX_MEM_rd != ID_EX_rs2)) &&
                 (MEM_WB_rd == ID_EX_rs2)) begin
        ForwardB = 2'b01; // Forward from MEM/WB (Data memory or earlier ALU result)
    end
end

endmodule
