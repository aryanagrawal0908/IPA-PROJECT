module data_memory (                 
    input MemRead,              
    input MemWrite,             
    input [63:0] address,       
    input [63:0] write_data,    
    output reg [63:0] read_data 
);

    reg [63:0] memory [0:1023]; 
    initial begin
        memory[0] = 64'b1;
        memory[1] = 64'd10;
    end
    always @(*) begin
        if (MemWrite) 
            memory[address[9:0]] <= write_data;  
    end

    always @(*) begin
        if (MemRead) 
            read_data = memory[address[9:0]];   
        else 
            read_data = 64'b0;                 
    end

endmodule

