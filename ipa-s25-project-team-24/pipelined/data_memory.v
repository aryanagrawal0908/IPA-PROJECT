module data_memory (                 
    input MemRead_mem,              
    input MemWrite_mem,     
    inout [63:0] address,       
    input [63:0] write_data,   
    output reg [63:0] read_data_mem 
);
    reg [63:0] memory [0:1023]; 
   
    always @(*) begin
        if (MemWrite_mem) 
            memory[address[9:0]] <= write_data;  
    end

    always @(*) begin
        if (MemRead_mem) 
            read_data_mem = memory[address[9:0]];   
        else 
            read_data_mem = 64'b0;                 
    end

endmodule

