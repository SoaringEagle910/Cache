//for chiplab simutation，我们没能将IP核实现的BRAM接入chiplab进行测试，simple dual port ram
`timescale 1ns / 1ps
module dBRAM #(//a端口写，b端口读
    parameter DATA_SIZE = 32,
    parameter ADDR_SIZE = 7
) (
    input logic clka,
    input logic clkb,
    input logic ena,
    input logic enb,

    input logic[3:0] wea, 
    input  logic [DATA_SIZE-1:0] dina,
    input  logic [ADDR_SIZE-1:0] addra,

    input  logic [ADDR_SIZE-1:0] addrb,
    output logic [DATA_SIZE-1:0] doutb
);

    (* ram_style = "block" *) logic [DATA_SIZE-1:0] data[2**ADDR_SIZE];

    wire [31:0]write_mask_a;
    assign write_mask_a={{8{wea[3]}},{8{wea[2]}},{8{wea[1]}},{8{wea[0]}}};

    // initialization
    initial begin
        for (integer i = 0; i < 2 ** ADDR_SIZE; i++) begin
            data[i] = 0;
        end
    end

    // Read
    always_ff @(posedge clkb) begin
        doutb <= data[addrb];
    end

    // Write
    always_ff @(posedge clka) begin
        if (ena & (|wea)) begin
            data[addra] <= (dina&write_mask_a)|(data[addra]&~write_mask_a);
        end
    end
    
endmodule