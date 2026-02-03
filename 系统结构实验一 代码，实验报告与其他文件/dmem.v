`timescale 1ns / 1ps

module dmem(
    input clk,     
    input ram_ena, 
    input wena,    
    input [7 : 0] addr,    
    input [15 : 0] data_in,
    output [15 : 0] data_out
    );
    
    // 256 x 16-bit 存储器
    reg [15:0] data [255:0]; 
    
    // ---- 内存初始化 ----
    integer i;
    initial begin
        // 全部初始化为 0（避免出现 xxxx）
        for (i = 0; i < 256; i = i + 1)
            data[i] = 16'h0000;

        // （可选）根据你的测试程序初始化一些数据：
        // 比如 DMEM[0] 放 N，
        // data[8'h00] = 16'h0080;  
    end
    
    // 异步读取
    assign data_out = ram_ena ? data[addr] : 16'hzzzz;
    
    // 同步写入
    always @ (posedge clk)
    begin
        if (ram_ena && wena)
            data[addr] <= data_in;
    end
endmodule
