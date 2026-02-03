`timescale 1ns / 1ps

module dmem(
    input clk,     
    input ram_ena, 
    input wena,    
    input [9 : 0] addr,     // 更改：地址位宽从 7:0 (8位) 变为 9:0 (10位)
    input [31 : 0] data_in,  // 更改：数据位宽从 15:0 (16位) 变为 31:0 (32位)
    output [31 : 0] data_out // 更改：数据位宽从 15:0 (16位) 变为 31:0 (32位)
    );
    
    // 1024 x 32-bit 存储器 (2^10 = 1024)
    // 存储器容量从 256 个 16位字 升级到 1024 个 32位字
    reg [31:0] data [1023:0]; 
    
    // ---- 内存初始化 ----
    integer i;
    initial begin
        // 全部初始化为 0（避免出现 xxxx）
        // 循环范围从 256 升级到 1024
        for (i = 0; i < 1024; i = i + 1)
            data[i] = 32'h0000_0000; // 更改：初始化值也变为 32位

        // （可选）根据你的测试程序初始化一些数据：
        // data[10'h000] = 32'h0000_0080; // 示例：地址 0 存储 32位值 0x80
    end
    
    // 异步读取
    // 更改：读取数据为 32位，未启用时输出 32位高阻态
    assign data_out = ram_ena ? data[addr] : 32'hzzzz_zzzz;
    
    // 同步写入
    always @ (posedge clk)
    begin
        if (ram_ena && wena)
            data[addr] <= data_in;
    end
endmodule