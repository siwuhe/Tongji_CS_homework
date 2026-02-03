module imem(
    input [7:0] addr,         // 8位地址，对应最大256条指令
    output [15:0] instr       // 16位指令输出
    );

    // 使用 Xilinx IP 核 dist_mem_gen
    dist_mem_gen_0 instr_mem(
        .a(addr),             // 地址输入
        .spo(instr)           // 数据输出
    );

endmodule
