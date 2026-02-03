`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////
// Testbench for the top module (CPU + Memories)
//////////////////////////////////////////////////////////////////////////////////

module top_tb;

    // 1. 定义顶层模块的输入信号 (必须声明为 reg)
    reg clk_in;
    reg enable;
    reg reset;
    reg start;

    // 2. 定义顶层模块的输出信号 (必须声明为 wire)
    wire [7:0] o_seg;
    wire [7:0] o_sel;
    wire [7:0] i_addr;
     wire [15:0] result_f1_tb;
     wire [15:0] result_f2_tb;
     wire [15:0] result_f3_tb;
     wire [15:0] instruction;
    // 时钟参数
    localparam CLK_PERIOD = 10; // 10ns 周期 (100MHz)

    // 3. 实例化顶层模块
    top uut (
        .clk_in(clk_in),
        .enable(enable),
        .reset(reset),
        .start(start),
        .o_seg(o_seg),
        .o_sel(o_sel),
        .i_addr(i_addr),
        .result_f1(result_f1_tb),
        .result_f2(result_f2_tb),
        .result_f3(result_f3_tb),
        .instruction(instruction)
    );

    // 4. 时钟生成逻辑
    initial begin
        clk_in = 0;
        // 循环产生时钟信号
        forever #(CLK_PERIOD / 2) clk_in = ~clk_in; 
    end

    // 5. 激励序列 (初始化、复位、启动)
    initial begin
        // 初始化所有输入信号
        enable = 1'b0;
        reset  = 1'b0;
        start  = 1'b0;

        // 等待时钟稳定
        @(posedge clk_in);

        // --- 1. 启动复位 ---
        // 确保所有寄存器和存储器初始化
        reset  = 1'b1;
        enable = 1'b1; // 允许 CPU 和内存工作

        // 保持复位 5 个周期
        repeat(5) @(posedge clk_in);

        // --- 2. 结束复位 ---
        reset = 1'b0;
        $display("System Reset complete. Starting CPU.");

        // --- 3. 给出启动脉冲 ---
        start = 1'b1;
        @(posedge clk_in);
        start = 1'b0; // 启动脉冲持续 1 个周期

        // --- 4. 运行仿真 ---
        // 运行足够长的时间，让 CPU 完成您的摔鸡蛋算法
        repeat(500000) @(posedge clk_in); 

        // --- 5. 仿真结束 ---
        $display("Simulation finished. Final results should be in registers R14 and R15.");
        $finish; 
    end

    // 可选：波形导出，便于在 Vivado 中查看内部信号
    initial begin
        $dumpfile("top.vcd");
        $dumpvars(0, top_tb); 
    end
    
endmodule