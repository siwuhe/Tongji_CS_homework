`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Testbench for top module
//////////////////////////////////////////////////////////////////////////////////

module top_tb;

    // ======================
    // 输入
    // ======================
    reg clk_in;
    reg enable;
    reg reset;
    reg start;
    reg sw_int;

    // ======================
    // 输出（调试）
    // ======================
    wire [7:0]  o_seg;
    wire [7:0]  o_sel;

    wire [31:0] dbg_i_addr;
    wire [31:0] dbg_instruction;
    wire [31:0] dbg_r1;
    wire [31:0] dbg_r2;
    wire [31:0] dbg_r3;
    wire [31:0] dbg_d_addr;
    wire        dbg_wena;
    wire [31:0] dbg_dataout;

    // ======================
    // 实例化 DUT
    // ======================
    top uut (
        .clk_in(clk_in),
        .enable(enable),
        .reset(reset),
        .start(start),
        .sw_int(sw_int),

        .o_seg(o_seg),
        .o_sel(o_sel),

        .dbg_i_addr(dbg_i_addr),
        .dbg_instruction(dbg_instruction),
        .dbg_r1(dbg_r1),
        .dbg_r2(dbg_r2),
        .dbg_r3(dbg_r3),
        .dbg_d_addr(dbg_d_addr),
        .dbg_wena(dbg_wena),
        .dbg_dataout(dbg_dataout)
    );

    // ======================
    // 时钟
    // ======================
    localparam CLK_PERIOD = 100;

    initial begin
        clk_in = 0;
        forever #(CLK_PERIOD/2) clk_in = ~clk_in;
    end

    // ======================
    // 激励
    // ======================
    initial begin
        enable = 0;
        reset  = 0;
        start  = 0;
        sw_int = 0;

        @(posedge clk_in);

        // 复位
        reset  = 1;
        enable = 1;
        repeat(5) @(posedge clk_in);
        reset = 0;

        $display("[%0t] Reset done", $time);

        // 启动
        start = 1;
        @(posedge clk_in);
        start = 0;

        // 运行
        repeat(30000) @(posedge clk_in);

        // 暂停
        sw_int = 1;
        repeat(5000) @(posedge clk_in);

        // 恢复
        sw_int = 0;
        repeat(30000) @(posedge clk_in);

        $display("[%0t] Simulation finished", $time);
        $finish;
    end

    // ======================
    // 波形
    // ======================
    initial begin
        $dumpfile("cpu.vcd");
        $dumpvars(0, top_tb);
    end

endmodule
