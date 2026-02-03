`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Top Module for Nexys4DDR
// 32-bit Dynamic Pipeline CPU + IMEM + DMEM + seg7x16
//////////////////////////////////////////////////////////////////////////////////

module top(
    input  clk_in,     // 板载系统时钟
    input  enable,     // BTNU - CPU 总使能（需保持）
    input  reset,      // BTNC - 复位
    input  start,      // BTND - 启动 CPU
    input  sw_int,     // 开关 - 暂停 CPU

    output [7:0] o_seg,
    output [7:0] o_sel
);

    // ======================================================
    //              CPU <-> IMEM / DMEM 信号
    // ======================================================
    wire [31:0] instruction;
    wire [31:0] r1;
    wire [31:0] datain;
    wire [31:0] dataout;
    wire [9:0]  i_addr;
    wire [9:0]  d_addr;
    wire        wena;

    // ======================================================
    //                  CLOCK DIVIDER
    // ======================================================
    reg [24:0] cnt = 0;

    always @(posedge clk_in or posedge reset)
        if (reset)
            cnt <= 0;
        else
            cnt <= cnt + 1'b1;

    // CPU 时钟（约 6.25 MHz）
    wire clk_cpu_raw = cnt[20];

    // sw_int = 1 → 暂停 CPU
    wire clk_cpu = clk_cpu_raw & (~sw_int);

    // ======================================================
    //                  7-SEG DISPLAY
    // 显示 PC（i_addr）
    // ======================================================
    wire [31:0] seg_data = {22'b0, i_addr};

    seg7x16 seg7_inst(
        .clk   (clk_in),
        .reset (reset),
        .cs    (1'b1),
        .i_data(r1),
        .o_seg (o_seg),
        .o_sel (o_sel)
    );

    // ======================================================
    //                      CPU
    // ======================================================
    pcpu cpu_inst(
        .clk        (clk_cpu),
        .enable     (enable),
        .reset      (reset),
        .start      (start),

        .instruction(instruction),
        .datain     (datain),

        .i_addr     (i_addr),
        .d_addr     (d_addr),
        .wena       (wena),
        .dataout    (dataout),

        // 调试端口不用，悬空即可
        .result_f1(),
        .result_f2(r1),
        .result_f3()
    );

    // ======================================================
    //                      IMEM
    // ======================================================
    imem imem_inst(
        .addr (i_addr),
        .instr(instruction)
    );

    // ======================================================
    //                      DMEM
    // ======================================================
    dmem dmem_inst(
        .clk     (clk_cpu),
        .ram_ena (enable),
        .wena    (wena),
        .addr    (d_addr),
        .data_in (dataout),
        .data_out(datain)
    );

endmodule
