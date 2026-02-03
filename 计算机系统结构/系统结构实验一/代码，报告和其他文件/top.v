`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Top Module for Nexys4DDR
//////////////////////////////////////////////////////////////////////////////////

module top(
    input  clk_in,     // 系统时钟
    input  enable,     // BTNU   - CPU 总使能（必须保持按下）
    input  reset,      // BTNC   - 复位（按一下即可）
    input  start,      // BTND   - 启动 CPU（按一下）
    output [7:0] o_seg,
    output [7:0] o_sel
);

    // ======== CPU <-> IMEM / DMEM 信号 ========
    wire [15:0] instruction;
    wire [15:0] datain;
    wire [7:0]  i_addr;
    wire [7:0]  d_addr;
    wire wena;
    wire [15:0] dataout;

    // ======================================================
    //                  CLOCK DIVIDER
    // ======================================================
    reg [24:0] cnt = 0;

    always @(posedge clk_in or posedge reset)
        if (reset)
            cnt <= 0;
        else
            cnt <= cnt + 1'b1;

    // clk_cpu = clk_in / 8 ≈ 12.5MHz → 流水线足够快
    wire clk_cpu = cnt[24];

    // ======================================================
    //                  7-SEG DISPLAY
    // 显示 PC 高 8 位 + 指令低 8 位
    // ======================================================
    wire [15:0] seg_data ={8'b0000_0000, instruction};

    seg7x16 seg7_inst(
        .clk(clk_in),
        .reset(reset),
        .cs(1'b1),
        .i_data(seg_data),
        .o_seg(o_seg),
        .o_sel(o_sel)
    );

    // ======================================================
    //                      CPU
    // ======================================================
    pcpu cpu_inst(
        .clk(clk_cpu),
        .enable(enable),     // 现在由按键控制，不再写死
        .reset(reset),
        .start(start),
        .instruction(instruction),
        .datain(datain),
        .i_addr(i_addr),
        .d_addr(d_addr),
        .wena(wena),
        .dataout(dataout),
        .result_f1(), .result_f2(), .result_f3()
    );

    // ======================================================
    //                      IMEM
    // ======================================================
    imem imem_inst(
        .addr(i_addr),
        .instr(instruction)
    );

    // ======================================================
    //                      DMEM
    // ======================================================
    dmem dmem_inst(
        .clk(clk_cpu),
        .ram_ena(enable),   // 不再写死，外部 enable 控制
        .wena(wena),
        .addr(d_addr),
        .data_in(dataout),
        .data_out(datain)
    );

endmodule
