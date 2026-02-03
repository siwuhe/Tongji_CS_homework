`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Pipelined CPU Core (5-stage) - Robust Version with Forwarding + Load-Use Stall
//////////////////////////////////////////////////////////////////////////////////

`define CPU_IDLE 1'b0
`define CPU_EXEC 1'b1

// OPCODES
`define NOP   4'b0000
`define ADD   4'b0001
`define CMP   4'b0010
`define LOAD  4'b0011
`define STORE 4'b0100
`define BZ    4'b0101
`define BN    4'b0110
`define HALT  4'b0111
`define ADDI  4'b1000
`define SHR   4'b1001
`define SHL   4'b1010
`define SHRI  4'b1011
`define SHLI  4'b1100
`define SUB   4'b1101
`define SUBI  4'b1110

module pcpu(
    input clk,
    input enable,
    input reset,
    input start,
    input [15:0] instruction,
    input [15:0] datain,
    output reg [7:0] i_addr,
    output [7:0] d_addr,
    output reg wena,
    output [15:0] dataout,
    output [15:0] result_f1,
    output [15:0] result_f2,
    output [15:0] result_f3
    );

    // 状态机
    reg cpu_state, next_cpu_state;

    // 流水线寄存器
    reg [15:0] instr_ID, instr_EX, instr_MEM, instr_WB;
    reg [15:0] instr_WB_final;

    // 标志位
    reg z_flag, n_flag, c_flag;

    // 通用寄存器堆
    reg [15:0] gpr_file[15:0];
    integer i;

    // ID 阶段寄存器读
    wire [15:0] reg_A_read_ID;
    wire [15:0] reg_B_read_ID;

    // EX 阶段操作数
    reg [15:0] alu_op_A, alu_op_B;
    reg [15:0] final_alu_input_A, final_alu_input_B;

    // 写回控制
    wire RegWrite_EX;
    reg  RegWrite_MEM;

    // 转发控制
    reg [1:0] ForwardA, ForwardB;
    reg [1:0] Forward_Store;

    // EX/MEM 阶段
    reg [15:0] alu_out_EX;
    reg [15:0] mem_write_data_EX;  // STORE 要写入 DMEM 的数据

    // MEM/WB 阶段
    reg [15:0] wb_data_MEM;   // 准备写回寄存器的数据
    reg [15:0] wb_data_final; // 延后一拍，用于额外转发

    // 其它信号
    wire is_branch_taken;
    reg  [15:0] alu_result_comb;
    reg  c_flag_comb;

    // ==========================================================
    // 1. WB 阶段寄存器写回
    // ==========================================================
    wire RegWrite_WB =
        (instr_WB[15:12] == `ADD  || instr_WB[15:12] == `ADDI ||
         instr_WB[15:12] == `LOAD || instr_WB[15:12] == `CMP  ||
         instr_WB[15:12] == `SHR  || instr_WB[15:12] == `SHL  ||
         instr_WB[15:12] == `SHRI || instr_WB[15:12] == `SHLI ||
         instr_WB[15:12] == `SUB  || instr_WB[15:12] == `SUBI);

    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            for (i = 0; i < 16; i = i + 1)
                gpr_file[i] <= 16'h0000;
            wb_data_final <= 16'h0000;
        end
        else if (cpu_state == `CPU_EXEC) begin
            if (RegWrite_WB && instr_WB[11:8] != 4'h0) begin
                gpr_file[instr_WB[11:8]] <= wb_data_MEM;
                wb_data_final <= wb_data_MEM;
            end
        end
    end

    // ==========================================================
    // 2. CPU 状态机
    // ==========================================================
    always @(posedge clk) begin
        if (reset)
            cpu_state <= `CPU_IDLE;
        else
            cpu_state <= next_cpu_state;
    end

    always @(*) begin
        if (cpu_state == `CPU_IDLE)
            next_cpu_state = (enable && start) ? `CPU_EXEC : `CPU_IDLE;
        else
            next_cpu_state = ((!enable) || (instr_WB[15:12] == `HALT)) ? `CPU_IDLE : `CPU_EXEC;
    end

    // ==========================================================
    // 3. ID 阶段寄存器读
    // ==========================================================
    wire [3:0] Rs1_Addr_ID_temp =
        (instr_ID[15:12] == `ADDI || instr_ID[15:12] == `SUBI)
        ? instr_ID[11:8]       // I-type: Rs = Rd
        : instr_ID[7:4];       // R-type: Rs1
    wire [3:0] Rs2_Addr_ID_temp =
            (instr_ID[15:12] == `ADD ||
             instr_ID[15:12] == `SUB ||
             instr_ID[15:12] == `CMP ||
             instr_ID[15:12] == `SHR ||
             instr_ID[15:12] == `SHL)
            ? instr_ID[3:0]     // R-type Rs2
            : 4'h0;             // I-type 不用 Rs2
    assign reg_A_read_ID = gpr_file[Rs1_Addr_ID_temp];
    assign reg_B_read_ID = gpr_file[Rs2_Addr_ID_temp];

    // ==========================================================
    // 4. 旁路前的 STORE 写数据（ID 阶段）
    // ==========================================================
    wire [15:0] store_data_forwarded =
        (Forward_Store == 2'b01) ? alu_out_EX :
        (Forward_Store == 2'b10) ? wb_data_MEM :
        (Forward_Store == 2'b11) ? wb_data_final :
        gpr_file[instr_ID[11:8]];  // STORE 的数据寄存器

    // ==========================================================
    // 5. Load-Use Hazard 检测（包括 LOAD 后紧跟 STORE 的情况）
    // ==========================================================
    // ID 阶段源寄存器地址
    wire [3:0] Rs1_Addr_ID;
    assign Rs1_Addr_ID =
        (instr_ID[15:12] == `ADDI || instr_ID[15:12] == `SUBI ||
         instr_ID[15:12] == `BZ   || instr_ID[15:12] == `BN)
        ? instr_ID[11:8] : instr_ID[7:4];

    wire [3:0] Rs2_Addr_ID;
    assign Rs2_Addr_ID = instr_ID[3:0]; // R-Type / CMP / SHIFT

    wire [3:0] StoreData_Addr_ID;
    assign StoreData_Addr_ID = instr_ID[11:8]; // STORE 写入数据的寄存器

    // 当前 ID 指令是否实际用到这些寄存器
    wire uses_Rs1_ID =
        (instr_ID[15:12] == `ADD  || instr_ID[15:12] == `SUB  ||
         instr_ID[15:12] == `CMP  || instr_ID[15:12] == `SHR  ||
         instr_ID[15:12] == `SHL  || instr_ID[15:12] == `LOAD ||
         instr_ID[15:12] == `STORE|| instr_ID[15:12] == `ADDI ||
         instr_ID[15:12] == `SUBI);

    wire uses_Rs2_ID =
        (instr_ID[15:12] == `ADD  || instr_ID[15:12] == `SUB  ||
         instr_ID[15:12] == `CMP  || instr_ID[15:12] == `SHR  ||
         instr_ID[15:12] == `SHL);

    wire uses_StoreData_ID =
        (instr_ID[15:12] == `STORE);

    // EX 阶段 LOAD 的目的寄存器
    wire [3:0] load_dest_EX;
    assign load_dest_EX = instr_EX[11:8];

    // 典型的 load-use stall 条件：EX 是 LOAD，ID 使用了它的结果
    wire load_use_stall;
    assign load_use_stall =
        (instr_EX[15:12] == `LOAD) && (load_dest_EX != 4'h0) &&
        (
            (uses_Rs1_ID       && (load_dest_EX == Rs1_Addr_ID)) ||
            (uses_Rs2_ID       && (load_dest_EX == Rs2_Addr_ID)) ||
            (uses_StoreData_ID && (load_dest_EX == StoreData_Addr_ID))
        );

    // ==========================================================
    // 6. IF 阶段：PC & 指令获取（带 load-use stall & 分支）
    // ==========================================================
    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            instr_ID       <= {`NOP, 12'b0};
            instr_WB_final <= {`NOP, 12'b0};
            i_addr         <= 8'h00;
        end
        else if (cpu_state == `CPU_EXEC) begin
            if (load_use_stall) begin
                // 发现 LOAD-USE 冒险：冻结 PC 和 IF/ID
                i_addr   <= i_addr;
                instr_ID <= instr_ID;
            end
            else begin
                if (is_branch_taken)
                    i_addr <= alu_out_EX[7:0];
                else
                    i_addr <= i_addr + 1;

                if (is_branch_taken)
                    instr_ID <= {`NOP, 12'b0};
                else if (load_use_stall)
                        instr_ID <= instr_ID;
                else
                        instr_ID <= instruction;
            end

            // 这个只是多一拍的历史记录，用于额外的转发
            instr_WB_final <= instr_WB;
        end
    end

    // ==========================================================
    // 7. ID/EX 流水线寄存器（带 branch flush + load-use bubble）
    // ==========================================================
    // =========== EX 阶段 STORE 数据旁路 =============
    // EX 阶段需要使用 instr_EX[11:8] 作为 STORE 的数据寄存器号
    wire [15:0] store_data_forwarded_EX;
    assign store_data_forwarded_EX =
        (Forward_Store == 2'b01) ? alu_out_EX :
        (Forward_Store == 2'b10) ? wb_data_MEM :
        (Forward_Store == 2'b11) ? wb_data_final :
        gpr_file[instr_EX[11:8]]; // EX 阶段的 Rs(store_data)
    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            instr_EX        <= {`NOP, 12'b0};
            mem_write_data_EX <= 16'h0000;
            alu_op_A        <= 16'h0000;
            alu_op_B        <= 16'h0000;
        end
        else if (cpu_state == `CPU_EXEC) begin
            if (is_branch_taken) begin
                // 分支被采用：清空 EX
                instr_EX <= {`NOP, 12'b0};
                alu_op_A <= 16'h0000;
                alu_op_B <= 16'h0000;
                // mem_write_data_EX 保持即可，无影响
            end
            else if (load_use_stall) begin
                // 插入一个气泡到 EX，LOAD 留在 EX -> MEM，ID 留在原位
                instr_EX <= {`NOP, 12'b0};
                alu_op_A <= 16'h0000;
                alu_op_B <= 16'h0000;
                // mem_write_data_EX 保持
            end
            else begin
                // 正常推进
                instr_EX <= instr_ID;
                alu_op_A <= reg_A_read_ID;
                alu_op_B <= reg_B_read_ID;

                // 只有"新进入 EX 的 STORE"才更新 mem_write_data_EX
                 if (instr_EX[15:12] == `STORE)
                    mem_write_data_EX <= store_data_forwarded_EX;
            end
        end
    end

    // ==========================================================
    // 8. Forwarding Unit（旁路单元）
    // ==========================================================
    // EX/MEM 写回使能
    wire RegWrite_EX_MEM =
        (instr_MEM[15:12] == `ADD  || instr_MEM[15:12] == `ADDI ||
         instr_MEM[15:12] == `LOAD || instr_MEM[15:12] == `CMP  ||
         instr_MEM[15:12] == `SHR  || instr_MEM[15:12] == `SHL  ||
         instr_MEM[15:12] == `SHRI || instr_MEM[15:12] == `SHLI ||
         instr_MEM[15:12] == `SUB  || instr_MEM[15:12] == `SUBI);

    // MEM/WB 写回使能（其实就是 RegWrite_MEM）
    wire RegWrite_MEM_WB = RegWrite_MEM;

    // EX 阶段 Rs1 的地址
    wire [3:0] Rs1_Addr_EX;
    assign Rs1_Addr_EX =
        (instr_EX[15:12] == `ADDI || instr_EX[15:12] == `SUBI ||
         instr_EX[15:12] == `BZ   || instr_EX[15:12] == `BN)
        ? instr_EX[11:8] : instr_EX[7:4];

    // ==========================================================
// Forwarding Unit (修复版：对 MEM/WB 和 WB_final 加上类型检查)
// ==========================================================
wire RegWrite_WB_stage =
    (instr_WB[15:12] == `ADD  || instr_WB[15:12] == `ADDI ||
     instr_WB[15:12] == `LOAD || instr_WB[15:12] == `CMP  ||
     instr_WB[15:12] == `SHR  || instr_WB[15:12] == `SHL  ||
     instr_WB[15:12] == `SHRI || instr_WB[15:12] == `SHLI ||
     instr_WB[15:12] == `SUB  || instr_WB[15:12] == `SUBI);

wire RegWrite_WB_final_stage =
    (instr_WB_final[15:12] == `ADD  || instr_WB_final[15:12] == `ADDI ||
     instr_WB_final[15:12] == `LOAD || instr_WB_final[15:12] == `CMP  ||
     instr_WB_final[15:12] == `SHR  || instr_WB_final[15:12] == `SHL  ||
     instr_WB_final[15:12] == `SHRI || instr_WB_final[15:12] == `SHLI ||
     instr_WB_final[15:12] == `SUB  || instr_WB_final[15:12] == `SUBI);

    always @(*) begin
        ForwardA      = 2'b00;
        ForwardB      = 2'b00;
        Forward_Store = 2'b00;
    
        // -----------------------
        // ForwardA：EX 阶段 Rs1
        // -----------------------
        if (RegWrite_EX_MEM && instr_MEM[11:8] != 4'h0 &&
            instr_MEM[11:8] == Rs1_Addr_EX)
            ForwardA = 2'b01; // 来自 EX/MEM
        else if (RegWrite_WB_stage && instr_WB[11:8] != 4'h0 &&
                 instr_WB[11:8] == Rs1_Addr_EX)
            ForwardA = 2'b10; // 来自 MEM/WB (wb_data_MEM)
        else if (RegWrite_WB_final_stage && instr_WB_final[11:8] != 4'h0 &&
                 instr_WB_final[11:8] == Rs1_Addr_EX)
            ForwardA = 2'b11; // 来自 "再早一拍"
    
        // -----------------------
        // ForwardB：EX 阶段 Rs2
        // -----------------------
        if (RegWrite_EX_MEM && instr_MEM[11:8] != 4'h0 &&
            instr_MEM[11:8] == instr_EX[3:0])
            ForwardB = 2'b01;
        else if (RegWrite_WB_stage && instr_WB[11:8] != 4'h0 &&
                 instr_WB[11:8] == instr_EX[3:0])
            ForwardB = 2'b10;
        else if (RegWrite_WB_final_stage && instr_WB_final[11:8] != 4'h0 &&
                 instr_WB_final[11:8] == instr_EX[3:0])
            ForwardB = 2'b11;
    
        // -----------------------
        // STORE 写数据前推：ID 阶段的 Rs(store_data)
        // -----------------------
        
        if (RegWrite_EX_MEM && instr_MEM[11:8] != 4'h0 &&
            instr_MEM[11:8] == instr_EX[11:8])
            Forward_Store = 2'b01;
        else if (RegWrite_WB_stage && instr_WB[11:8] != 4'h0 &&
                 instr_WB[11:8] == instr_EX[11:8])
            Forward_Store = 2'b10;
        else if (RegWrite_WB_final_stage && instr_WB_final[11:8] != 4'h0 &&
                 instr_WB_final[11:8] == instr_EX[11:8])
            Forward_Store = 2'b11;
        
    end

    // ==========================================================
    // 9. EX 阶段 ALU 输入 MUX
    // ==========================================================
    wire [15:0] alu_op_A_forwarded =
        (ForwardA == 2'b01) ? alu_out_EX  :
        (ForwardA == 2'b10) ? wb_data_MEM :
        (ForwardA == 2'b11) ? wb_data_final :
                              alu_op_A;

    wire [15:0] alu_op_B_forwarded =
        (ForwardB == 2'b01) ? alu_out_EX  :
        (ForwardB == 2'b10) ? wb_data_MEM :
        (ForwardB == 2'b11) ? wb_data_final :
                              alu_op_B;

    // B 输入：根据指令类型选择寄存器 / 立即数 / 偏移
    always @(*) begin
        case (instr_EX[15:12])
            `ADD, `SUB, `CMP, `SHR, `SHL:
                final_alu_input_B = alu_op_B_forwarded;
            `ADDI, `SUBI:
                final_alu_input_B = {{8{instr_EX[7]}}, instr_EX[7:0]};  // 8-bit 符号扩展
            `LOAD, `STORE:
                final_alu_input_B = {{12{1'b0}}, instr_EX[3:0]};        // 4-bit offset
            `SHRI, `SHLI:
                final_alu_input_B = {{12{1'b0}}, instr_EX[3:0]};        // shamt
            `BZ, `BN:
                final_alu_input_B = {{8{1'b0}}, instr_EX[7:0]};         // 8-bit 跳转目标
            default:
                final_alu_input_B = 16'h0000;
        endcase
    end

    // A 输入：分支用 R0（0），其余用旁路后的 Rs1
    always @(*) begin
        case (instr_EX[15:12])
            `BZ, `BN:
                final_alu_input_A = 16'h0000;
            default:
                final_alu_input_A = alu_op_A_forwarded;
        endcase
    end

    // ==========================================================
    // 10. EX 阶段 ALU 运算
    // ==========================================================
    always @(*) begin
        c_flag_comb    = 1'b0;
        alu_result_comb = 16'h0000;

        case (instr_EX[15:12])
            `CMP:  {c_flag_comb, alu_result_comb} = final_alu_input_A - final_alu_input_B;
            `LOAD,
            `STORE,
            `BZ,
            `BN,
            `ADD,
            `ADDI: {c_flag_comb, alu_result_comb} = final_alu_input_A + final_alu_input_B;
            `SUB,
            `SUBI: {c_flag_comb, alu_result_comb} = final_alu_input_A - final_alu_input_B;
            `SHR:  alu_result_comb = final_alu_input_A >> final_alu_input_B[3:0];
            `SHL:  alu_result_comb = final_alu_input_A << final_alu_input_B[3:0];
            `SHRI: alu_result_comb = final_alu_input_A >> final_alu_input_B[3:0];
            `SHLI: alu_result_comb = final_alu_input_A << final_alu_input_B[3:0];
            default: alu_result_comb = 16'h0000;
        endcase
    end

    // ==========================================================
    // 11. EX/MEM 流水线寄存器
    // ==========================================================
    assign RegWrite_EX =
        (instr_EX[15:12] == `ADD  || instr_EX[15:12] == `ADDI ||
         instr_EX[15:12] == `LOAD || instr_EX[15:12] == `CMP  ||
         instr_EX[15:12] == `SHR  || instr_EX[15:12] == `SHL  ||
         instr_EX[15:12] == `SHRI || instr_EX[15:12] == `SHLI ||
         instr_EX[15:12] == `SUB  || instr_EX[15:12] == `SUBI);
    // New register
         reg [15:0] alu_out_MEM;
    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            instr_MEM    <= {`NOP, 12'b0};
            alu_out_EX   <= 16'h0000;
            alu_out_MEM <= 16'h0000;  // <-- NEW
            wena         <= 1'b1;
            z_flag       <= 1'b0;
            n_flag       <= 1'b0;
            c_flag       <= 1'b0;
            RegWrite_MEM <= 1'b0;
        end
        else if (cpu_state == `CPU_EXEC) begin
            if (is_branch_taken) begin
                instr_MEM    <= {`NOP, 12'b0};
                RegWrite_MEM <= 1'b0;
            end
            else begin
                instr_MEM    <= instr_EX;
                RegWrite_MEM <= RegWrite_EX;
            end

            alu_out_EX <= alu_result_comb;
            alu_out_MEM <= alu_result_comb;

            // 更新标志位
            if (instr_EX[15:12] == `CMP  || instr_EX[15:12] == `ADD  ||
                instr_EX[15:12] == `SUB  || instr_EX[15:12] == `ADDI ||
                instr_EX[15:12] == `SUBI || instr_EX[15:12] == `SHR  ||
                instr_EX[15:12] == `SHL  || instr_EX[15:12] == `SHRI ||
                instr_EX[15:12] == `SHLI) begin
                z_flag <= (alu_result_comb == 16'h0000);
                n_flag <= alu_result_comb[15];
                if (instr_EX[15:12] == `ADD  || instr_EX[15:12] == `ADDI ||
                    instr_EX[15:12] == `SUB  || instr_EX[15:12] == `SUBI ||
                    instr_EX[15:12] == `CMP)
                    c_flag <= c_flag_comb;
            end
        end
    end

    // ==========================================================
    // 12. MEM 阶段 & 分支判定
    // ==========================================================
    assign d_addr = alu_out_EX[7:0];
    assign dataout = mem_write_data_EX;

    assign is_branch_taken =
        (instr_MEM[15:12] == `BZ) ? (z_flag == 1'b1) :
        (instr_MEM[15:12] == `BN) ? (n_flag == 1'b1) :
                                     1'b0;

    // ==========================================================
    // 13. MEM/WB 流水线寄存器
    // ==========================================================
    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            instr_WB   <= {`NOP, 12'b0};
            wb_data_MEM <= 16'h0000;
        end
        else if (cpu_state == `CPU_EXEC) begin
            instr_WB <= instr_MEM;
            if (instr_MEM[15:12] == `LOAD)
                wb_data_MEM <= datain;
            else
                wb_data_MEM <= alu_out_EX;
        end
    end

    // ==========================================================
    // 14. 调试输出
    // ==========================================================
    assign result_f1 = gpr_file[15];
    assign result_f2 = gpr_file[14];
    assign result_f3 = final_alu_input_B;
    
endmodule
