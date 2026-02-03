`timescale 1ns / 1ps

`define CPU_IDLE 1'b0
`define CPU_EXEC 1'b1


// =========================================================
// MIPS-Like 32-bit CPU 指令集定义 (33 个独立 Opcode)
// =========================================================

// 1. 算术运算 (6 条)
`define ADD     6'b000001   // 有符号加法 (R-Type: Rd = Rs + Rt)
`define SUB     6'b000010   // 有符号减法 (R-Type: Rd = Rs - Rt)
`define MUL     6'b000011   // 乘法 (R-Type: Rd = Rs * Rt)
`define DIV     6'b000100   // 除法 (R-Type: Rd = Rs / Rt)
`define ADDU    6'b011110   // (R-Type: Rd = Rs + Rt)
`define SUBU    6'b011111   //  (R-Type: Rd = Rs - Rt)

// 2. 立即数算术/内存/比较 (8 条)
`define SLTI    6'b000101   // Set Less Than Immediate (I-Type: Rd = Rs < Imm ? 1 : 0)
`define SLTIU   6'b000110   // Set Less Than Unsigned Immediate (I-Type: Rd = Rs < Imm ? 1 : 0)
`define ADDI    6'b000111   // 加立即数 (I-Type: Rd = Rs + Imm)
`define SUBI    6'b001000   // 减立即数 (I-Type: Rd = Rs - Imm)
`define LUI     6'b001001   // Load Upper Immediate (I-Type: Rd = {Imm, 16'b0})
`define LW      6'b001010   // Load Word (I-Type: Rd = Mem[Rs + Imm])
`define SW      6'b001011   // Store Word (I-Type: Mem[Rs + Imm] = Rt)
`define XORI    6'b001111   // 新增：XOR Immediate (I-Type: Rd = Rs ^ Imm)

// 3. 逻辑运算 (3 条)
`define AND     6'b001100   // 逻辑与 (R-Type: Rd = Rs & Rt)
`define OR      6'b001101   // 逻辑或 (R-Type: Rd = Rs | Rt)
`define XOR     6'b001110   // 逻辑异或 (R-Type: Rd = Rs ^ Rt)

// 4. 移位操作 - Shamt (3 条)
`define SLL     6'b010010   // 逻辑左移 (Shamt: Rd = Rt << Shamt)
`define SRL     6'b010011   // 逻辑右移 (Shamt: Rd = Rt >> Shamt)
`define SRA     6'b010100   // 算术右移 (Shamt: Rd = Rt >>> Shamt)

// 5. 移位操作 - Variable (3 条)
`define SLLV    6'b100000   // 新增：逻辑左移 (R-Type: Rd = Rt << Rs)
`define SRLV    6'b100001   // 新增：逻辑右移 (R-Type: Rd = Rt >> Rs)
`define SRAV    6'b100010   // 新增：算术右移 (R-Type: Rd = Rt >>> Rs)

// 6. 比较与设置 (2 条)
`define SLT     6'b010101   // Set Less Than (R-Type: Rd = Rs < Rt ? 1 : 0)
`define SLTU    6'b010110   // Set Less Than Unsigned (R-Type: Rd = Rs < Rt ? 1 : 0)

// 7. 分支与跳转 (6 条)
`define JMP     6'b010111   // Jump (J-Type: PC = Target)
`define JAL     6'b011000   // Jump and Link (J-Type: R31 = PC+4; PC = Target)
`define BEQ     6'b011010   // Branch if Equal (I-Type: if (Rs == Rt) PC += Imm)
`define BNE     6'b011011   // Branch if Not Equal (I-Type: if (Rs != Rt) PC += Imm)
`define BGTZ    6'b011100   // Branch if Greater Than Zero (I-Type: if (Rs > 0) PC += Imm)
`define BLTZ    6'b011101   // Branch if Less Than Zero (I-Type: if (Rs < 0) PC += Imm)

// 8. 系统与空操作 (2 条)
`define NOP     6'b111110   
`define HALT    6'b111111   


module pcpu(
    input clk,
    input enable,
    input reset,
    input start,
    input [31:0] instruction, 
    input [31:0] datain,      
    
    output reg [31:0] i_addr, 
    
    output [31:0] d_addr,      
    
    output reg wena,
    output [31:0] dataout,  
    output [31:0] result_f1, 
    output [31:0] result_f2, 
    output [31:0] result_f3  
    );

    reg [31:0] i_addr_ID;
    reg [31:0] i_addr_EX;
    reg [31:0] i_addr_MEM;
    
    reg cpu_state, next_cpu_state;
    
    reg [31:0] instr_ID, instr_EX, instr_MEM, instr_WB;
    reg [31:0] instr_WB_final;

    // 标志位（通常不用于 MIPS，但若设计保留则保持 1 位）
    reg z_flag_EX, n_flag_EX, c_flag;
    reg z_flag_MEM;
    reg n_flag_MEM;  
    reg [31:0] gpr_file[0:31];
    integer i;

    wire [31:0] reg_A_read_ID;
    wire [31:0] reg_B_read_ID;

    reg [31:0] alu_op_A, alu_op_B;

    // 写回控制 (保持 1 位)
    wire RegWrite_EX;
    reg  RegWrite_MEM;

    // 转发控制 (保持 2 位)
    reg [1:0] ForwardA, ForwardB;
    reg [1:0] Forward_Store; // 用于 SW 指令数据转发

    // EX/MEM 阶段
    reg [31:0] alu_out_EX;
    reg [31:0] mem_write_data_EX;  // STORE 要写入 DMEM 的数据

    
    reg [31:0] wb_data_MEM;   // 准备写回寄存器的数据
    reg [31:0] wb_data_final; // 延后一拍，用于额外转发

    // 其它信号
    wire is_branch_taken;
    reg [31:0] alu_result_comb; 
    reg  c_flag_comb; // 1 位保持不变
   // output [9:0] d_addr, // 原有定义
   
   // DMEM 写入数据输出：来自 EX/MEM 寄存器
   assign dataout = mem_write_data_EX;
              
wire instr_dummy_use = ^instruction;
// =========================================================
// 1. WB 阶段 - 寄存器写回逻辑 (已升级为 32位/6位 Opcode)
// =========================================================

// 1.1 写使能信号：RegWrite_WB (基于 instr_WB[31:26] - 6位 Opcode)
wire is_I_Type_Write = 
    (instr_WB[31:26] == `SLTI) || (instr_WB[31:26] == `SLTIU) ||
    (instr_WB[31:26] == `ADDI) || (instr_WB[31:26] == `SUBI) ||
    (instr_WB[31:26] == `LUI)  || (instr_WB[31:26] == `XORI) ||
    (instr_WB[31:26] == `LW);

// 辅助：判断是否为需要写回的 R-Type 指令 (Rd)
wire is_R_Type_Write = 
    (instr_WB[31:26] == `ADD)   || (instr_WB[31:26] == `SUB)   || 
    (instr_WB[31:26] == `MUL)   || (instr_WB[31:26] == `DIV)   ||
    (instr_WB[31:26] == `ADDU)  || (instr_WB[31:26] == `SUBU)  ||
    (instr_WB[31:26] == `AND)   || (instr_WB[31:26] == `OR)    || 
    (instr_WB[31:26] == `XOR)   || (instr_WB[31:26] == `SLL)   || 
    (instr_WB[31:26] == `SRL)   || (instr_WB[31:26] == `SRA)   || 
    (instr_WB[31:26] == `SLLV)  || (instr_WB[31:26] == `SRLV)  || 
    (instr_WB[31:26] == `SRAV)  || (instr_WB[31:26] == `SLT)   ||
    (instr_WB[31:26] == `SLTU);
    
// 最终的写使能信号
wire RegWrite_WB = is_R_Type_Write || is_I_Type_Write || 
                   (instr_WB[31:26] == `JAL); // JAL 写 R31

// 1.2 写回目标寄存器地址：WB_WriteRegAddr (5位)
wire [4:0] WB_WriteRegAddr;

// 目标地址逻辑：JAL (R31) > R-Type (Rd: 15:11) > I-Type (Rt: 20:16)
assign WB_WriteRegAddr = 
    (instr_WB[31:26] == `JAL) ? 5'd31 :          // JAL 总是写 R31
    is_R_Type_Write ? instr_WB[15:11] :          // R-Type 目标 Rd
    is_I_Type_Write ? instr_WB[20:16] :          // I-Type 目标 Rt
    5'h0; // 默认值 (不写回时使用)

// -----------------------------------------------------------
// 2. GPR 寄存器堆写回 - 时序逻辑 (保留状态机控制)
// -----------------------------------------------------------
always @(posedge clk or posedge reset)
begin
    if (reset) begin
        for (i = 0; i < 32; i = i + 1) 
            gpr_file[i] <= 32'h0000_0000;
        wb_data_final <= 32'h00000000; 
    end
    // 更改：保留您的状态机判断
    else if (cpu_state == `CPU_EXEC) begin 
        // 写回逻辑：
        // 1. 检查 RegWrite_WB 是否开启
        // 2. 检查目标地址是否是 R0 (5'h00)
        if (RegWrite_WB && WB_WriteRegAddr != 5'h00) begin
            gpr_file[WB_WriteRegAddr] <= wb_data_MEM; 
            wb_data_final <= wb_data_MEM;
        end
        // 确保 wb_data_final 仍然要更新，用于下一周期的转发
        else begin
            wb_data_final <= wb_data_MEM; 
        end
    end
end

  // ==========================================================
// 2. CPU 状态机 (升级 6位 Opcode 并保留 HALT)
// ==========================================================
always @(posedge clk) begin
    if (reset)
        cpu_state <= `CPU_IDLE;
    else
        cpu_state <= next_cpu_state;
end

wire is_halt; // 声明 is_halt
assign is_halt = (instr_WB[31:26] == `HALT); 

always @(*) begin
    // 注意：is_halt 在这里作为输入信号使用，但没有被声明或赋值。
    if (cpu_state == `CPU_IDLE)
        next_cpu_state = (enable && start) ? `CPU_EXEC : `CPU_IDLE;
    else // cpu_state == `CPU_EXEC
    // 逻辑：遇到 HALT 或 enable 关闭，则返回 IDLE
        next_cpu_state = ((!enable) || is_halt) ? `CPU_IDLE : `CPU_EXEC;
end

// ==========================================================
// 3. ID 阶段寄存器读 (32位升级/最终确定版)
//    - 遵循最通用逻辑：Rs1 地址统一为 [25:21] (Rs)。
//    - Rs2 地址 MUX 现已涵盖所有需要第二个寄存器的 R-Type 指令。
// ==========================================================

// 1. Rs1 地址 (Source A)
// 逻辑：所有指令的 Rs1 地址都从 MIPS Rs 字段 [25:21] 读取。
wire [4:0] Rs1_Addr_ID_temp = instr_ID[25:21];


// 2. Rs2 地址 MUX 逻辑 (Source B)
// 逻辑：确定哪些指令需要读取 Rt 寄存器值作为 Rs2 (ALU 输入 B)。
wire is_R_Type_ALU = 
    (instr_ID[31:26] == `ADD || instr_ID[31:26] == `SUB ||
     instr_ID[31:26] == `MUL || instr_ID[31:26] == `DIV ||
     instr_ID[31:26] == `ADDU || instr_ID[31:26] == `SUBU || // 算术
     
     instr_ID[31:26] == `AND || instr_ID[31:26] == `OR ||
     instr_ID[31:26] == `XOR || // 逻辑

     instr_ID[31:26] == `SLLV || instr_ID[31:26] == `SRLV ||
     instr_ID[31:26] == `SRAV || // 移位变量 (Rs2=Rt)

     instr_ID[31:26] == `SLT || instr_ID[31:26] == `SLTU|| // 比较设置
     
    // 双操作数分支 (需要 Rs2 (Rt) 值进行比较)
    (instr_ID[31:26] == `BEQ || instr_ID[31:26] == `BNE)
    );


// MUX：如果需要 Rs2 (R-Type ALU)，则地址为 Rt；否则为 R0。
wire [4:0] Rs2_Addr_ID_temp =
    (is_R_Type_ALU)
    ? instr_ID[20:16]     // R-Type 使用 Rt 作为 Rs2 地址
    : 5'h00;             // 所有 I-Type (ADDI, LOAD, BRANCH) 和 Shamt 移位都使用 R0


// 3. GPR 读逻辑 (组合逻辑 + R0 零值约束)
// ... （保持不变）
assign reg_A_read_ID = 
    (Rs1_Addr_ID_temp == 5'h00) ? 32'h00000000 : 
    gpr_file[Rs1_Addr_ID_temp];
assign reg_B_read_ID = 
    (Rs2_Addr_ID_temp == 5'h00) ? 32'h00000000 : 
    gpr_file[Rs2_Addr_ID_temp];

   // ==========================================================
    // 4. 旁路前的 STORE 写数据（ID 阶段） - 32位升级
    // ==========================================================

    
    // --- ID 阶段源寄存器地址 (5位) ---
    // Rs1 地址：始终使用 Rs 字段 [25:21]
    wire [4:0] Rs1_Addr_ID_HazardCheck = instr_ID[25:21];
    
    // Rs2 地址：始终使用 Rt 字段 [20:16]
    wire [4:0] Rs2_Addr_ID_HazardCheck = instr_ID[20:16];
    
    // STORE 写入数据的寄存器地址 (作为源操作数)
    // STORE 指令写入的数据来自于 Rt 字段 [20:16]
    wire [4:0] StoreData_Addr_ID_HazardCheck = instr_ID[20:16];
    
    
    // --- 当前 ID 指令是否实际用到这些寄存器 ---
    // 目标：判断 ID 阶段指令是否会读取对应的寄存器。
    // R-Type/I-Type 运算、内存、分支都需要 Rs1 (Rs)
    wire uses_Rs1_ID =
        // R-Type 算术/逻辑/比较/Variable Shift (Rs作为源操作数)
        (instr_ID[31:26] == `ADD || instr_ID[31:26] == `SUB || instr_ID[31:26] == `MUL ||
         instr_ID[31:26] == `DIV || instr_ID[31:26] == `ADDU || instr_ID[31:26] == `SUBU ||
         instr_ID[31:26] == `AND || instr_ID[31:26] == `OR || instr_ID[31:26] == `XOR ||
         instr_ID[31:26] == `SLT || instr_ID[31:26] == `SLTU ||
         instr_ID[31:26] == `SLLV || instr_ID[31:26] == `SRLV || instr_ID[31:26] == `SRAV) ||
        
        // I-Type 算术/逻辑/比较/内存 (Rs作为源操作数/基地址)
        (instr_ID[31:26] == `SLTI || instr_ID[31:26] == `SLTIU || instr_ID[31:26] == `ADDI ||
         instr_ID[31:26] == `SUBI || instr_ID[31:26] == `XORI || instr_ID[31:26] == `LW ||
         instr_ID[31:26] == `SW) ||
         
        // 跳转/分支 (Rs作为跳转地址/比较操作数)
        (instr_ID[31:26] == `BEQ || instr_ID[31:26] == `BNE ||
         instr_ID[31:26] == `BGTZ || instr_ID[31:26] == `BLTZ);
        // 必须列出所有使用 Rs 字段作为源操作数的指令 Opcode。此处仅列出部分。
 wire is_Rs2_needed = 
            // R-Type ALU/Shift (需要 Rs2 作为 ALU 输入)
            (instr_ID[31:26] == `ADD || instr_ID[31:26] == `SUB || instr_ID[31:26] == `MUL ||
             instr_ID[31:26] == `DIV || instr_ID[31:26] == `ADDU || instr_ID[31:26] == `SUBU || 
             instr_ID[31:26] == `AND || instr_ID[31:26] == `OR || instr_ID[31:26] == `XOR ||
             instr_ID[31:26] == `SLLV || instr_ID[31:26] == `SRLV || instr_ID[31:26] == `SRAV || 
             instr_ID[31:26] == `SLT || instr_ID[31:26] == `SLTU) ||
             
            // 双操作数分支 (需要 Rs2 (Rt) 值进行比较)
            (instr_ID[31:26] == `BEQ || instr_ID[31:26] == `BNE);  
    // R-Type 和双操作数分支需要 Rs2 (Rt)
    
    wire uses_Rs2_ID = is_Rs2_needed; // 复用我们上一轮定义的 is_Rs2_needed 信号。
    
    // STORE 指令需要 StoreData (Rt)
    wire uses_StoreData_ID = (instr_ID[31:26] == `SW); 
    // 核心改动：使用 6位 Opcode `SW`
    
    
    wire [4:0] load_dest_EX;
    assign load_dest_EX = instr_EX[20:16]; // 核心改动：Rt 字段 [20:16]
    
    // 典型的 load-use stall 条件：EX 是 LOAD，ID 使用了它的结果
    wire load_use_stall;
    assign load_use_stall =
        (instr_EX[31:26] == `LW) && (load_dest_EX != 5'h00) && // 核心改动：6位 Opcode `LW`
        (
            // 1. ID 指令使用了 Rs1
            (uses_Rs1_ID       && (load_dest_EX == Rs1_Addr_ID_HazardCheck)) ||
            // 2. ID 指令使用了 Rs2
            (uses_Rs2_ID       && (load_dest_EX == Rs2_Addr_ID_HazardCheck)) ||
            // 3. ID 指令是 STORE，使用了 StoreData
            (uses_StoreData_ID && (load_dest_EX == StoreData_Addr_ID_HazardCheck))
        );
        

// ==========================================================
// 6. IF 阶段：i_addr & instr_ID 寄存器（32位升级/最终确定）
// ==========================================================
always @(posedge clk or posedge reset)
begin
    if (reset) begin
        instr_ID       <= 32'h00000000; 
        instr_WB_final <= 32'h00000000; 
        i_addr         <= 32'h00000000;
        i_addr_ID      <= 32'h00000000;
    end
    else if (cpu_state == `CPU_EXEC) begin
        
        // --- 1. i_addr (PC) 更新逻辑 ---
        if (load_use_stall) begin
            // 发现 LOAD-USE 冒险：冻结 i_addr
            i_addr <= i_addr;
        end
        else begin
            if (is_branch_taken)
                i_addr <= branch_target_MEM;
            else
                i_addr <= i_addr + 32'd1;
        end
        // --- 2. instr_ID (IF/ID 寄存器) 更新逻辑 ---
        if (is_branch_taken) begin
            instr_ID <= 32'h00000000; // 插入 NOP 气泡
            i_addr_ID <= 32'h0;
        end
        else if (load_use_stall) begin
            instr_ID <= 32'h00000000; // 插入 NOP 气泡
            i_addr_ID <= 32'h0;
        end
        else begin
            // 正常情况：锁存取到的指令
            instr_ID <= instruction;
            i_addr_ID <= i_addr+32'd1;
        end
    end
end
    // ==========================================================
    // 7. ID/EX 流水线寄存器（带 branch flush + load-use bubble）
    // ==========================================================
    // =========== EX 阶段 STORE 数据旁路 =============
    // EX 阶段需要使用 instr_EX[11:8] 作为 STORE 的数据寄存器号
    wire [31:0] store_data_forwarded_EX;
    assign store_data_forwarded_EX =
        (Forward_Store == 2'b01) ? alu_out_EX :
        (Forward_Store == 2'b10) ? wb_data_MEM :
        (Forward_Store == 2'b11) ? wb_data_final :
        mem_write_data_EX;
    // ==========================================================
    // 7. ID/EX 流水线寄存器（纯格式升级版）
    // ==========================================================
    // 假设：instr_EX, mem_write_data_EX, alu_op_A, alu_op_B 都已在顶层声明为 reg [31:0]
    always @(posedge clk or posedge reset)
    begin
        if (reset) begin
            instr_EX        <= 32'h00000000; 
            i_addr_EX  <= 32'd0;
            mem_write_data_EX <= 32'h00000000;
            alu_op_A        <= 32'h00000000;
            alu_op_B        <= 32'h00000000;
        end
        else if (cpu_state == `CPU_EXEC) begin
            if (is_branch_taken) begin
                instr_EX <= 32'h00000000; 
                i_addr_EX  <= 32'd0;
                alu_op_A <= 32'h00000000;
                alu_op_B <= 32'h00000000;
            end
            else if (load_use_stall) begin
                // 插入一个气泡到 EX
                instr_EX <= 32'h00000000; 
                i_addr_EX  <= 32'd0;
                alu_op_A <= 32'h00000000;
                alu_op_B <= 32'h00000000;
                // mem_write_data_EX 保持原逻辑（无清零）
            end
            else begin
                // 正常推进 (数据路径升级)
                instr_EX <= instr_ID; 
                i_addr_EX  <= i_addr_ID;
                alu_op_A <= reg_A_read_ID; 
                alu_op_B <= reg_B_read_ID; 
                // 只有"新进入 EX 的 STORE"才更新 mem_write_data_EX
                if (instr_ID[31:26] == `SW) 
                    mem_write_data_EX <= store_data_forwarded_EX; // 32位数据赋值
                // 否则 mem_write_data_EX 保持原逻辑（没有else）
            end
        end
    end

    // ==========================================================
    // 8. Forwarding Unit（旁路单元）
    // ==========================================================
   wire RegWrite_EX_MEM = 
        (instr_MEM[31:26] == `ADD   || instr_MEM[31:26] == `SUB   ||
         instr_MEM[31:26] == `MUL   || instr_MEM[31:26] == `DIV   ||
         instr_MEM[31:26] == `ADDU  || instr_MEM[31:26] == `SUBU  ||
         instr_MEM[31:26] == `SLTI  || instr_MEM[31:26] == `SLTIU ||
         instr_MEM[31:26] == `ADDI  || instr_MEM[31:26] == `SUBI  ||
         instr_MEM[31:26] == `LUI   || instr_MEM[31:26] == `LW   ||
         instr_MEM[31:26] == `XORI  || instr_MEM[31:26] == `AND   ||
         instr_MEM[31:26] == `OR    || instr_MEM[31:26] == `XOR   ||
         instr_MEM[31:26] == `SLL   || instr_MEM[31:26] == `SRL   ||
         instr_MEM[31:26] == `SRA   || instr_MEM[31:26] == `SLLV  ||
         instr_MEM[31:26] == `SRLV  || instr_MEM[31:26] == `SRAV  ||
         instr_MEM[31:26] == `SLT   || instr_MEM[31:26] == `SLTU);
    
    // MEM/WB 写回使能（保持原逻辑结构）
    wire RegWrite_MEM_WB = RegWrite_MEM;
    // EX 阶段 Rs1 的地址
    wire [4:0] Rs1_Addr_EX;
    assign Rs1_Addr_EX = instr_EX[25:21]; 
// ==========================================================
// Forwarding Unit (修复版：对 MEM/WB 和 WB_final 加上类型检查)
// ==========================================================
// 写回使能（MEM/WB 阶段）
wire RegWrite_WB_stage = 
    (instr_WB[31:26] == `ADD   || instr_WB[31:26] == `SUB   ||
     instr_WB[31:26] == `MUL   || instr_WB[31:26] == `DIV   ||
     instr_WB[31:26] == `ADDU  || instr_WB[31:26] == `SUBU  ||
     instr_WB[31:26] == `SLTI  || instr_WB[31:26] == `SLTIU ||
     instr_WB[31:26] == `ADDI  || instr_WB[31:26] == `SUBI  ||
     instr_WB[31:26] == `LUI   || instr_WB[31:26] == `LW   ||
     instr_WB[31:26] == `XORI  || instr_WB[31:26] == `AND   ||
     instr_WB[31:26] == `OR    || instr_WB[31:26] == `XOR   ||
     instr_WB[31:26] == `SLL   || instr_WB[31:26] == `SRL   ||
     instr_WB[31:26] == `SRA   || instr_WB[31:26] == `SLLV  ||
     instr_WB[31:26] == `SRLV  || instr_WB[31:26] == `SRAV  ||
     instr_WB[31:26] == `SLT   || instr_WB[31:26] == `SLTU);

// 写回使能（WB_final 阶段）
wire RegWrite_WB_final_stage = 
    (instr_WB_final[31:26] == `ADD   || instr_WB_final[31:26] == `SUB   ||
     instr_WB_final[31:26] == `MUL   || instr_WB_final[31:26] == `DIV   ||
     instr_WB_final[31:26] == `ADDU  || instr_WB_final[31:26] == `SUBU  ||
     instr_WB_final[31:26] == `SLTI  || instr_WB_final[31:26] == `SLTIU ||
     instr_WB_final[31:26] == `ADDI  || instr_WB_final[31:26] == `SUBI  ||
     instr_WB_final[31:26] == `LUI   || instr_WB_final[31:26] == `LW   ||
     instr_WB_final[31:26] == `XORI  || instr_WB_final[31:26] == `AND   ||
     instr_WB_final[31:26] == `OR    || instr_WB_final[31:26] == `XOR   ||
     instr_WB_final[31:26] == `SLL   || instr_WB_final[31:26] == `SRL   ||
     instr_WB_final[31:26] == `SRA   || instr_WB_final[31:26] == `SLLV  ||
     instr_WB_final[31:26] == `SRLV  || instr_WB_final[31:26] == `SRAV  ||
     instr_WB_final[31:26] == `SLT   || instr_WB_final[31:26] == `SLTU);
    // =======================================================
     // 【修正区域 1】将 wire 声明和 assign 移到 always 块外部
     // =======================================================;
     
     
`define R_TYPE_WRITE_RD_OPCODES \
    (`ADD) || (`SUB) || (`MUL) || (`DIV) || \
    (`ADDU) || (`SUBU) || \
    (`AND) || (`OR) || (`XOR) || \
    (`SLL) || (`SRL) || (`SRA) || \
    (`SLLV) || (`SRLV) || (`SRAV) || \
    (`SLT) || (`SLTU)
    
    
// 在模块的顶层（不在任何 always 块内）使用 assign 语句进行连续赋值
// --- 阶段 1: 判断指令是否为写入 Rd (Instr[15:11]) 的 R-Type 指令 ---
// MEM 阶段：判断 instr_MEM 是否为写入 Rd 的指令
wire is_Write_Rd_MEM = 
    // 1. 算术运算 (ADD, SUB, MUL, DIV, ADDU, SUBU)
    (instr_MEM[31:26] == `ADD)  || (instr_MEM[31:26] == `SUB)  || 
    (instr_MEM[31:26] == `MUL)  || (instr_MEM[31:26] == `DIV)  ||
    (instr_MEM[31:26] == `ADDU) || (instr_MEM[31:26] == `SUBU) ||
    
    // 2. 逻辑运算 (AND, OR, XOR)
    (instr_MEM[31:26] == `AND)  || (instr_MEM[31:26] == `OR)   || (instr_MEM[31:26] == `XOR) ||
    
    // 3. 移位操作 - Shamt (SLL, SRL, SRA)
    (instr_MEM[31:26] == `SLL)  || (instr_MEM[31:26] == `SRL)  || (instr_MEM[31:26] == `SRA) ||
    
    // 4. 移位操作 - Variable (SLLV, SRLV, SRAV)
    (instr_MEM[31:26] == `SLLV) || (instr_MEM[31:26] == `SRLV) || (instr_MEM[31:26] == `SRAV) ||
    
    // 5. 比较与设置 (SLT, SLTU)
    (instr_MEM[31:26] == `SLT)  || (instr_MEM[31:26] == `SLTU);


// WB 阶段：判断 instr_WB 是否为写入 Rd 的指令
wire is_Write_Rd_WB = 
    (instr_WB[31:26] == `ADD)  || (instr_WB[31:26] == `SUB)  || 
    (instr_WB[31:26] == `MUL)  || (instr_WB[31:26] == `DIV)  ||
    (instr_WB[31:26] == `ADDU) || (instr_WB[31:26] == `SUBU) ||
    (instr_WB[31:26] == `AND)  || (instr_WB[31:26] == `OR)   || (instr_WB[31:26] == `XOR) ||
    (instr_WB[31:26] == `SLL)  || (instr_WB[31:26] == `SRL)  || (instr_WB[31:26] == `SRA) ||
    (instr_WB[31:26] == `SLLV) || (instr_WB[31:26] == `SRLV) || (instr_WB[31:26] == `SRAV) ||
    (instr_WB[31:26] == `SLT)  || (instr_WB[31:26] == `SLTU);


// WB_final 阶段：判断 instr_WB_final 是否为写入 Rd 的指令 (假设您有这个 5 级外的 WB 级)
wire is_Write_Rd_WB_final = 
    (instr_WB_final[31:26] == `ADD)  || (instr_WB_final[31:26] == `SUB)  || 
    (instr_WB_final[31:26] == `MUL)  || (instr_WB_final[31:26] == `DIV)  ||
    (instr_WB_final[31:26] == `ADDU) || (instr_WB_final[31:26] == `SUBU) ||
    (instr_WB_final[31:26] == `AND)  || (instr_WB_final[31:26] == `OR)   || (instr_WB_final[31:26] == `XOR) ||
    (instr_WB_final[31:26] == `SLL)  || (instr_WB_final[31:26] == `SRL)  || (instr_WB_final[31:26] == `SRA) ||
    (instr_WB_final[31:26] == `SLLV) || (instr_WB_final[31:26] == `SRLV) || (instr_WB_final[31:26] == `SRAV) ||
    (instr_WB_final[31:26] == `SLT)  || (instr_WB_final[31:26] == `SLTU);


// --- 阶段 2: 定义 RegWrite 地址 MUX 信号 ---

wire [4:0] RegWrite_Rd_Addr_MEM;  
wire [4:0] RegWrite_Rd_Addr_WB;  
wire [4:0] RegWrite_Rd_Addr_WB_final;
wire [4:0] Rs2_Addr_EX = instr_EX[20:16];

// MUX 逻辑：如果指令是写入 Rd 的 R-Type (is_Write_Rd_...), 则使用 [15:11]；否则使用 [20:16] (Rt)
assign RegWrite_Rd_Addr_MEM = is_Write_Rd_MEM ? instr_MEM[15:11] : instr_MEM[20:16];
assign RegWrite_Rd_Addr_WB  = is_Write_Rd_WB  ? instr_WB[15:11]  : instr_WB[20:16];
assign RegWrite_Rd_Addr_WB_final = is_Write_Rd_WB_final ? instr_WB_final[15:11] : instr_WB_final[20:16];

// =========================================================================

always @(*) begin
    ForwardA    = 2'b00;
    ForwardB    = 2'b00;
    Forward_Store = 2'b00;
    
    if (RegWrite_EX_MEM && RegWrite_Rd_Addr_MEM != 5'h0 &&
        RegWrite_Rd_Addr_MEM == Rs1_Addr_EX)
        ForwardA = 2'b01; 
    else if (RegWrite_WB_stage && RegWrite_Rd_Addr_WB != 5'h0 &&
            RegWrite_Rd_Addr_WB == Rs1_Addr_EX)
        ForwardA = 2'b10; 
    else if (RegWrite_WB_final_stage && RegWrite_Rd_Addr_WB_final != 5'h0 &&
            RegWrite_Rd_Addr_WB_final == Rs1_Addr_EX)
        ForwardA = 2'b11; 
    
    if (RegWrite_EX_MEM && RegWrite_Rd_Addr_MEM != 5'h0 &&
        RegWrite_Rd_Addr_MEM == Rs2_Addr_EX)
        ForwardB = 2'b01;
    else if (RegWrite_WB_stage && RegWrite_Rd_Addr_WB != 5'h0 &&
            RegWrite_Rd_Addr_WB == Rs2_Addr_EX)
        ForwardB = 2'b10;
    else if (RegWrite_WB_final_stage && RegWrite_Rd_Addr_WB_final != 5'h0 &&
            RegWrite_Rd_Addr_WB_final == Rs2_Addr_EX)
        ForwardB = 2'b11;

end
// ==========================================================
// 9. EX 阶段 ALU 输入 MUX (位宽升级)
// ==========================================================

wire [31:0] alu_op_A_forwarded =
(ForwardA == 2'b01) ? alu_out_EX  :     
(ForwardA == 2'b10) ? wb_data_MEM :     
(ForwardA == 2'b11) ? wb_data_final :   
                        alu_op_A;        

wire [31:0] alu_op_B_forwarded =
(ForwardB == 2'b01) ? alu_out_EX  :    
(ForwardB == 2'b10) ? wb_data_MEM :   
(ForwardB == 2'b11) ? wb_data_final :   
                        alu_op_B;      

reg [31:0] final_alu_input_B;
                        
always @(*) begin
    // R-Type 指令 (Rs2/Rt 作为 B 输入)
    case (instr_EX[31:26])
    `ADD, `SUB, `MUL, `DIV, `ADDU, `SUBU, `AND, `OR, `XOR, `SLT, `SLTU,
    `SLLV, `SRLV, `SRAV: 
        final_alu_input_B = alu_op_B_forwarded; // 32位旁路数据
    
    // I-Type (加/减立即数)
    `ADDI, `SUBI, `SLTI, `SLTIU, `XORI:
        final_alu_input_B = {{16{instr_EX[15]}}, instr_EX[15:0]};
        
    // I-Type (内存操作：基址 + 偏移)
    `LW, `SW:
        final_alu_input_B = {{16{instr_EX[15]}}, instr_EX[15:0]};
        
    // 移位指令 (Shamt 作为 B 输入)
    `SLL, `SRL, `SRA:
        final_alu_input_B = {{27{1'b0}}, instr_EX[10:6]};
        
    `BEQ, `BNE:
    final_alu_input_B = alu_op_B_forwarded; // Rt

    `BGTZ, `BLTZ:
        final_alu_input_B = 32'h00000000;        // 0   
    // 【新增】LUI：特殊 I-Type，B 输入必须为 0
    `LUI:
        final_alu_input_B = 32'h00000000;
        
    default:
        final_alu_input_B = 32'h00000000; // 32位默认值
endcase
end

// ==========================================================
// 9. EX 阶段 ALU 输入 MUX - A 输入修正 (32位 & 字段映射)
// ==========================================================
reg [31:0] final_alu_input_A;

always @(*) begin
    case (instr_EX[31:26])
        `SLL, `SRL, `SRA, 
        `SLLV, `SRLV, `SRAV:
            final_alu_input_A = alu_op_B_forwarded; // Rt
        default:
            final_alu_input_A = alu_op_A_forwarded; // Rs
    endcase
end

// ==========================================================
// 10. EX 阶段 ALU 运算 - 32位修正
// ==========================================================
wire [31:0] branch_target_address_EX;

assign branch_target_address_EX =
    i_addr_EX + {{16{instr_EX[15]}}, instr_EX[15:0]};

wire [32:0] temp_result = final_alu_input_A - final_alu_input_B;


always @(*) begin
    c_flag_comb    = 1'b0;
    alu_result_comb = 32'h00000000;

// 【修正】控制字段：从 instr_EX[15:12] 修正到 instr_EX[31:26]
case (instr_EX[31:26])
    // 1. 比较 (Set Less Than)
    // 这些指令计算 Rs < Rt/Imm，结果是 0 或 1。ALU 执行减法比较。
    `SLT, `SLTI: 
        alu_result_comb = ($signed(final_alu_input_A) < $signed(final_alu_input_B)) ? 32'd1 : 32'd0;
    `SLTU, `SLTIU: 
        alu_result_comb = (final_alu_input_A < final_alu_input_B) ? 32'd1 : 32'd0;
    // 2. 地址计算 / 有符号加法
        `LW, `SW, // 仅保留地址计算
        `ADD, `ADDI, `ADDU: 
            {c_flag_comb, alu_result_comb} = final_alu_input_A + final_alu_input_B;
            
    // 3. 有符号减法 (用于设置标志位)
            `SUB, `SUBI, `SUBU, `BEQ, `BNE, `BGTZ, `BLTZ: // 【修正】分支移到 SUB 组
                {c_flag_comb, alu_result_comb} = final_alu_input_A - final_alu_input_B;
        
    // 4. 乘法和除法 (ALU执行乘法/除法)
    `MUL:
        // 乘法结果通常需要 64位，这里保留 32位低位。
        alu_result_comb = final_alu_input_A * final_alu_input_B; 
    `DIV:
        // 零除保护逻辑可能在 ALU 外部，这里只执行操作。
        alu_result_comb = final_alu_input_A / final_alu_input_B;

    // 5. 逻辑运算 (AND, OR, XOR)
    `AND:  alu_result_comb = final_alu_input_A & final_alu_input_B;
    `OR:   alu_result_comb = final_alu_input_A | final_alu_input_B;
    `XOR, `XORI: alu_result_comb = final_alu_input_A ^ final_alu_input_B;

    // 6. 移位操作 (Shamt/Rs 都是 B 输入)
    
    // 逻辑右移
    `SRL, `SRLV: alu_result_comb = final_alu_input_A >> final_alu_input_B[4:0];
    // 算术右移 (保留符号位)
    `SRA, `SRAV: alu_result_comb = $signed(final_alu_input_A) >>> final_alu_input_B[4:0];
    // 逻辑左移
    `SLL, `SLLV: alu_result_comb = final_alu_input_A << final_alu_input_B[4:0];

    // 7. 特殊操作
    `LUI:
        // Load Upper Immediate (LUI) 操作：Rd = {Imm[15:0], 16'b0}
        // A 输入用于寄存器Rs，B 输入 MUX 设为 0。LUI 需要特殊处理。
        // 假设 ALU 外部 MUX 会将 Imm[15:0] 拼接/移位到高位。
        // 如果 ALU 内部处理，则需要：
        alu_result_comb = {instr_EX[15:0], 16'h0000};
        
    // 8. 默认 (JMP/JAL/NOP/HALT)
    // 这些指令通常不需要 ALU 计算结果，或者结果由 PC 更新。
    default: alu_result_comb = 32'h00000000;
endcase
end

// ==========================================================
// 11. EX/MEM 流水线寄存器 - 32位修正
// ==========================================================

reg [31:0] branch_addr_MEM; // 分支目标地址 (来自 EX 阶段组合逻辑)
// 这与 Forwarding Unit 中的 RegWrite_EX_MEM 逻辑相同。
assign RegWrite_EX =
(instr_EX[31:26] == `ADD   || instr_EX[31:26] == `SUB   ||
    instr_EX[31:26] == `MUL   || instr_EX[31:26] == `DIV   ||
    instr_EX[31:26] == `ADDU  || instr_EX[31:26] == `SUBU  ||
    instr_EX[31:26] == `SLTI  || instr_EX[31:26] == `SLTIU ||
    instr_EX[31:26] == `ADDI  || instr_EX[31:26] == `SUBI  ||
    instr_EX[31:26] == `LUI   || instr_EX[31:26] == `LW   ||
    instr_EX[31:26] == `XORI  || instr_EX[31:26] == `AND   ||
    instr_EX[31:26] == `OR    || instr_EX[31:26] == `XOR   ||
    instr_EX[31:26] == `SLL   || instr_EX[31:26] == `SRL   ||
    instr_EX[31:26] == `SRA   || instr_EX[31:26] == `SLLV  ||
    instr_EX[31:26] == `SRLV  || instr_EX[31:26] == `SRAV  ||
    instr_EX[31:26] == `SLT   || instr_EX[31:26] == `SLTU||
    instr_EX[31:26] == `BEQ   || instr_EX[31:26] == `BNE   ||
    instr_EX[31:26] == `BGTZ  || instr_EX[31:26] == `BLTZ);


always @(posedge clk or posedge reset)
begin
    if (reset) begin
        instr_MEM    <= {`NOP, 26'b0}; 
        i_addr_MEM <= 32'd0;
        alu_out_EX   <= 32'h00000000;
        wena         <= 1'b1; // 保持原逻辑
        z_flag_EX       <= 1'b0;
        n_flag_EX       <= 1'b0;
        c_flag       <= 1'b0;
        RegWrite_MEM <= 1'b0;
        branch_addr_MEM <= 32'd0;
        z_flag_MEM <= 1'b0;
        n_flag_MEM <= 1'b0;
    end
    else if (cpu_state == `CPU_EXEC) begin
        // --- 指令/RegWrite 传递逻辑保留 ---
        if (is_branch_taken) begin
            // 【修正】指令位宽和 NOP 填充
            instr_MEM    <= {`NOP, 26'b0};
            i_addr_MEM <= 32'd0;
            z_flag_MEM <= 1'b0;
            n_flag_MEM <= 1'b0;
            z_flag_EX       <= 1'b0;
            n_flag_EX       <= 1'b0;
        end
        else begin
            z_flag_MEM      <= z_flag_EX;
            n_flag_MEM      <= n_flag_EX;                   // 【新增】锁存 Z 标志
            branch_addr_MEM <= branch_target_address_EX; // 【新增】锁存目标地址
            instr_MEM    <= instr_EX;
            i_addr_MEM <= i_addr_EX;
            RegWrite_MEM <= RegWrite_EX;
        end
        // --- ALU 结果传递逻辑 ---
        // 【修正】ALU 结果位宽
        alu_out_EX <= alu_result_comb;

        // --- 标志位更新逻辑 ---
        if (RegWrite_EX) begin // 假设所有写回指令都需要更新标志位
            z_flag_EX <= (alu_result_comb == 32'h00000000); 
            n_flag_EX <= alu_result_comb[31]; 
            // 假设 CMP, ADD, SUB 类指令 Opcode 集合与您原逻辑匹配
            if (instr_EX[31:26] == `SLT   || instr_EX[31:26] == `SLTU ||
                instr_EX[31:26] == `SLTI  || instr_EX[31:26] == `SLTIU || // CMP 类
                instr_EX[31:26] == `ADD   || instr_EX[31:26] == `ADDI ||
                instr_EX[31:26] == `ADDU  || // ADD 类
                instr_EX[31:26] == `SUB   || instr_EX[31:26] == `SUBI ||
                instr_EX[31:26] == `SUBU) // SUB 类
                c_flag <= c_flag_comb;
        end
        
    end
end
// ==========================================================
// 12. MEM 阶段 & 分支判定 - 32位修正
// ==========================================================
wire is_BEQ_MEM = (instr_MEM[31:26] == `BEQ);
wire is_BNE_MEM = (instr_MEM[31:26] == `BNE);
// ... 其他分支指令判定 ...

assign d_addr = alu_out_EX; 

assign dataout = store_data_forwarded_EX; // 32位写内存数据


assign is_branch_taken =
    // 1. 条件分支
    ((instr_MEM[31:26] == `BEQ)  && (z_flag_EX  == 1'b1)) ||
    ((instr_MEM[31:26] == `BNE)  && (z_flag_EX  == 1'b0)) ||
    ((instr_MEM[31:26] == `BGTZ) && (n_flag_EX == 1'b0 && z_flag_EX  == 1'b0)) ||
    ((instr_MEM[31:26] == `BLTZ) && (n_flag_EX == 1'b1)) ||

    // 2. 无条件跳转
    (instr_MEM[31:26] == `JMP)  ||
    (instr_MEM[31:26] == `JAL);
 // ==========================================================
// 12.x MEM 阶段：统一跳转目标地址生成（组合逻辑）
// ==========================================================

// MEM 阶段统一的跳转目标地址
wire [31:0] branch_target_MEM;

assign branch_target_MEM =
    ((instr_MEM[31:26] == `JMP) || (instr_MEM[31:26] == `JAL)) ?
        { 6'b0, instr_MEM[25:0] } :
        branch_addr_MEM; 

always @(posedge clk or posedge reset)
begin
    if (reset) begin
        // 【修正】指令位宽和 NOP 填充
        instr_WB   <= {`NOP, 26'b0};
        // 【修正】数据位宽
        wb_data_MEM <= 32'h00000000;
    end
    else if (cpu_state == `CPU_EXEC) begin
        instr_WB <= instr_MEM;
    
        // 使用 32位 LOAD 指令 `LW
        if (instr_MEM[31:26] == `LW)
            wb_data_MEM <= datain;
        else
            if (instr_MEM[31:26] == `JAL)
                wb_data_MEM <= i_addr_MEM + 32'd1;
            else
                wb_data_MEM <= alu_out_EX;
    end
end
     
// ==========================================================
// 14. 调试输出
// ==========================================================
    assign result_f1 = gpr_file[5];//i
    assign result_f2 = gpr_file[1];//c
    assign result_f3 = gpr_file[2];//d
endmodule
