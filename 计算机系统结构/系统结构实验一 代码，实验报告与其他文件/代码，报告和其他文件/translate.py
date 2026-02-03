import sys
import re

# --- 最终确定的 15 条指令集定义 (OpCode 4-bit) ---
INSTRUCTION_SET = {
    # R-Type: OpCode(4) | Rd(4) | Rs1(4) | Rs2(4)
    'ADD': '0001', 
    'CMP': '0010', 
    'SHR': '1001', 
    'SHL': '1010',
    'SUB': '1101', 

    # I-Type (Rd | Rs | Imm[3:0] or Shamt[3:0] or Addr_Low[3:0])
    'LOAD': '0011', 
    'STORE': '0100', 
    'SHRI': '1011', 
    'SHLI': '1100', 
     

    # I-Type (8-bit Imm): OpCode(4) | Rd(4) | Imm_High(4) | Imm_Low(4)
    'ADDI': '1000',
    'SUBI': '1110',
    
    # J-Type/Branch (Reg + Imm): OpCode(4) | Rs(4) | Imm_High(4) | Imm_Low(4)
    'BZ': '0101', 
    'BN': '0110', 
    
    # Control: OpCode(4) | 0000 0000 0000
    'NOP': '0000', 
    'HALT': '0111',
}

# 寄存器映射 (R0 到 R15)
REGISTERS = {f'R{i}': i for i in range(16)}

def reg_to_bin(reg_name: str) -> str:
    """将寄存器名称转换为 4 位二进制"""
    index = REGISTERS.get(reg_name.upper())
    if index is None:
        raise ValueError(f"无效的寄存器名称: {reg_name}")
    return format(index, '04b')

def parse_val(value: str, bits: int) -> str:
    """解析立即数或地址，并格式化为指定位数的二进制字符串"""
    try:
        if isinstance(value, str) and value.lower().startswith('0x'):
            val = int(value, 16)
        else:
            val = int(value)
            
        if bits == 8:
            if not (-128 <= val <= 255):
                 raise ValueError(f"数值 {value} 超出 8 位范围 (-128 到 255)")
            val = val & 0xFF 
        elif not (0 <= val < (1 << bits)):
            raise ValueError(f"数值 {value} (十进制 {val}) 超出 {bits} 位范围 (0-{(1 << bits) - 1})")
            
        return format(val, f'0{bits}b')
    except ValueError as e:
        if "out of range" in str(e):
             raise ValueError(f"无效数值或地址: {value} (超出 {bits} 位范围)")
        raise ValueError(f"无效数值或地址: {value}")


def translate_line(instruction: str, operands: list, labels: dict) -> str:
    """将单条汇编指令翻译为 16 位二进制机器码"""
    instruction = instruction.upper()
    op_code = INSTRUCTION_SET.get(instruction)

    if op_code is None:
        raise ValueError(f"不支持的指令: {instruction}")

    # --- 1. Control Type (NOP, HALT) ---
    if instruction in ['NOP', 'HALT']:
        if operands: 
            raise ValueError(f"指令 {instruction} 不接受操作数")
        return op_code + '000000000000'
        
    # --- 2. R-Type (ADD, CMP, SHR, SHL, SUB) ---
    elif instruction in ['ADD', 'CMP', 'SHR', 'SHL', 'SUB']:
        if len(operands) != 3:
            raise ValueError(f"指令 {instruction} 需要 3 个寄存器操作数 (Rd, Rs1, Rs2)")
        rd, rs, rt = operands
        return op_code + reg_to_bin(rd) + reg_to_bin(rs) + reg_to_bin(rt)
        
    # --- 3. I-Type ---
    
    # 3.1 I-Type (Rd, Rs1, Imm[3:0] / Shamt[3:0] / Addr_Low[3:0])
    elif instruction in ['LOAD', 'STORE', 'SHRI', 'SHLI']:
        if len(operands) != 3:
            raise ValueError(f"指令 {instruction} 需要 2 个寄存器和 1 个 4 位立即数/地址 (Rd, Rs1, Imm)")
        
        rd, rs1, imm = operands
        return op_code + reg_to_bin(rd) + reg_to_bin(rs1) + parse_val(imm, 4)

    # 3.2 I-Type (ADDI) - 8-bit Imm 
    elif instruction in ['ADDI','SUBI']:
        if len(operands) != 2:
            raise ValueError(f"指令 {instruction} 需要 1 个寄存器和 1 个 8 位立即数 (Rd, Imm)")
        rd, imm = operands
        imm_val_bin = parse_val(imm, 8)
        imm_high = imm_val_bin[:4] 
        imm_low = imm_val_bin[4:] 
        return op_code + reg_to_bin(rd) + imm_high + imm_low

    # --- 4. Branch Type (BZ, BN) ---
    elif instruction in ['BN', 'BZ']:
        if len(operands) != 1: 
            raise ValueError(f"分支指令 {instruction} 格式应为 BZ LABEL")
            
        target_label = operands[0]
        target_addr = labels.get(target_label)
        
        if target_addr is None:
            raise ValueError(f"未找到标签地址: {target_label}")
            
        # ⭐ 核心修正: 适配 Verilog 中的 Reg + Imm 寻址
        # 策略: 使用 R0 ('0000') 作为基址寄存器 Rs (instr[11:8])，
        # 并将目标地址 Target_Addr 编码为 8 位立即数 Imm (instr[7:0])。
        # 假设 R0=0，Verilog计算 Target = Reg[R0] + Imm。
        
        rs_reg_bin = '0000' # Rs 字段使用 R0
        
        # 目标地址必须在 8 位无符号范围内 (0-255)
        if not (0 <= target_addr <= 255):
            raise ValueError(f"目标地址 {target_addr} 超出 8 位地址范围 (0-255)")
        
        # 将目标地址转换为 8 位无符号二进制
        target_addr_8bit_bin = format(target_addr, '08b')
        
        imm_high = target_addr_8bit_bin[:4] 
        imm_low = target_addr_8bit_bin[4:] 
        
        # 机器码格式: OpCode(4) | Rs(4) | Imm_High(4) | Imm_Low(4)
        return op_code + rs_reg_bin + imm_high + imm_low 
        
    return ""

def preprocess_asm(asm_code: str):
    """预处理汇编代码：计算标签地址，并清理指令和注释"""
    labels = {}
    current_addr = 0
    clean_asm_lines = []

    for line in asm_code.splitlines():
        line = line.strip()
        if not line:
            continue
        
        line = re.sub(r'^[;#].*$', '', line).strip()
        if not line:
            continue
            
        line = re.sub(r'[;#].*$', '', line).strip()
        if not line:
            continue

        parts = line.split()
        
        if line.endswith(':'):
            label = line.rstrip(':').strip()
            labels[label] = current_addr
            continue
            
        if ':' in parts[0]:
            label = parts[0].rstrip(':')
            labels[label] = current_addr
            parts.pop(0) 
            if not parts:
                continue
            line = ' '.join(parts)

        line = line.replace(',', ' ')
        clean_asm_lines.append((current_addr, line))
        current_addr += 1
        
    return labels, clean_asm_lines


def create_coe_program_16bit(asm_code: str, memory_depth_words: int = 128) -> str:
    """
    将汇编代码字符串转换为 16 位 COE 格式的机器码序列
    """
    labels, clean_asm_lines = preprocess_asm(asm_code)
            
    machine_code_16bit_hex = []
    
    for addr, line in clean_asm_lines:
        parts = line.split()
        instruction = parts[0].upper()
        operands = [p.upper() for p in parts[1:]]
        
        try:
            binary_code = translate_line(instruction, operands, labels)
            hex_code = format(int(binary_code, 2), '04x')
            machine_code_16bit_hex.append(hex_code)
            
        except ValueError as e:
            # 遇到错误时打印错误信息并填充 NOP (0000)
            sys.stderr.write(f"\n[Error at Addr 0x{format(addr, '02x')}] Line: '{line.strip()}' - {e}\n")
            machine_code_16bit_hex.append('0000') 
            
    coe_output = []
    coe_output.append("memory_initialization_radix = 16;")
    coe_output.append("memory_initialization_vector =")

    # --- 生成机器码序列，无逗号 ---
    
    # 1. 添加实际的机器码
    for hex_code in machine_code_16bit_hex:
        coe_output.append(hex_code)
    
    # 2. 填充剩余的内存空间 (0000)
    actual_depth = len(machine_code_16bit_hex)
    if actual_depth > memory_depth_words:
          memory_depth_words = actual_depth 
          
    # 填充 NOP 直到预设深度
    for _ in range(actual_depth, memory_depth_words):
          coe_output.append("0000")

    # 3. 修正最后一行：确保最后一行是分号，前面的行都没有逗号。
    if coe_output and len(coe_output) > 2:
        # 移除倒数第一行的内容 (即最后一个 0000 或指令)
        last_item = coe_output.pop()
        
        # 将最后一行内容重新添加，并加上分号
        coe_output.append(f"{last_item};")
        
    
    return "\n".join(coe_output)

# --- 测试用汇编代码 (包含分支测试) ---
NEW_ASSEMBLY_CODE = """
; ===============================
; 初始化常量和状态
; ===============================

    SUB  R1, R1, R1
    ADDI R1, 0x40      ; N_FLOORS = 64

    SUB  R2, R2, R2
    ADDI R2, 0x26      ; F_TRUE = 38

    SUB  R3, R3, R3    ; total_up = 0
    SUB  R4, R4, R4    ; total_down = 0
    SUB  R5, R5, R5    ; broken_eggs = 0
    SUB  R6, R6, R6    ; total_throws = 0

    ADD  R7, R1, R0    ; high = 64
    SUB  R8, R8, R8
    ADDI R8, 0x1       ; low = 1

    ADD  R9, R1, R0    ; last_floor = N_FLOORS

    SUB  R12, R12, R12 ; result_F = 0
    SUB  R14, R14, R14 ; cost_scarcity
    SUB  R15, R15, R15 ; cost_labor

; ===============================
; Phase 1: 二分查找 while (high - low >= 2)
; ===============================

PHASE1_CHECK:
    SUB  R11, R7, R8
    SUBI R11, 0x2
    CMP  R13, R11, R0
    BN   PHASE1_END
    ; diffHL >=2 → 进入循环体

PHASE1_LOOP:
    ADDI R6, 0x1       ; total_throws++

    ADD  R10, R7, R8
    SHRI R10, R10, 0x1 ; floor_to_test = (high+low)/2

    SUB  R11, R10, R9  ; diff = test - last

    CMP  R13, R11, R0
    BN   PH1_DIFF_DOWN
    BZ   PH1_DIFF_DOWN

PH1_DIFF_UP:
    ADD  R3, R3, R11
    CMP  R13, R0, R0
    BZ   PH1_DIFF_END

PH1_DIFF_DOWN:
    SUB  R13, R0, R11
    ADD  R4, R4, R13

PH1_DIFF_END:
    ADD  R9, R10, R0   ; last_floor = floor_to_test

    SUB  R11, R10, R2  ; diffTF
    CMP  R13, R11, R0
    BN   PH1_NO_BREAK
    BZ   PH1_NO_BREAK

PH1_BREAK:
    ADDI R5, 0x1
    SUBI R10, 0x1
    ADD  R7, R10, R0
    CMP  R13, R0, R0
    BZ   PHASE1_CHECK

PH1_NO_BREAK:
    ADD  R8, R10, R0
    CMP  R13, R0, R0
    BZ   PHASE1_CHECK

PHASE1_END:

; ===============================
; Phase 2
; ===============================

    SUB  R11, R7, R8
    CMP  R13, R11, R0
    BZ   PH2_EQUAL

    CMP  R13, R0, R0
    BZ   PH2_DIFF1

PH2_EQUAL:
    ADD  R12, R7, R0
    CMP  R13, R0, R0
    BZ   PHASE3_COST

PH2_DIFF1:
    ADDI R6, 0x01

    SUB  R11, R7, R9
    CMP  R13, R11, R0
    BN   PH2D1_DOWN
    BZ   PH2D1_DOWN

PH2D1_UP:
    ADD  R3, R3, R11
    CMP  R13, R0, R0
    BZ   PH2D1_DIFF_END

PH2D1_DOWN:
    SUB  R13, R0, R11
    ADD  R4, R4, R13

PH2D1_DIFF_END:
    SUB  R11, R7, R2
    CMP  R13, R11, R0
    BN   PH2D1_NOTBREAK
    BZ   PH2D1_NOTBREAK

PH2D1_BREAK:
    ADDI R5, 0x01
    ADD  R12, R8, R0
    CMP  R13, R0, R0
    BZ   PHASE3_COST

PH2D1_NOTBREAK:
    ADD  R12, R7, R0

; ===============================
; Phase 3 成本计算
; ===============================

PHASE3_COST:

    ; --- cost_scarcity = m*2 + n + h*4 ---
    ADD  R13, R3, R0
    SHLI R13, R13, 0x1     ; m*2
    ADD  R14, R13, R4

    ADD  R13, R5, R0
    SHLI R13, R13, 0x2     ; h*4
    ADD  R14, R14, R13

    ; --- cost_labor = m*4 + n + h*2 ---
    ADD  R13, R3, R0
    SHLI R13, R13, 0x2     ; m*4
    ADD  R15, R13, R4

    ADD  R13, R5, R0
    SHLI R13, R13, 0x1     ; h*2
    ADD  R15, R15, R13

    HALT


"""


if __name__ == "__main__":
    OUTPUT_FILENAME = "program.coe"
    try:
        coe_file_content = create_coe_program_16bit(NEW_ASSEMBLY_CODE)
        
        # 直接写入文件
        with open(OUTPUT_FILENAME, 'w', encoding='ascii') as f:
            f.write(coe_file_content)
            
        print(f"✅ 机器码已成功生成到文件: {OUTPUT_FILENAME}\n")
        print("--- 生成的 COE 文件内容 ---")
        print(coe_file_content)
        
    except Exception as e:
        sys.stderr.write(f"\n程序运行错误: {e}\n")
###
"""

; --- 0x00: 初始化常量和变量 ---
    ADDI R0 0           ; R0 = 0 (Constant)
    ADDI R12 38         ; R12 = F_TRUE = 38 (0x26)
    ADDI R13 64         ; R13 = N_FLOORS = 64 (0x40)
    
    ADDI R1 64          ; R1 = high = 64
    ADDI R2 1           ; R2 = low = 1
    ADDI R3 64          ; R3 = last_floor = 64
    ADDI R4 0           ; R4 = total_up_floors (m) = 0
    ADDI R5 0           ; R5 = total_down_floors (n) = 0
    ADDI R6 0           ; R6 = broken_eggs (h) = 0
    ADDI R7 0           ; R7 = total_throws (count) = 0
    ADDI R11 0          ; R11 = result_F = 0

; --- 0x0A: Phase 1: 二分查找循环 ---
LOOP_START:
    ; 检查循环条件: high - low >= 2
    SUB R9 R1 R2        ; R9 = high - low
    ADDI R10 2          ; R10 = 2
    
    CMP R0 R9 R10       ; R9 vs R10 sets flags based on R9 - R10
    BN  PHASE2_CHECK    ; If (high - low) < 2 (Negative), jump to PHASE2_CHECK
    
    ; 0x0E: total_throws++
    ADDI R7 1           ; R7 = total_throws + 1

    ; 0x0F: floor_to_test = (high + low) / 2
    ADD R8 R1 R2        ; R8 = high + low
    SHRI R8 R8 1        ; R8 = R8 / 2 (floor_to_test)
    
    ; 0x11: diff = floor_to_test - last_floor
    SUB R9 R8 R3        ; R9 = diff = R8 - R3

    ; 0x13: 成本计算: if (diff > 0)
    CMP R0 R9 R0        ; R9 vs R0 sets flags based on R9 - 0
    BN  COST_DOWN       ; If diff <= 0 (Negative or Zero), jump to COST_DOWN
    
    ; 0x15: diff > 0 (上楼): total_up_floors += diff
    ADD R4 R4 R9        ; R4 = m + diff

    ; === 修正点 1: 无条件跳转 ===
    CMP R0 R0 R0        ; Set Z=1
    BZ  LAST_FLOOR_UPDATE ; Unconditional Jump (Skip COST_DOWN)

COST_DOWN:
    ; 0x18: diff <= 0 (下楼或原地)
    ; abs_diff = 0 - diff 
    SUB R10 R0 R9       ; R10 = abs_diff
    ADD R5 R5 R10       ; R5 = n + abs_diff

LAST_FLOOR_UPDATE:
    ; 0x1A: last_floor = floor_to_test
    ADD R3 R8 R0        ; R3 = R8

    ; 0x1C: 模拟摔蛋: if (floor_to_test > F_TRUE)
    CMP R0 R8 R12       ; R8 vs R12 sets flags based on R8 - R12
    BN  EGG_SAFE        ; If R8 <= R12 (Negative or Zero), jump to EGG_SAFE

    ; 0x1E: 摔破了 (R8 > R12)
    ADDI R6 1           ; R6 = broken_eggs++
    SUBI R1 1           ; R1 = high - 1

    ; === 修正点 2: 无条件跳转 ===
    CMP R0 R0 R0        ; Set Z=1
    BZ  LOOP_START      ; Unconditional Jump

EGG_SAFE:
    ; 0x21: 没碎 (R8 <= R12)
    ADD R2 R8 R0        ; R2 = low = floor_to_test
    
    ; === 修正点 3: 无条件跳转 ===
    CMP R0 R0 R0        ; Set Z=1
    BZ LOOP_START       ; Unconditional Jump

; ==========================================================
; --- 0x24: Phase 2: 最终确认 (high - low < 2) ---
PHASE2_CHECK:
    SUB R9 R1 R2        ; R9 = high - low

    CMP R0 R9 R0        ; R9 vs 0
    BZ  FINALIZE_RESULT ; If R9 == 0, jump to FINALIZE_RESULT (high == low)

; --- 0x27: high = low + 1 (需试摔 high 楼层) ---
    ADDI R7 1           ; R7 = total_throws++
    
    ; 0x28: 成本计算 (测试 high 楼层)
    SUB R9 R1 R3        ; R9 = diff = high - last_floor
    
    CMP R0 R9 R0        ; R9 vs 0
    BN  COST_DOWN_F     ; If diff <= 0, jump to COST_DOWN_F
    
    ADD R4 R4 R9        ; R4 = m + diff (diff > 0)

    ; === 修正点 4: 无条件跳转 ===
    CMP R0 R0 R0        ; Set Z=1
    BZ  FINAL_THROW     ; Unconditional Jump (Skip COST_DOWN_F)

COST_DOWN_F:
    SUB R10 R0 R9       ; R10 = abs_diff = 0 - diff
    ADD R5 R5 R10       ; R5 = n + abs_diff

FINAL_THROW:
    ; 0x2F: 最终试摔 high 楼层: if (high > F_TRUE)
    CMP R0 R1 R12       ; R1 vs R12
    BN  FINAL_SAFE      ; If R1 <= R12, jump to FINAL_SAFE

    ; 0x31: 摔破了 (R1 > R12)
    ADDI R6 1           ; R6 = broken_eggs++
    ADD R11 R2 R0       ; R11 = result_F = low
    
    CMP R0 R0 R0        ; Set Z=1
    BZ  CALCULATE_COSTS ; Unconditional Jump to cost calc

FINAL_SAFE:
    ; 0x35: 没碎 (R1 <= R12)
    ADD R11 R1 R0       ; R11 = result_F = high

; --- 0x36: 跳转到成本计算 (如果 FINAL_SAFE 被执行) ---
CALCULATE_COSTS_JUMP:
    CMP R0 R0 R0        ; Set Z=1
    BZ  CALCULATE_COSTS ; Unconditional Jump

; --- 0x38: high == low (无需试摔) ---
FINALIZE_RESULT:
    ADD R11 R1 R0       ; R11 = result_F = high (或 low)

; --- 0x39: 跳转到成本计算 (如果 FINALIZE_RESULT 被执行) ---
    CMP R0 R0 R0        ; Set Z=1
    BZ  CALCULATE_COSTS ; Unconditional Jump

; ==========================================================
; --- 0x3B: Phase 3: 成本计算 f = m*p1 + n*p2 + h*p3 ---

CALCULATE_COSTS:
; ... 成本计算逻辑保持不变 ...

; --- 0x3B: 成本分析 - 物质匮乏时期 (p1=2, p2=1, p3=4) ---
; Cost = m*2 + n*1 + h*4
    
    ; R4 * 2 (m*p1)
    SHLI R14 R4 1       ; R14 = R4 << 1 (m * 2)
    
    ; R5 * 1 (n*p2)
    ADD R14 R14 R5      ; R14 = m*2 + n
    
    ; R6 * 4 (h*p3)
    SHLI R15 R6 2       ; R15 = R6 << 2 (h * 4)
    
    ; R14 = R14 + R15 
    ADD R14 R14 R15     ; R14 (Result 1) = Cost Scarcity

; --- 0x40: 成本分析 - 人力成本增长时期 (p1=4, p2=1, p3=2) ---
; Cost = m*4 + n*1 + h*2

    ; R4 * 4 (m*p1)
    SHLI R15 R4 2       ; R15 = R4 << 2 (m * 4)
    
    ; R5 * 1 (n*p2)
    ADD R15 R15 R5      ; R15 = m*4 + n
    
    ; R6 * 2 (h*p3)
    SHLI R9 R6 1        ; R9 = R6 << 1 (h * 2)
    
    ; R15 = R15 + R9 
    ADD R15 R15 R9      ; R15 (Result 2) = Cost Labor

; --- 0x45: 结束 ---
    HALT
"""
###