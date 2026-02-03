import re

# =========================================================
# 1. Opcode 定义（与你 Verilog 完全一致，6 bit）
# =========================================================
OPCODES = {
    # 算术
    'ADD':  0b000001,
    'SUB':  0b000010,
    'MUL':  0b000011,
    'DIV':  0b000100,
    'ADDU': 0b011110,
    'SUBU': 0b011111,

    # I-Type
    'SLTI':  0b000101,
    'SLTIU': 0b000110,
    'ADDI':  0b000111,
    'SUBI':  0b001000,
    'LUI':   0b001001,
    'LW':    0b001010,
    'SW':    0b001011,
    'XORI':  0b001111,

    # 逻辑
    'AND': 0b001100,
    'OR':  0b001101,
    'XOR': 0b001110,

    # Shift
    'SLL': 0b010010,
    'SRL': 0b010011,
    'SRA': 0b010100,

    'SLLV': 0b100000,
    'SRLV': 0b100001,
    'SRAV': 0b100010,

    # 比较
    'SLT':  0b010101,
    'SLTU': 0b010110,

    # 跳转 / 分支
    'JMP':  0b010111,
    'JAL':  0b011000,
    'BEQ':  0b011010,
    'BNE':  0b011011,
    'BGTZ': 0b011100,
    'BLTZ': 0b011101,

    # 系统
    'NOP':  0b111110,
    'HALT': 0b111111,
}

REG = {f'R{i}': i for i in range(32)}


def parse_int(s: str) -> int:
    """
    支持：
      - 十进制：10
      - 十六进制：0x10
      - 负数：-5 / -0x5
    """
    return int(s, 0)


# =========================================================
# 2. 预处理：标签扫描（word-PC：每条指令 pc += 1）
# =========================================================
def preprocess(lines):
    labels = {}
    pc = 0          # word address: 第几条指令
    cleaned = []

    for line in lines:
        line = line.split('#')[0].split(';')[0].strip()
        if not line:
            continue

        # 纯标签行：LABEL:
        if line.endswith(':'):
            labels[line[:-1].strip()] = pc
            continue

        # 行内标签：LABEL: INSTR ...
        if ':' in line:
            label, rest = line.split(':', 1)
            labels[label.strip()] = pc
            line = rest.strip()
            if not line:
                continue

        cleaned.append((pc, line))
        pc += 1

    return labels, cleaned


# =========================================================
# 3. 单条指令编码（word-PC 版本）
# =========================================================
def assemble(line, pc, labels):
    parts = re.split(r'[,\s]+', line.strip())
    parts = [p for p in parts if p]  # 去掉空 token
    op = parts[0].upper()

    if op not in OPCODES:
        raise ValueError(f'Unknown instruction: {op}')

    opcode = OPCODES[op] << 26

    # ---------------- NOP / HALT ----------------
    if op in ('NOP', 'HALT'):
        return opcode

    # ---------------- J-Type ----------------
    # 这里按 word-PC：target 直接写 label 的“指令索引”
    if op in ('JMP', 'JAL'):
        if len(parts) != 2:
            raise ValueError(f'Bad {op} format. Expected: {op} LABEL')
        label = parts[1]
        if label not in labels:
            raise ValueError(f'Unknown label: {label}')
        target = labels[label] & 0x03FFFFFF
        return opcode | target

    # ---------------- Branch ----------------
    # offset = label - (pc + 1)   （word-PC 下，“下一条指令”为基准）
    if op in ('BEQ', 'BNE'):
        if len(parts) != 4:
            raise ValueError(f'Bad {op} format. Expected: {op} RS, RT, LABEL')
        rs = REG[parts[1].upper()]
        rt = REG[parts[2].upper()]
        label = parts[3]
        if label not in labels:
            raise ValueError(f'Unknown label: {label}')
        offset = labels[label] - (pc + 1)
        return opcode | (rs << 21) | (rt << 16) | (offset & 0xFFFF)

    if op in ('BGTZ', 'BLTZ'):
        if len(parts) != 3:
            raise ValueError(f'Bad {op} format. Expected: {op} RS, LABEL')
        rs = REG[parts[1].upper()]
        label = parts[2]
        if label not in labels:
            raise ValueError(f'Unknown label: {label}')
        offset = labels[label] - (pc + 1)
        return opcode | (rs << 21) | (offset & 0xFFFF)

    # ---------------- R-Type ----------------
    if op in (
        'ADD', 'SUB', 'MUL', 'DIV', 'ADDU', 'SUBU',
        'AND', 'OR', 'XOR', 'SLT', 'SLTU',
        'SLLV', 'SRLV', 'SRAV'
    ):
        if len(parts) != 4:
            raise ValueError(f'Bad {op} format. Expected: {op} RD, RS, RT')
        rd = REG[parts[1].upper()]
        rs = REG[parts[2].upper()]
        rt = REG[parts[3].upper()]
        return opcode | (rs << 21) | (rt << 16) | (rd << 11)

    # ---------------- Shift (shamt) ----------------
    if op in ('SLL', 'SRL', 'SRA'):
        if len(parts) != 4:
            raise ValueError(f'Bad {op} format. Expected: {op} RD, RT, SHAMT')
        rd = REG[parts[1].upper()]
        rt = REG[parts[2].upper()]
        sh = parse_int(parts[3]) & 0x1F
        return opcode | (rt << 16) | (rd << 11) | (sh << 6)

    # ---------------- I-Type ----------------
    # rt, rs, imm
    if op in ('ADDI', 'SUBI', 'SLTI', 'SLTIU', 'XORI', 'LW'):
        if len(parts) != 4:
            raise ValueError(f'Bad {op} format. Expected: {op} RT, RS, IMM')
        rt = REG[parts[1].upper()]
        rs = REG[parts[2].upper()]
        imm = parse_int(parts[3]) & 0xFFFF
        return opcode | (rs << 21) | (rt << 16) | imm

    if op == 'SW':
        if len(parts) != 4:
            raise ValueError('Bad SW format. Expected: SW RT, RS, IMM')
        rt = REG[parts[1].upper()]
        rs = REG[parts[2].upper()]
        imm = parse_int(parts[3]) & 0xFFFF
        return opcode | (rs << 21) | (rt << 16) | imm

    if op == 'LUI':
        if len(parts) != 3:
            raise ValueError('Bad LUI format. Expected: LUI RT, IMM')
        rt = REG[parts[1].upper()]
        imm = parse_int(parts[2]) & 0xFFFF
        return opcode | (rt << 16) | imm

    raise ValueError(f'Unsupported instruction: {op}')


# =========================================================
# 4. 生成 COE（逐项逗号分隔）
# =========================================================
def assemble_to_coe(asm_text, depth=256):
    lines = asm_text.splitlines()
    labels, cleaned = preprocess(lines)

    code = []
    for pc, line in cleaned:
        instr = assemble(line, pc, labels)
        code.append(f'{instr:08x}')

    if len(code) > depth:
        raise ValueError(f'Program too large: {len(code)} instructions, depth={depth}')

    while len(code) < depth:
        code.append('00000000')

    return (
        'memory_initialization_radix=16;\n'
        'memory_initialization_vector=\n'
        + ',\n'.join(code) +
        ';\n'
    )


# =========================================================
# 5. CLI
# =========================================================
if __name__ == '__main__':
    with open('prog.asm', encoding='utf-8') as f:
        asm = f.read()

    coe = assemble_to_coe(asm, depth=256)

    with open('program.coe', 'w', encoding='utf-8') as f:
        f.write(coe)

    print('✅ program.coe generated')
