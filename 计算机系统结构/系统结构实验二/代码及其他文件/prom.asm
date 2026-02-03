# ---- 初始化 ----
ADDI R1, R0, 0        # a = 0
ADDI R2, R0, 1        # b = 1
ADDI R5, R0, 0        # i = 0

ADDI R7, R0, 1
ADDI R8, R0, 3
ADDI R9, R0, 19
ADDI R10, R0, 39
ADDI R11, R0, 59

LOOP:
# if (i > 0)
BEQ  R5, R0, SKIP_AB

ADD  R1, R1, R5       # a += i
MUL  R6, R5, R8
ADD  R2, R2, R6       # b += 3*i

SKIP_AB:

# ---- c[i] ----
SLTI R6, R5, 20
BNE  R6, R0, C_IS_A

SLTI R6, R5, 40
BNE  R6, R0, C_IS_SUM

MUL  R3, R1, R2       # c = a*b
JMP  C_DONE

C_IS_SUM:
ADD  R3, R1, R2       # c = a+b
JMP  C_DONE

C_IS_A:
ADD  R3, R1, R0       # c = a

C_DONE:

# ---- d[i] ----
SLTI R6, R5, 20
BNE  R6, R0, D_IS_B

SLTI R6, R5, 40
BNE  R6, R0, D_IS_AC

MUL  R4, R3, R2       # d = c*b
JMP  D_DONE

D_IS_AC:
MUL  R4, R1, R3       # d = a*c
JMP  D_DONE

D_IS_B:
ADD  R4, R2, R0       # d = b

D_DONE:

# i++
ADD  R5, R5, R7
BNE  R5, R11, LOOP

HALT
