#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rv_emu.h"
#include "bits.h"

#define DEBUG 0

static void unsupported(char *s, uint32_t n) {
    printf("unsupported %s 0x%x\n", s, n);
    exit(-1);
}

void emu_r_type(rv_state *rsp, uint32_t iw) {
    uint32_t rd = (iw >> 7) & 0b11111;
    uint32_t rs1 = (iw >> 15) & 0b11111;
    uint32_t rs2 = (iw >> 20) & 0b11111;
    uint32_t funct3 = (iw >> 12) & 0b111;
    uint32_t funct7 = (iw >> 25) & 0b1111111;

    if (funct3 == 0b000 && funct7 == 0b0000000) {
        // if statement for add in riscv
        rsp->regs[rd] = rsp->regs[rs1] + rsp->regs[rs2];
    } 
    else if (funct3 == 0b000 && funct7 == 0b0000001) {
        // if for multiply
        rsp->regs[rd] = rsp->regs[rs1] * rsp->regs[rs2];
    }
    else if (funct3 == 0b000 && funct7 == 0b0100000) {
        // if for minus
        rsp->regs[rd] = rsp->regs[rs1] - rsp->regs[rs2];
    }
    else if (funct3 == 0b100 && funct7 == 0b0000001) {
        // if for divide
        rsp->regs[rd] = rsp->regs[rs1] / rsp->regs[rs2];
    }
    else if (funct3 == 0b101 && funct7 == 0b0000000) {
        // if for shift right
        rsp->regs[rd] = rsp->regs[rs1] >> rsp->regs[rs2];
    }
    else if (funct3 == 0b001 && funct7 == 0b0000000) {
        // if for shift left
        rsp->regs[rd] = rsp->regs[rs1] << rsp->regs[rs2];
    }
    else if (funct3 == 0b111 && funct7 == 0b0000000) {
        // if for bitwise AND
        rsp->regs[rd] = rsp->regs[rs1] & rsp->regs[rs2];
    }
    else if (funct3 == 0b110 && funct7 == 0b0000001) {
        // if for rem
        rsp->regs[rd] = rsp->regs[rs1] % rsp->regs[rs2];
    }
    else {
        unsupported("R-type funct3", funct3);
    }
    rsp->pc += 4; // Next instruction
}

int emu_b_type(rv_state *rsp, uint32_t iw) {

    uint32_t immed11 = (iw >> 7) & 0b1;
    uint32_t immed41 = (iw >> 8) & 0b1111;
    uint32_t funct3 = (iw >> 12) & 0b111;
    uint32_t rs1 = (iw >> 15) & 0b11111;
    uint32_t rs2 = (iw >> 20) & 0b11111;
    uint32_t immed105 = (iw >> 25) & 0b111111;
    uint32_t immed12 = (iw >> 31) & 0b1;
    uint32_t imm = (immed12 << 12) | (immed11 << 11) | (immed105 << 5) | (immed41 << 1);
 
    int32_t immSigned = sign_extend(imm, 12);
    //redefine it in signed variable and shift right by len

    if (funct3 == 0b100 && (int)rsp->regs[rs1] < (int)rsp->regs[rs2]) {
        //BLT condition
        rsp->pc += immSigned;
    }
    else if (funct3 == 0b001 && (int)rsp->regs[rs1] != (int)rsp->regs[rs2]) {
        //BNE condition
        rsp->pc += immSigned;
    }
    else if (funct3 == 0b000 && (int)rsp->regs[rs1] == (int)rsp->regs[rs2]) {
        //BEQ condition
        rsp->pc += immSigned;
    }
    else if (funct3 == 0b101 && (int)rsp->regs[rs1] >= (int)rsp->regs[rs2]) {
        //BGE condition
        rsp->pc += immSigned;
    }
    else if (funct3 == 0b000 && (int)rsp->regs[rs1] == 0 && (int)rsp->regs[rs2] == 0) {
        //BEQZ condition
        rsp->pc += immSigned;
    }
    else {
        //next instruction
        rsp->pc += 4;
        return 0;
    }
    return 1;
}

void emu_j_type(rv_state *rsp, uint32_t iw) {
    // uint32_t rd = (iw >> 7) & 0b11111;
    uint32_t rd = get_bits(iw, 7, 5);
    uint32_t immed1912 = (iw >> 12) & 0b11111111;
    uint32_t immed11 = (iw >> 20) & 0b1;
    uint32_t immed101 = (iw >> 21) & 0b1111111111;
    uint32_t immed20 = (iw >> 31) & 0b1;
    uint32_t imm = (immed20 << 20) | (immed1912 << 12) | (immed11 << 11) | (immed101 << 1);
    int32_t shift = sign_extend(imm, 20);
    //redefine it in signed variable and shift right by len
    
    if (rd != 0) {
        //jump/JAL condition
        rsp->regs[rd] = (uint64_t)rsp->pc + 4;
    }

    rsp->pc += shift; //next instruction
}

void emu_i_type(rv_state *rsp, uint32_t iw) {
    uint32_t rd = (iw >> 7) & 0b11111;
    uint32_t funct3 = (iw >> 12) & 0b111;
    uint32_t rs1 = (iw >> 15) & 0b11111;
    uint32_t rs2 = (iw >> 20) & 0b11111;
    uint32_t imm = (iw >> 20) & 0b111111111111;
    int32_t shift = sign_extend(imm, 12); 
    //redefine it in signed variable and shift right by len

    if (funct3 == 0b101) {
        //SRLI condition
        rsp->regs[rd] = rsp->regs[rs1] >> rs2;
    }
    else if (funct3 == 0b001) {
        //SLLI condition
        rsp->regs[rd] = rsp->regs[rs1] << imm;
    }
    else if (funct3 == 0b111) {
        //ANDI condition
        rsp->regs[rd] = rsp->regs[rs1] & imm;
    }
    else if (funct3 == 0b110) {
        //ORI condition
        rsp->regs[rd] = rsp->regs[rs1] | imm;
    }
    else if (funct3 == 0b000) {
        //MV, LI, ADDI condition
        rsp->regs[rd] = rsp->regs[rs1] + shift;
    }
    else {
        unsupported("I-type funct3", funct3);
    }
    rsp->pc += 4; //next instruction
}

void emu_s_type(rv_state *rsp, uint32_t iw) {
    uint32_t funct3 = get_bits(iw, 12, 3);
    uint32_t rs1 = get_bits(iw, 15, 5);
    uint32_t rs2 = get_bits(iw, 20, 5);
    uint64_t imm = (uint32_t)(get_bits(iw, 25, 7) << 5) | (uint32_t)(get_bits(iw, 7, 5));
    imm = sign_extend(imm, 12);

    if (funct3 == 0b000) {
        //sb
        *(uint8_t*)(rsp->regs[rs1] + imm) = rsp->regs[rs2];
    }
    else if (funct3 == 0b011) {
        //sd
        *(uint64_t*)(rsp->regs[rs1] + imm) = rsp->regs[rs2];
    }
    else if (funct3 == 0b010) {
        //sw
        *(uint32_t*)(rsp->regs[rs1] + imm) = rsp->regs[rs2];
    }
    else {
        unsupported("S-type function", funct3);
    }
    rsp->pc += 4;
}

void emu_ld_type(rv_state *rsp, uint32_t iw) {
    uint32_t rd = get_bits(iw, 7, 5);
    uint32_t funct3 = get_bits(iw, 12, 3);
    uint32_t rs1 = get_bits(iw, 15, 5);
    uint32_t imm = get_bits(iw, 20, 12);
    imm = sign_extend(imm, 12);

    if (funct3 == 0b000) {
        //lb
        rsp->regs[rd] = *(uint8_t*)(rsp->regs[rs1] + imm);
    }
    else if (funct3 == 0b011) {
        //ld
        rsp->regs[rd] = *(uint64_t*)(rsp->regs[rs1] + imm);
    }
    else if (funct3 == 0b010) {
        //lw
        rsp->regs[rd] = *(uint32_t*)(rsp->regs[rs1] + imm);
    }
    else {
        unsupported("LD-type function", funct3);
    }
    rsp->pc += 4;
}

void emu_jalr(rv_state *rsp, uint32_t iw) {
    uint32_t rs1 = (iw >> 15) & 0b1111;  // Will be ra (aka x1)
    uint64_t val = rsp->regs[rs1];  // Value of regs[1]

    rsp->pc = val;  // PC = return address
}

void rv_counter(rv_analysis *a, uint32_t opc, int truth) {

    switch (opc) {
        case 0b0110011:
            // R-type
            a->ir_count++;
            break;
        case 0b0010011:
            // I-type
            a->ir_count++;
            break;
        case 0b0000011:
            // ld-type
            a->ld_count++;
            break;
        case 0b0100011:
            // S-type
            a->st_count++;
            break;
        case 0b1100111:
            // JALR
            a->j_count++;
            break;
        case 0b1101111:
            // J and JAL
            a->j_count++;
            break;
        case 0b1100011:
            // B-type
            if (truth) {
                a->b_taken++;
            }
            else {
                a->b_not_taken++;
            }
            break;
        default:
            unsupported("Unknown opcode: ", opc);
    }
    a->i_count++;
}

static void rv_one(rv_state *state) {
    uint32_t iw  = *((uint32_t*) state->pc);
    uint32_t og = iw;
    iw = cache_lookup(&state->i_cache, (uint64_t) state->pc);
    // initiates cache process
    // saves original iw value
    iw = og;

    uint32_t opcode = get_bits(iw, 0, 7);
    // total counter
    int truth = 0;


#if DEBUG
    printf("iw: %x\n", iw);
#endif

    switch (opcode) {
        case 0b0110011:
            // R-type instructions have two register operands
            emu_r_type(state, iw);
            break;
        case 0b1100111:
            // JALR (aka RET) is a variant of I-type instructions
            emu_jalr(state, iw);
            break;
        case 0b0100011:
            // S-type instructions are st instructions (no rd)
            emu_s_type(state, iw);
            break;
        case 0b1100011:
            // SB-type instructions are conditional branch instructions
            truth = emu_b_type(state, iw);
            break;
        case 0b1101111:
            // UJ-type instructions are jump instructions j and jal (aka call)
            emu_j_type(state, iw);
            break;
        case 0b0010011:
            // I-type instructions
            emu_i_type(state, iw);
            break;
        case 0b0000011:
            // LD-type instructions
            emu_ld_type(state, iw);
            break;
        default:
            unsupported("Unknown opcode: ", opcode);
    }
    if (state->analyze) {
        rv_counter(&state->analysis, opcode, truth);
    }
}

void rv_init(rv_state *state, uint32_t *target, 
             uint64_t a0, uint64_t a1, uint64_t a2, uint64_t a3) {
    state->pc = (uint64_t) target;
    state->regs[RV_A0] = a0;
    state->regs[RV_A1] = a1;
    state->regs[RV_A2] = a2;
    state->regs[RV_A3] = a3;

    state->regs[RV_ZERO] = 0;  // zero is always 0  (:
    state->regs[RV_RA] = RV_STOP;
    state->regs[RV_SP] = (uint64_t) &state->stack[STACK_SIZE];

    memset(&state->analysis, 0, sizeof(rv_analysis));
    cache_init(&state->i_cache);
}

uint64_t rv_emulate(rv_state *state) {
    while (state->pc != RV_STOP) {
        rv_one(state);
    }
    return state->regs[RV_A0];
}

static void print_pct(char *fmt, int numer, int denom) {
    double pct = 0.0;

    if (denom)
        pct = (double) numer / (double) denom * 100.0;
    printf(fmt, numer, pct);
}

void rv_print(rv_analysis *a) {
    int b_total = a->b_taken + a->b_not_taken;

    printf("=== Analysis\n");
    print_pct("Instructions Executed  = %d\n", a->i_count, a->i_count);
    print_pct("R-type + I-type        = %d (%.2f%%)\n", a->ir_count, a->i_count);
    print_pct("Loads                  = %d (%.2f%%)\n", a->ld_count, a->i_count);
    print_pct("Stores                 = %d (%.2f%%)\n", a->st_count, a->i_count);    
    print_pct("Jumps/JAL/JALR         = %d (%.2f%%)\n", a->j_count, a->i_count);
    print_pct("Conditional branches   = %d (%.2f%%)\n", b_total, a->i_count);
    print_pct("  Branches taken       = %d (%.2f%%)\n", a->b_taken, b_total);
    print_pct("  Branches not taken   = %d (%.2f%%)\n", a->b_not_taken, b_total);
}
