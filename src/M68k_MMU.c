
#include <stdint.h>
#include "../include/support.h"
#include "../include/M68k.h"
#include "../include/A64.h"
#include "../include/cache.h"

void invalidate_entire_dcache(void);
/* Invalidate entire data cache, code after ARMv8 architecture reference manual */
void __attribute__((used)) __invalidate_entire_dcache(void)
{
    asm volatile(
"       .globl  invalidate_entire_dcache\n"
"invalidate_entire_dcache:              \n"
"       stp     x0, x1, [sp, #-112]!    \n"
"       stp     x2, x3, [sp, #16]       \n"
"       stp     x4, x5, [sp, #2*16]     \n"
"       stp     x7, x8, [sp, #3*16]     \n"
"       stp     x9, x10, [sp, #4*16]    \n"
"       stp     x11, x16, [sp, #5*16]   \n"
"       str     x17, [sp, #6*16]        \n"
"       mrs     x0, CLIDR_EL1           \n"
"       and     w3, w0, #0x07000000     \n" // Get 2 x Level of Coherence
"       lsr     w3, w3, #23             \n"
"       cbz     w3, 5f                  \n"
"       mov     w10, #0                 \n" // W10 = 2 x cache level
"       mov     w8, #1                  \n" // W8 = constant 0b1
"1:     add     w2, w10, w10, lsr #1    \n" // Calculate 3 x cache level
"       lsr     w1, w0, w2              \n" // extract 3-bit cache type for this level
"       and     w1, w1, #0x7            \n"
"       cmp     w1, #2                  \n"
"       b.lt    4f                      \n" // No data or unified cache at this level
"       msr     CSSELR_EL1, x10         \n" // Select this cache level
"       isb                             \n" // Synchronize change of CSSELR
"       mrs     x1, CCSIDR_EL1          \n" // Read CCSIDR
"       and     w2, w1, #7              \n" // W2 = log2(linelen)-4
"       add     w2, w2, #4              \n" // W2 = log2(linelen)
"       ubfx    w4, w1, #3, #10         \n" // W4 = max way number, right aligned
"       clz     w5, w4                  \n" // W5 = 32-log2(ways), bit position of way in DC operand
"       lsl     w9, w4, w5              \n" // W9 = max way number, aligned to position in DC operand
"       lsl     w16, w8, w5             \n" // W16 = amount to decrement way number per iteration
"2:     ubfx    w7, w1, #13, #15        \n" // W7 = max set number, right aligned
"       lsl     w7, w7, w2              \n" // W7 = max set number, aligned to position in DC operand
"       lsl     w17, w8, w2             \n" // W17 = amount to decrement set number per iteration
"3:     orr     w11, w10, w9            \n" // W11 = combine way number and cache number ...
"       orr     w11, w11, w7            \n" // ... and set number for DC operand
"       dc      ISW, x11                \n" // Do data cache invalidate by set and way
"       subs    w7, w7, w17             \n" // Decrement set number
"       b.ge    3b                      \n"
"       subs    x9, x9, x16             \n" // Decrement way number
"       b.ge    2b                      \n"
"4:     add     w10, w10, #2            \n" // Increment 2 x cache level
"       cmp     w3, w10                 \n"
"       dsb     sy                      \n" // Ensure completion of previous cache maintenance instruction
"       b.gt    1b                      \n"
"5:                                     \n"
"       ldp     x2, x3, [sp, #16]       \n"
"       ldp     x4, x5, [sp, #2*16]     \n"
"       ldp     x7, x8, [sp, #3*16]     \n"
"       ldp     x9, x10, [sp, #4*16]    \n"
"       ldp     x11, x16, [sp, #5*16]   \n"
"       ldr     x17, [sp, #6*16]        \n"
"       ldp     x0, x1, [sp], #112      \n"
"       ret                             \n"
"       .ltorg                          \n"
    );
}

void trampoline_icache_invalidate(void);
void __attribute__((used)) __trampoline_icache_invalidate(void)
{
    asm volatile(".globl trampoline_icache_invalidate\ntrampoline_icache_invalidate: bl invalidate_instruction_cache\n\tbr x0");
}

uint32_t *EMIT_CINV(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	if ((opcode & 0x0018) != 0) {

		uint8_t tmp = 0xff;
        uint8_t tmp2 = 0xff;
        uint8_t tmp3 = 0xff;
        uint8_t tmp4 = 0xff;

        ptr = EMIT_FlushPC(ptr);

        /* Invalidating data cache? */
        if (opcode & 0x40) {
            /* Get the scope */
            switch (opcode & 0x18) {
                case 0x08:  /* Line */
                    tmp = RA_CopyFromM68kRegister(&ptr, 8 + (opcode & 7));
                    tmp2 = RA_AllocARMRegister(&ptr);
                    tmp3 = RA_AllocARMRegister(&ptr);
                    *ptr++ = mov_immed_u8(tmp3, 4);
                    *ptr++ = mrs(tmp2, 3, 3, 0, 0, 1); // Get CTR_EL0
                    *ptr++ = ubfx(tmp2, tmp2, 16, 4);
                    *ptr++ = lslv(tmp2, tmp3, tmp2);
                    *ptr++ = sub_immed(tmp2, tmp2, 1);
                    *ptr++ = dsb_sy();
                    *ptr++ = bic_reg(tmp, tmp, tmp2, LSL, 0);
                    *ptr++ = dc_ivac(tmp);
                    *ptr++ = dsb_sy();
                    RA_FreeARMRegister(&ptr, tmp2);
                    RA_FreeARMRegister(&ptr, tmp3);
                    RA_FreeARMRegister(&ptr, tmp);
                    break;
                case 0x10:  /* Page */
                    tmp = RA_CopyFromM68kRegister(&ptr, 8 + (opcode & 7));
                    tmp2 = RA_AllocARMRegister(&ptr);
                    tmp3 = RA_AllocARMRegister(&ptr);
                    tmp4 = RA_AllocARMRegister(&ptr);
                    *ptr++ = mrs(tmp3, 3, 3, 0, 0, 1); // Get CTR_EL0
                    *ptr++ = ubfx(tmp3, tmp3, 16, 4);
                    *ptr++ = mov_immed_u16(tmp2, 1024, 0);
                    *ptr++ = lsrv(tmp2, tmp2, tmp3);
                    *ptr++ = bic_immed(tmp, tmp, 12, 0);
                    *ptr++ = mov_immed_u8(tmp4, 4);
                    *ptr++ = lslv(tmp4, tmp4, tmp3);
                    *ptr++ = dc_ivac(tmp);
                    *ptr++ = add_reg(tmp, tmp, tmp4, LSL, 0);
                    *ptr++ = subs_immed(tmp2, tmp2, 1);
                    *ptr++ = b_cc(A64_CC_NE, -3);
                    *ptr++ = dsb_sy();
                    RA_FreeARMRegister(&ptr, tmp3);
                    RA_FreeARMRegister(&ptr, tmp4);
                    RA_FreeARMRegister(&ptr, tmp);
                    RA_FreeARMRegister(&ptr, tmp2);
                    break;
                case 0x18: { /* All */
                        union {
                            uint64_t u64;
                            uint32_t u32[2];
                        } u;

                        u.u64 = (uintptr_t)invalidate_entire_dcache;

                        *ptr++ = stp64_preindex(31, 0, 30, -16);
                        *ptr++ = ldr64_pcrel(0, 4);
                        *ptr++ = blr(0);
                        *ptr++ = ldp64_postindex(31, 0, 30, 16);
                        *ptr++ = b(3);
                        *ptr++ = u.u32[0];
                        *ptr++ = u.u32[1];
                	}
                    break;
            }
        }
        /* Invalidating instruction cache? */
        if (opcode & 0x80) {
            int8_t off = 0;
            ptr = EMIT_GetOffsetPC(ptr, &off);

            union {
                uint64_t u64;
                uint32_t u32[2];
            } u;
            u.u64 = (uintptr_t)trampoline_icache_invalidate;

            *ptr++ = stp64_preindex(31, 0, 1, -176);
            for (int i=2; i < 20; i+=2)
                *ptr++ = stp64(31, i, i + 1, i * 8);
            *ptr++ = stp64(31, 29, 30, 160);
            if ((opcode & 0x18) == 0x08 || (opcode & 0x18) == 0x10)
            {
                uint8_t tmp = RA_MapM68kRegister(&ptr, 8 + (opcode & 7));
                *ptr++ = mov_reg(0, tmp);
            }
            if (off >= 0)
                *ptr++ = add_immed(1, REG_PC, off);
            else
                *ptr++ = sub_immed(1, REG_PC, -off);

            *ptr++ = adr(2, 4*6);
            *ptr++ = ldr64_pcrel(3, 3);
            *ptr++ = br(3);
            *ptr++ = b(3);
            *ptr++ = u.u32[0];
            *ptr++ = u.u32[1];

            for (int i=2; i < 20; i+=2)
                *ptr++ = ldp64(31, i, i + 1, i * 8);
            *ptr++ = ldp64(31, 29, 30, 160);
            *ptr++ = ldp64_postindex(31, 0, 1, 176);
        }

        *ptr++ = add_immed(REG_PC, REG_PC, 2);

        /* Cache flushing is context synchronizing. Stop translating code here */
        *ptr++ = INSN_TO_LE(0xffffffff);
        *ptr++ = INSN_TO_LE(0xfffffff0);

	}
	return ptr;
}

uint32_t *EMIT_CPUSH(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t tmp = 0xff;
    uint8_t tmp2 = 0xff;
    uint8_t tmp3 = 0xff;
    uint8_t tmp4 = 0xff;

    ptr = EMIT_FlushPC(ptr);

    /* Flush data cache? */
    if (opcode & 0x40) {
        /* Get the scope */
        switch (opcode & 0x18) {
            case 0x08:  /* Line */
                tmp = RA_CopyFromM68kRegister(&ptr, 8 + (opcode & 7));
                tmp2 = RA_AllocARMRegister(&ptr);
                tmp3 = RA_AllocARMRegister(&ptr);
                *ptr++ = mov_immed_u8(tmp3, 4);
                *ptr++ = mrs(tmp2, 3, 3, 0, 0, 1); // Get CTR_EL0
                *ptr++ = ubfx(tmp2, tmp2, 16, 4);
                *ptr++ = lslv(tmp2, tmp3, tmp2);
                *ptr++ = sub_immed(tmp2, tmp2, 1);
                *ptr++ = dsb_sy();
                *ptr++ = bic_reg(tmp, tmp, tmp2, LSL, 0);
                *ptr++ = dc_civac(tmp);
                *ptr++ = dsb_sy();
                RA_FreeARMRegister(&ptr, tmp2);
                RA_FreeARMRegister(&ptr, tmp3);
                RA_FreeARMRegister(&ptr, tmp);
            break;
            case 0x10:  /* Page */
                tmp = RA_CopyFromM68kRegister(&ptr, 8 + (opcode & 7));
                tmp2 = RA_AllocARMRegister(&ptr);
                tmp3 = RA_AllocARMRegister(&ptr);
                tmp4 = RA_AllocARMRegister(&ptr);
                *ptr++ = mrs(tmp3, 3, 3, 0, 0, 1); // Get CTR_EL0
                *ptr++ = ubfx(tmp3, tmp3, 16, 4);
                *ptr++ = mov_immed_u16(tmp2, 1024, 0);
                *ptr++ = lsrv(tmp2, tmp2, tmp3);
                *ptr++ = bic_immed(tmp, tmp, 12, 0);
                *ptr++ = mov_immed_u8(tmp4, 4);
                *ptr++ = lslv(tmp4, tmp4, tmp3);
                *ptr++ = dc_civac(tmp);
                *ptr++ = add_reg(tmp, tmp, tmp4, LSL, 0);
                *ptr++ = subs_immed(tmp2, tmp2, 1);
                *ptr++ = b_cc(A64_CC_NE, -3);
                *ptr++ = dsb_sy();
                RA_FreeARMRegister(&ptr, tmp4);
                RA_FreeARMRegister(&ptr, tmp3);
                RA_FreeARMRegister(&ptr, tmp2);
                RA_FreeARMRegister(&ptr, tmp);
            break;
            case 0x18:  /* All */
            {
                union {
                    uint64_t u64;
                    uint32_t u32[2];
                } u;

                u.u64 = (uintptr_t)clear_entire_dcache;

                *ptr++ = stp64_preindex(31, 0, 30, -16);
                *ptr++ = ldr64_pcrel(0, 4);
                *ptr++ = blr(0);
                *ptr++ = ldp64_postindex(31, 0, 30, 16);
                *ptr++ = b(3);
                *ptr++ = u.u32[0];
                *ptr++ = u.u32[1];
            }
            break;
        }
    }
    /* Invalidating instruction cache? */
    if (opcode & 0x80) {
        int8_t off = 0;
        ptr = EMIT_GetOffsetPC(ptr, &off);

        union {
            uint64_t u64;
            uint32_t u32[2];
        } u;
        u.u64 = (uintptr_t)trampoline_icache_invalidate;

        *ptr++ = stp64_preindex(31, 0, 1, -176);
        for (int i=2; i < 20; i+=2)
            *ptr++ = stp64(31, i, i + 1, i * 8);
        *ptr++ = stp64(31, 29, 30, 160);
        if ((opcode & 0x18) == 0x08 || (opcode & 0x18) == 0x10) {
        	uint8_t tmp = RA_MapM68kRegister(&ptr, 8 + (opcode & 7));

         	*ptr++ = mov_reg(0, tmp);
        }
        if (off >= 0)
            *ptr++ = add_immed(1, REG_PC, off);
        else
            *ptr++ = sub_immed(1, REG_PC, -off);

        *ptr++ = adr(2, 4*6);
        *ptr++ = ldr64_pcrel(3, 3);
        *ptr++ = br(3);
        *ptr++ = b(3);
        *ptr++ = u.u32[0];
        *ptr++ = u.u32[1];

        for (int i=2; i < 20; i+=2)
            *ptr++ = ldp64(31, i, i + 1, i * 8);
        *ptr++ = ldp64(31, 29, 30, 160);
        *ptr++ = ldp64_postindex(31, 0, 1, 176);
    }
    *ptr++ = add_immed(REG_PC, REG_PC, 2);

    /* Cache is context synchronizing. Break up here! */
    *ptr++ = INSN_TO_LE(0xffffffff);
    *ptr++ = INSN_TO_LE(0xfffffff0);

	return ptr;
}

uint32_t *EMIT_PFLUSH(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr)__attribute__((alias("EMIT_PTEST")));
uint32_t *EMIT_PTEST(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	*ptr++ = nop();
    (*m68k_ptr)+=1;
    ptr = EMIT_AdvancePC(ptr, 2);
	return ptr;
}

static struct OpcodeDef InsnMMUTable[512] = {
  [0010 ... 0037] = { { EMIT_CINV }, NULL, SR_S, 0, 1, 0, 0},
  [0050 ... 0077] = { { EMIT_CPUSH }, NULL, SR_S, 0, 1, 0, 0},
  [0110 ... 0137] = { { EMIT_CINV }, NULL, SR_S, 0, 1, 0, 0},
  [0150 ... 0177] = { { EMIT_CPUSH }, NULL, SR_S, 0, 1, 0, 0},
  [0210 ... 0237] = { { EMIT_CINV }, NULL, SR_S, 0, 1, 0, 0},
  [0250 ... 0277] = { { EMIT_CPUSH }, NULL, SR_S, 0, 1, 0, 0},
  [0310 ... 0337] = { { EMIT_CINV }, NULL, SR_S, 0, 1, 0, 0},
  [0350 ... 0377] = { { EMIT_CPUSH }, NULL, SR_S, 0, 1, 0, 0},
  [0400 ... 0437] = { { EMIT_PFLUSH }, NULL, SR_S, 0, 1, 0, 0 },
  [0500 ... 0517] = { { EMIT_PTEST }, NULL, SR_S, 0, 1, 0, 0 },
  [0540 ... 0557] = { { EMIT_PTEST }, NULL, SR_S, 0, 1, 0, 0 },
};

uint32_t *EMIT_MMU(uint32_t *ptr, uint16_t **m68k_ptr, uint16_t *insn_consumed) {

  uint16_t opcode = BE16((*m68k_ptr)[0]);
    (*m68k_ptr)++;
    *insn_consumed = 1;

    if (InsnMMUTable[opcode & 0777].od_Emit)
        ptr = InsnMMUTable[opcode & 0777].od_Emit(ptr, opcode, m68k_ptr);

    else {
        ptr = EMIT_FlushPC(ptr);
        ptr = EMIT_InjectDebugString(ptr, "[JIT] opcode %04x at %08x not implemented\n", opcode, *m68k_ptr - 1);
        *ptr++ = svc(0x100);
        *ptr++ = svc(0x101);
        *ptr++ = svc(0x103);
        *ptr++ = (uint32_t)(uintptr_t)(*m68k_ptr - 8);
        *ptr++ = 48;
        ptr = EMIT_Exception(ptr, VECTOR_LINE_F, 0);
        *ptr++ = INSN_TO_LE(0xffffffff);
    }

    return ptr;
}

uint32_t GetSR_LineF(uint16_t opcode)
{
    /* If instruction is in the table, return what flags it needs (shifted 16 bits left) and flags it sets */
    if (InsnMMUTable[opcode & 0777].od_Emit) {
        return (InsnMMUTable[opcode & 0777].od_SRNeeds << 16) | InsnMMUTable[opcode & 0777].od_SRSets << 16;
    }
    /* Instruction not found, i.e. it needs all flags and sets none (ILLEGAL INSTRUCTION exception) */
    else {
        kprintf("Undefined LineF\n");
        return SR_CCR;
    }
}

int M68K_GetLineFLength(uint16_t *insn_stream)
{
    uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)insn_stream);

    int length = 0;
    int need_ea = 0;
    int opsize = 0;

    if(InsnMMUTable[opcode & 0777].od_Emit) {
        length = InsnMMUTable[opcode & 0777].od_BaseLength;
        need_ea = InsnMMUTable[opcode & 0777].od_HasEA;
        opsize = InsnMMUTable[opcode & 0777].od_OpSize;
    }

    if(need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 077, opsize);
    }
    return length;
}
