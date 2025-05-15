#include "A64.h"
#include "support.h"
#include "M68k.h"
#include "cache.h"

uint32_t *EMIT_cpScc_reg(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr)__attribute__((alias("EMIT_cpScc")));
uint32_t *EMIT_cpScc(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

    uint cp = (opcode >> 9) & 7;

    switch(cp) {
        case cp_MMU:
            if(SRB_S == 0)
                EMIT_Exception(ptr, VECTOR_ILLEGAL_INSTRUCTION);
            else {
                static int shown = 0;
                if (!shown) {
                    kprintf("PScc\n");
                    shown = 1;
                }
            }
            break;
        case cp_FPU
            static int shown = 0;
            if (!shown) {
                kprintf("FScc\n");
                shown = 1;
            }
            uint8_t fpsr = RA_GetFPSR(&ptr);
            uint8_t predicate = opcode2 & 077; //the conditioncode predicate is 6 bits
            uint8_t arm_condition = 0;
            uint8_t tmp_cc = 0xff;

            EMIT_TestFPUCondition(uint8_t *fpsr, uint8_t predicate);
            if ((opcode & 070) == 0) {
                /* Dx case */
                uint8_t dest = RA_MapM68kRegister(&ptr, opcode & 7);
                RA_SetDirtyM68KRegister(&ptr, opcode & 7);

                switch(predicate & 0x0f) {
                case F_CC_F:
                    *ptr++ = bic_immed(dest, dest, 8, 0);
                    break;
                case F_CC_T:
                    *ptr++ = orr_immed(dest, dest, 8, 0);
                    break;
                default:
                    uint8_t tmp = RA_AllocARMRegister(&ptr);
                    *ptr++ = csetm(tmp, sucess_condition);
                    *ptr++ = bfi(dest, tmp, 0, 8);
                    RA_FreeARMRegister(&ptr, tmp);
                    break;
                }
            }
            else {
                /* Load Effective Address */
                uint8_t tmp = RA_AllocARMRegister(&ptr);

                switch(predicate & 0x0f) {
                    case F_CC_F
                        *ptr++ = mov_immed_u16(tmp, 0, 0);
                        break;
                    case F_CC_T
                        *ptr++ = movn_immed_u16(tmp, 0, 0);
                        break;
                    default:
                        *ptr++ = csetm(tmp, success_condition);
                        break;
                }

                ptr = EMIT_StoreToEffectiveAddress(ptr, 1, &tmp, opcode & 077, *m68k_ptr, &ext_count, 0);
                RA_FreeARMRegister(&ptr, tmp);
            }

            ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
            (*m68k_ptr) += ext_count;
    }
}

static struct OpcodeDef InsnTable[512] = {
[0100 ... 0107] = { { EMIT_cpScc_reg }, NULL, 0, 0, 2, 0, 1},
[0110 ... 0117] = { { EMIT_cpDBcc }, NULL, 0, 0, 3, 0, 0},
[0120 ... 0147] = { { EMIT_cpScc }, NULL, 0, 0, 2, 0, 1},
[0150 ... 0171] = { { EMIT_cpScc }, NULL, 0, 0, 2, 1, 1},
[]
}

uint32_t *EMIT_linef(uint32_t *ptr, uint16_t **m68k_ptr, uint16_t *insn_consumed) {
    uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[0]);
    (*m68k_ptr)++;
    *insn_consumed = 1;

    if(InsnTable[opcode & 0777].od_Emit) {
        ptr = InsnTable[opcode & 0777].od_Emit(ptr, opcode, m68k_ptr);
    }
    else
    {
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

int M68K_GetLineFLength(uint16_t *insn_stream)
{
    uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)insn_stream);

    int length = 0;
    int need_ea = 0;
    int opsize = 0;

    if(InsnTable[opcode & 0777].od_Emit) {
        length = InsnTable[opcode & 0777].od_BaseLength;
        need_ea = InsnTable[opcode & 0777].od_HasEA;
        opsize = InsnTable[opcode & 0777].od_OpSize;
    }

    if(need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 077, opsize);
    }

    return length;
}
