/*
    Copyright © 2019 Michal Schulz <michal.schulz@gmx.de>
    https://github.com/michalsc

    This Source Code Form is subject to the terms of the
    Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
    with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <stdint.h>
#include <stdlib.h>

#include <stdio.h>
#include "ARM.h"
#include "M68k.h"
#include "RegisterAllocator.h"

uint32_t *EMIT_line6(uint32_t *ptr, uint16_t **m68k_ptr)
{
    uint16_t opcode = BE16((*m68k_ptr)[0]);
    (*m68k_ptr)++;

    /* 01100000xxxxxxxx - BRA */
    if ((opcode & 0xfe00) == 0x6000)
    {
        uint8_t reg = RA_AllocARMRegister(&ptr);
        uint8_t addend = 0;
        uint16_t *bra_rel_ptr = *m68k_ptr;
        int32_t bra_off = 0;

        ptr = EMIT_AdvancePC(ptr, 2);

        /* use 16-bit offset */
        if ((opcode & 0x00ff) == 0x00)
        {
            int8_t pc_off = 0;
            ptr = EMIT_GetOffsetPC(ptr, &pc_off);
            *ptr++ = ldrsh_offset(REG_PC, reg, pc_off);
            addend = 2;
            bra_off = (int16_t)(BE16((*m68k_ptr)[0]));
            (*m68k_ptr)++;
        }
        /* use 32-bit offset */
        else if ((opcode & 0x00ff) == 0xff)
        {
            int8_t pc_off = 0;
            ptr = EMIT_GetOffsetPC(ptr, &pc_off);
            *ptr++ = ldr_offset(REG_PC, reg, pc_off);
            addend = 4;
            bra_off = (int32_t)(BE32(*(uint32_t*)*m68k_ptr));
            (*m68k_ptr) += 2;
        }
        else
        /* otherwise use 8-bit offset */
        {
            *ptr++ = mov_immed_s8(reg, opcode & 0xff);
            bra_rel_ptr = *m68k_ptr;
            bra_off = (int8_t)(opcode & 0xff);
        }

        ptr = EMIT_FlushPC(ptr);

        /* Check if INSN is BSR */
        if (opcode & 0x0100)
        {
            uint8_t sp = RA_MapM68kRegister(&ptr, 15);
            RA_SetDirtyM68kRegister(&ptr, 15);
            if (addend)
            {
                uint8_t tmp = RA_AllocARMRegister(&ptr);
                *ptr++ = add_immed(tmp, REG_PC, addend);
                *ptr++ = str_offset_preindex(sp, tmp, -4);
                RA_FreeARMRegister(&ptr, tmp);
            }
            else
            {
                *ptr++ = str_offset_preindex(sp, REG_PC, -4);
            }
        }

        *ptr++ = add_reg(REG_PC, REG_PC, reg, 0);
        RA_FreeARMRegister(&ptr, reg);

        /* If branch is done within +- 4KB, try to inline it instead of breaking up the translation unit */
        if (bra_off >= -4096 && bra_off <= 4096) {
            *m68k_ptr = (void *)((uintptr_t)bra_rel_ptr + bra_off);
        }
        else
            *ptr++ = INSN_TO_LE(0xffffffff);
    }
    /* 0110ccccxxxxxxxx - Bcc */
    else
    {
        uint32_t *tmpptr;
        uint8_t m68k_condition = (opcode >> 8) & 15;
        uint8_t success_condition = 0;
        uint8_t cond_tmp = 0xff;

        switch (m68k_condition)
        {
            case M_CC_EQ:
                *ptr++ = tst_immed(REG_SR, SR_Z);
                success_condition = ARM_CC_NE;
                break;

            case M_CC_NE:
                *ptr++ = tst_immed(REG_SR, SR_Z);
                success_condition = ARM_CC_EQ;
                break;

            case M_CC_CS:
                *ptr++ = tst_immed(REG_SR, SR_C);
                success_condition = ARM_CC_NE;
                break;

            case M_CC_CC:
                *ptr++ = tst_immed(REG_SR, SR_C);
                success_condition = ARM_CC_EQ;
                break;

            case M_CC_PL:
                *ptr++ = tst_immed(REG_SR, SR_N);
                success_condition = ARM_CC_EQ;
                break;

            case M_CC_MI:
                *ptr++ = tst_immed(REG_SR, SR_N);
                success_condition = ARM_CC_NE;
                break;

            case M_CC_VS:
                *ptr++ = tst_immed(REG_SR, SR_V);
                success_condition = ARM_CC_NE;
                break;

            case M_CC_VC:
                *ptr++ = tst_immed(REG_SR, SR_V);
                success_condition = ARM_CC_EQ;
                break;

            case M_CC_LS:   /* C == 1 || Z == 1 */
                *ptr++ = tst_immed(REG_SR, SR_Z | SR_C);
                success_condition = ARM_CC_NE;
                break;

            case M_CC_HI:   /* C == 0 && Z == 0 */
                *ptr++ = tst_immed(REG_SR, SR_Z);
                *ptr++ = tst_cc_immed(ARM_CC_EQ, REG_SR, SR_C);
                success_condition = ARM_CC_EQ;
                break;

            case M_CC_GE:   /* (N==0 && V==0) || (N==1 && V==1) */
                cond_tmp = RA_AllocARMRegister(&ptr);
                *ptr++ = ands_immed(cond_tmp, REG_SR, SR_N | SR_V); /* Extract N and V, set ARM_CC_EQ if both clear */
                *ptr++ = teq_cc_immed(ARM_CC_NE, cond_tmp, SR_N | SR_V); /* If N and V != 0, perform equality check */
                success_condition = ARM_CC_EQ;
                RA_FreeARMRegister(&ptr, cond_tmp);
                break;

            case M_CC_LT:
                cond_tmp = RA_AllocARMRegister(&ptr);
                *ptr++ = and_immed(cond_tmp, REG_SR, SR_N | SR_V); /* Extract N and V */
                *ptr++ = teq_immed(cond_tmp, SR_N); /* Check N==1 && V==0 */
                *ptr++ = teq_cc_immed(ARM_CC_NE, cond_tmp, SR_V); /* Check N==0 && V==1 */
                success_condition = ARM_CC_EQ;
                RA_FreeARMRegister(&ptr, cond_tmp);
                break;

            case M_CC_GT:
                cond_tmp = RA_AllocARMRegister(&ptr);
                *ptr++ = ands_immed(cond_tmp, REG_SR, SR_N | SR_V | SR_Z); /* Extract Z, N and V, set ARM_CC_EQ if both clear */
                *ptr++ = teq_cc_immed(ARM_CC_NE, cond_tmp, SR_N | SR_V); /* If above fails, check if Z==0, N==1 and V==1 */
                success_condition = ARM_CC_EQ;
                RA_FreeARMRegister(&ptr, cond_tmp);
                break;

            case M_CC_LE:
                cond_tmp = RA_AllocARMRegister(&ptr);
                *ptr++ = and_immed(cond_tmp, REG_SR, SR_N | SR_V); /* Extract N and V, set ARM_CC_EQ if both clear */
                *ptr++ = teq_immed(cond_tmp, SR_N); /* Check N==1 && V==0 */
                *ptr++ = teq_cc_immed(ARM_CC_NE, cond_tmp, SR_V); /* Check N==0 && V==1 */
                *ptr++ = and_cc_immed(ARM_CC_NE, cond_tmp, REG_SR, SR_Z); /* If failed, extract Z flag */
                *ptr++ = teq_cc_immed(ARM_CC_NE, cond_tmp, SR_Z); /* Check if Z is set */
                success_condition = ARM_CC_EQ;
                RA_FreeARMRegister(&ptr, cond_tmp);
                break;

            default:
                printf("Default CC called! Can't be!\n");
                *ptr++ = udf(0x0bcc);
                break;
        }
        ptr = EMIT_FlushPC(ptr);

        /* Adjust PC accordingly */
        if ((opcode & 0x00ff) == 0x00)
        {
            *ptr++ = add_cc_immed(success_condition ^ 1, REG_PC, REG_PC, 4);
        }
        /* use 32-bit offset */
        else if ((opcode & 0x00ff) == 0xff)
        {
            *ptr++ = add_cc_immed(success_condition ^ 1, REG_PC, REG_PC, 6);
        }
        else
        /* otherwise use 8-bit offset */
        {
            *ptr++ = add_cc_immed(success_condition ^ 1, REG_PC, REG_PC, 2);
        }

        /* Next jump to skip the condition - invert bit 0 of the condition code here! */
        tmpptr = ptr;

        uint8_t reg = RA_AllocARMRegister(&ptr);

        *ptr++ = b_cc(success_condition ^ 1, 2);

        *ptr++ = add_immed(REG_PC, REG_PC, 2);

        /* use 16-bit offset */
        if ((opcode & 0x00ff) == 0x00)
        {
            *ptr++ = ldrsh_offset(REG_PC, reg, 0);
            (*m68k_ptr)++;
        }
        /* use 32-bit offset */
        else if ((opcode & 0x00ff) == 0xff)
        {
            *ptr++ = ldr_offset(REG_PC, reg, 0);
            (*m68k_ptr) += 2;
        }
        else
        /* otherwise use 8-bit offset */
        {
            *ptr++ = mov_immed_s8(reg, opcode & 0xff);
        }

        *ptr++ = add_reg(REG_PC, REG_PC, reg, 0);
        RA_FreeARMRegister(&ptr, reg);
        *ptr++ = (uint32_t)tmpptr;
        *ptr++ = 1;
        *ptr++ = INSN_TO_LE(0xfffffffe);
    }

    return ptr;
}
