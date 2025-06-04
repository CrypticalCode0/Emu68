#include <stdint.h>
#include <sys/types.h>
#include "../include/support.h"
#include "../include/M68k.h"
#include "../include/RegisterAllocator.h"
#include "../include/cache.h"

#include "M68k_CC.c"
#include "M68k_MMU.c"
#include "M68k_FPU.c"

uint32_t *EMIT_cpOP(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint16_t opcode2 = cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[1]);

	uint8_t cp = (opcode >> 9) & 7;

	switch(cp) {
		case cp_MMU:
			EMIT_MMU(*ptr, opcode, **m68k_ptr);
		return ptr;
		case cp_FPU:
			EMIT_FPU(*ptr, opcode, **m68k_ptr);
		return ptr;
		case cp_MOVE16:
			if(opcode <= 0047) {

				uint8_t buf1 = RA_AllocARMRegister(&ptr);
				uint8_t buf2 = RA_AllocARMRegister(&ptr);
				uint8_t src = RA_MapM68kRegister(&ptr, 8 + (opcode & 7));
				uint8_t dst = RA_MapM68kRegister(&ptr, 8 + ((opcode2 >> 12) & 7));

#if 0
				if (dst != src) {
		 		   *ptr++ = ldp64_postindex(src, buf1, buf2, 16);
				} else {
					*ptr++ = ldp64(src, buf1, buf2, 0);
				}
				*ptr++ = stp64_postindex(dst, buf1, buf2, 16);
#else
				uint8_t aligned_src = RA_AllocARMRegister(&ptr);
				uint8_t aligned_dst = RA_AllocARMRegister(&ptr);

				*ptr++ = bic_immed(aligned_src, src, 4, 0);
				*ptr++ = bic_immed(aligned_dst, dst, 4, 0);
				*ptr++ = ldp64(aligned_src, buf1, buf2, 0);
				*ptr++ = add_immed(src, src, 16);
				*ptr++ = stp64(aligned_dst, buf1, buf2, 0);

				// Update dst only if it is not the same as src!
				if (dst != src) {
					*ptr++ = add_immed(dst, dst, 16);
			   	}

			   	RA_FreeARMRegister(&ptr, aligned_src);
				RA_FreeARMRegister(&ptr, aligned_dst);
#endif
	 	   		RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));
			 	RA_SetDirtyM68kRegister(&ptr, 8 + ((opcode2 >> 12) & 7));

			 	RA_FreeARMRegister(&ptr, buf1);
				RA_FreeARMRegister(&ptr, buf2);

				(*m68k_ptr)+=2;
			   	ptr = EMIT_AdvancePC(ptr, 4);

			   	return ptr;
   			}
			else {

				uint8_t aligned_reg = RA_AllocARMRegister(&ptr);
			   	uint8_t aligned_mem = RA_AllocARMRegister(&ptr);
			   	uint8_t buf1 = RA_AllocARMRegister(&ptr);
			   	uint8_t buf2 = RA_AllocARMRegister(&ptr);
				uint8_t reg = RA_MapM68kRegister(&ptr, 8 + (opcode & 7));
				uint32_t mem = (cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[1]) << 16) | cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[2]);

				/* Align memory pointer */
				mem &= 0xfffffff0;
			   	*ptr++ = movw_immed_u16(aligned_mem, mem & 0xffff);
			   	if (mem & 0xffff0000)
				   	*ptr++ = movt_immed_u16(aligned_mem, mem >> 16);

			   	*ptr++ = bic_immed(aligned_reg, reg, 4, 0);

				if (opcode & 8) {
				   	*ptr++ = ldp64(aligned_mem, buf1, buf2, 0);
				   	*ptr++ = stp64(aligned_reg, buf1, buf2, 0);
				}
				else {
				   	*ptr++ = ldp64(aligned_reg, buf1, buf2, 0);
				   	*ptr++ = stp64(aligned_mem, buf1, buf2, 0);
				}

				if (!(opcode & 0x10)) {
				   	*ptr++ = add_immed(reg, reg, 16);
				   	RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));
				}

				RA_FreeARMRegister(&ptr, aligned_reg);
				RA_FreeARMRegister(&ptr, aligned_mem);
				RA_FreeARMRegister(&ptr, buf1);
				RA_FreeARMRegister(&ptr, buf2);

				(*m68k_ptr)+=3;
				ptr = EMIT_AdvancePC(ptr, 6);
		   	}
			return ptr;
	}
}

uint32_t *EMIT_FScc_reg(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr)__attribute__((alias("EMIT_FScc")));
uint32_t *EMIT_FScc(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;

	if(cp == cp_FPU) {
		static int shown = 0;
		if (!shown) {
			kprintf("FScc\n");
			shown = 1;
		}

		uint16_t opcode2 = cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[1]);

		uint8_t fpsr = RA_GetFPSR(&ptr);
		uint8_t predicate = opcode2 & 077; //the condition code predicate is 6 bits
		uint8_t ext_count = 0;
		uint8_t success_condition = 0;

		EMIT_TestFPUCondition(*fpsr, predicate);

		if ((opcode & 070) == 0) {
			/* Dx case */
			uint8_t dest = RA_MapM68kRegister(&ptr, opcode & 7);
			RA_SetDirtyM68kRegister(&ptr, opcode & 7);

			switch(predicate & 0x0f) {
				case F_CC_F:
					*ptr++ = bic_immed(dest, dest, 8, 0);
					break;
				case F_CC_T:
					*ptr++ = orr_immed(dest, dest, 8, 0);
					break;
				default:
					uint8_t tmp = RA_AllocARMRegister(&ptr);

					*ptr++ = csetm(tmp, success_condition);
					*ptr++ = bfi(dest, tmp, 0, 8);
					RA_FreeARMRegister(&ptr, tmp);
					break;
			}
		}
		else {
			/* Load Effective Address */
			uint8_t tmp = RA_AllocARMRegister(&ptr);

			switch(predicate & 0x0f) {
				case F_CC_F:
					*ptr++ = mov_immed_u16(tmp, 0, 0);
					break;
				case F_CC_T:
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
	return ptr;
}


uint32_t *EMIT_FDBcc(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;

	if(cp == cp_FPU) {
		ptr = EMIT_InjectDebugString(ptr, "[JIT] FDBcc at %08x not implemented\n", *m68k_ptr - 1);
		ptr = EMIT_InjectPrintContext(ptr);
		*ptr++ = udf(opcode);
	}
	return ptr;
}


uint32_t *EMIT_FTRAPcc(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;

	if(cp == cp_FPU) {
		ptr = EMIT_InjectDebugString(ptr, "[JIT] FTRAPcc at %08x not implemented\n", *m68k_ptr - 1);
		ptr = EMIT_InjectPrintContext(ptr);
		*ptr++ = udf(opcode);
	}
	return ptr;
}


uint32_t *EMIT_FNOP(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;
	uint8_t ext_count = 1;

	if(cp == cp_FPU) {

		static int shown = 0;
		if (!shown) {
			kprintf("FNOP\n");
			shown = 1;
		}

		ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
		(*m68k_ptr) += ext_count;
		ptr = EMIT_FlushPC(ptr);
	}
	return ptr;
}


uint32_t *EMIT_FBcc(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;

	if(cp == cp_FPU) {
		static int shown = 0;
		if (!shown) {
			kprintf("FBcc\n");
			shown = 1;
		}

		uint8_t fpsr = RA_GetFPSR(&ptr);
		uint8_t predicate = opcode & 077;
		uint8_t success_condition = 0;
		uint32_t *tmpptr;

		EMIT_TestFPUCondition(*fpsr, predicate);

		int8_t local_pc_off = 2;

		ptr = EMIT_GetOffsetPC(ptr, &local_pc_off);
		ptr = EMIT_ResetOffsetPC(ptr);

		uint8_t reg = RA_AllocARMRegister(&ptr);

		intptr_t branch_target = (intptr_t)(*m68k_ptr);
		intptr_t branch_offset = 0;

		/* use 16-bit offset */
		if ((opcode & 0x0040) == 0x0000) {
			branch_offset = (int16_t)cache_read_16(ICACHE, (uintptr_t)&(*(*m68k_ptr)++));
		}
		/* use 32-bit offset */
		else {
			uint16_t lo16, hi16;
			hi16 = cache_read_16(ICACHE, (uintptr_t)&(*(*m68k_ptr)++));
			lo16 = cache_read_16(ICACHE, (uintptr_t)&(*(*m68k_ptr)++));
			branch_offset = lo16 | (hi16 << 16);
		}

		branch_offset += local_pc_off;

		uint8_t pc_yes = RA_AllocARMRegister(&ptr);
		uint8_t pc_no = RA_AllocARMRegister(&ptr);

		if (branch_offset > 0 && branch_offset < 4096)
			*ptr++ = add_immed(pc_yes, REG_PC, branch_offset);
		else if (branch_offset > -4096 && branch_offset < 0)
			*ptr++ = sub_immed(pc_yes, REG_PC, -branch_offset);
		else if (branch_offset != 0) {
			*ptr++ = movw_immed_u16(reg, branch_offset);
			if ((branch_offset >> 16) & 0xffff)
				*ptr++ = movt_immed_u16(reg, (branch_offset >> 16) & 0xffff);
			*ptr++ = add_reg(pc_yes, REG_PC, reg, LSL, 0);
		}
		else { *ptr++ = mov_reg(pc_yes, REG_PC); }

		branch_target += branch_offset - local_pc_off;

		int16_t local_pc_off_16 = local_pc_off - 2;

		/* Adjust PC accordingly */
		if ((opcode & 0x0040) == 0x0000) {
			local_pc_off_16 += 4;
		}
		/* use 32-bit offset */
		else {
			local_pc_off_16 += 6;
		}

		if (local_pc_off_16 > 0 && local_pc_off_16 < 255)
			*ptr++ = add_immed(pc_no, REG_PC, local_pc_off_16);
		else if (local_pc_off_16 > -256 && local_pc_off_16 < 0)
			*ptr++ = sub_immed(pc_no, REG_PC, -local_pc_off_16);
		else if (local_pc_off_16 != 0) {
			*ptr++ = movw_immed_u16(reg, local_pc_off_16);
			if ((local_pc_off_16 >> 16) & 0xffff)
				*ptr++ = movt_immed_u16(reg, local_pc_off_16 >> 16);
			*ptr++ = add_reg(pc_no, REG_PC, reg, LSL, 0);
		}
		*ptr++ = csel(REG_PC, pc_yes, pc_no, success_condition);
		RA_FreeARMRegister(&ptr, pc_yes);
		RA_FreeARMRegister(&ptr, pc_no);
		tmpptr = ptr;
#if EMU68_DEF_BRANCH_AUTO
		if(
			branch_target < (intptr_t)*m68k_ptr &&
			((intptr_t)*m68k_ptr - branch_target) < EMU68_DEF_BRANCH_AUTO_RANGE
		)
			*ptr++ = b_cc(success_condition, 1);
		else
			*ptr++ = b_cc(success_condition^1, 1);
#else
#if EMU68_DEF_BRANCH_TAKEN
		*ptr++ = b_cc(success_condition, 1);
#else
		*ptr++ = b_cc(success_condition^1, 1);
#endif
#endif

#if EMU68_DEF_BRANCH_AUTO
		if(
			branch_target < (intptr_t)*m68k_ptr &&
			((intptr_t)*m68k_ptr - branch_target) < EMU68_DEF_BRANCH_AUTO_RANGE
		)
			*m68k_ptr = (uint16_t *)branch_target;
#else
#if EMU68_DEF_BRANCH_TAKEN
		*m68k_ptr = (uint16_t *)branch_target;
#endif
#endif
		RA_FreeARMRegister(&ptr, reg);
		*ptr++ = (uint32_t)(uintptr_t)tmpptr;
		*ptr++ = 1;
		*ptr++ = branch_target;
		*ptr++ = INSN_TO_LE(0xfffffffe);
	}
	return ptr;
}


uint32_t *EMIT_FSAVE(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;
	uint8_t sr = GetSR_LineF(opcode) & SR_S;
	uint8_t ext_count = 0;

	if(cp == cp_FPU && (sr  SR_S)) {
  		static int shown = 0;
		if (!shown) {
			kprintf("FSAVE\n");
			shown = 1;
		}
		uint8_t tmp = RA_AllocARMRegister(&ptr);

		ext_count = 0;

		*ptr++ = mov_immed_u16(tmp, 0x4100, 1);
		ptr = EMIT_StoreToEffectiveAddress(ptr, 4, &tmp, opcode & 0x3f, *m68k_ptr, &ext_count, 0);

		RA_FreeARMRegister(&ptr, tmp);

		ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
		(*m68k_ptr) += ext_count;
	}
	return ptr;
}

uint32_t *EMIT_FRESTORE(uint32_t *ptr, uint16_t opcode, uint16_t **m68k_ptr) {

	uint8_t cp = (opcode >> 9) & 7;

	if(cp == cp_FPU){
		static int shown = 0;
		if (!shown) {
			kprintf("FRESTORE\n");
			shown = 1;
		}
		uint8_t tmp = -1;
		uint32_t *tmp_ptr;
		uint8_t fpcr = RA_ModifyFPCR(&ptr);
		uint8_t fpsr = RA_ModifyFPSR(&ptr);
		uint8_t ext_count = 0;

		ptr = EMIT_LoadFromEffectiveAddress(ptr, 4, &tmp, opcode & 0x3f, *m68k_ptr, &ext_count, 0, NULL);

		// If Postincrement mode, eventually skip rest of the frame if IDLE was fetched
		if ((opcode & 0x38) == 0x18)
		{
			uint8_t An = RA_MapM68kRegister(&ptr, 8 + (opcode & 7));
			uint8_t tmp2 = RA_AllocARMRegister(&ptr);
			*ptr++ = tst_immed(tmp, 8, 8);
			*ptr++ = b_cc(A64_CC_EQ, 5);
			*ptr++ = ubfx(tmp2, tmp, 16, 8);
			*ptr++ = cmp_immed(tmp2, 0x18);
			*ptr++ = b_cc(A64_CC_NE, 2);
			*ptr++ = add_immed(An, An, 28 - 4);
			RA_FreeARMRegister(&ptr, tmp2);
		}

		// In case of NULL frame, reset FPU to vanilla state
		*ptr++ = tst_immed(tmp, 8, 8);
		tmp_ptr = ptr;
		*ptr++ = b_cc(A64_CC_NE, 0);

		uint8_t tmp_nan = RA_AllocARMRegister(&ptr);
		*ptr++ = movn64_immed_u16(tmp_nan, 0x8000, 3);

		for (int fp = 8; fp < 16; fp++)
			*ptr++ = mov_reg_to_simd(fp, TS_D, 0, tmp_nan); //fmov_0(8);

		RA_FreeARMRegister(&ptr, tmp_nan);

		*ptr++ = mov_immed_u16(fpcr, 0, 0);
		*ptr++ = mov_immed_u16(fpsr, 0, 0);

		*ptr++ = get_fpcr(tmp);
		*ptr++ = bic_immed(tmp, tmp, 2, 32 - 22);
		*ptr++ = set_fpcr(tmp);
		*ptr++ = mov_reg_to_simd(29, TS_S, 1, 31);

		*tmp_ptr = b_cc(A64_CC_NE, ptr - tmp_ptr);

		RA_FreeARMRegister(&ptr, tmp);

		ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
		(*m68k_ptr) += ext_count;
		ptr = EMIT_FlushPC(ptr);
	}
	return ptr;
}

static struct FPUOpcodeDef InsnTable[512] = {
  /* the whole shabam! */
  [0000 ... 0074] = { { EMIT_cpOP }, NULL, 0, FPCC, 2, 0, 0},
  /* cc */
  [0100 ... 0107] = { { EMIT_FScc_reg }, NULL, FPCC, 0, 2, 0, 1},
  [0110 ... 0117] = { { EMIT_FDBcc }, NULL, FPCC, 0, 3, 0, 0},
  [0120 ... 0147] = { { EMIT_FScc }, NULL, FPCC, 0, 2, 0, 1},
  [0150 ... 0171] = { { EMIT_FScc }, NULL, FPCC, 0, 2, 1, 1},
  [0172]		  = { { EMIT_FTRAPcc }, NULL, FPCC, 3, 0, 0},
  [0173]		  = { { EMIT_FTRAPcc }, NULL, FPCC, 4, 0, 0},
  [0174]		  = { { EMIT_FTRAPcc }, NULL, FPCC, 2, 0, 0},
  [0200]		  = { { EMIT_FNOP }, NULL , 0, 0, 2, 0, 0},
  [0201 ... 0237] = { { EMIT_FBcc }, NULL, FPCC, 0, 2, 0, 0},
  [0300 ... 0337] = { { EMIT_FBcc }, NULL, FPCC, 0, 3, 0, 0},
  /* cp */
  [0420 ... 0427] = { { EMIT_FSAVE }, NULL, SR_S, 0, 1, 0, 0 },
  [0440 ... 0447] = { { EMIT_FSAVE }, NULL, SR_S, 0, 1, 0, 0 },
  [0450 ... 0471] = { { EMIT_FSAVE }, NULL, SR_S, 0, 1, 1, 0 },
  [0520 ... 0537] = { { EMIT_FRESTORE }, NULL, SR_S, 0, 1, 0, 0 },
  [0550 ... 0573] = { { EMIT_FRESTORE }, NULL, SR_S, 0, 1, 1, 0 },
};

uint32_t *EMIT_lineF(uint32_t *ptr, uint16_t **m68k_ptr, uint16_t *insn_consumed) {

	uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[0]);
	(*m68k_ptr)++;
	*insn_consumed = 1;

	if(InsnTable[opcode & 0777].od_Emit) {
		ptr = InsnTable[opcode & 0777].od_Emit(ptr, opcode, m68k_ptr);
	}
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

int M68K_GetLineFLength(uint16_t *insn_stream) {

	uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)insn_stream);

	uint8_t length = 0;
	uint8_t need_ea = 0;
	uint8_t opsize = 0;

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
