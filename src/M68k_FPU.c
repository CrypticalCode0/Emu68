
/****************************************************/
/* FPU Instruction opcodes and condition fields.	*/
/*													*/
/* This part holds all tables,						*/
/* Header according to the following format.		*/
/* Table Name.										*/
/*													*/
/****************************************************/

#include <stdint.h>
#include "../include/A64.h"
#include "../include/M68k.h"
#include "../include/support.h"
#include "../include/cache.h"
#include "math/libm.h"

uint64_t Load96bit(uintptr_t __ignore, uintptr_t base);
uint64_t Store96bit(uintptr_t value, uintptr_t base);

extern uint8_t reg_Load96;
extern uint8_t reg_Save96;
extern uint32_t val_FPIAR;

/* Helper functions*/
uint32_t * get_Load96(uint32_t *ptr)
{
	if (reg_Load96 == 0xff) {
		reg_Load96 = RA_AllocARMRegister(&ptr);
		uint32_t val = (uintptr_t)Load96bit;

		*ptr++ = mov_immed_u16(reg_Load96, val & 0xffff, 0);
		*ptr++ = movk_immed_u16(reg_Load96, val >> 16, 1);
		*ptr++ = orr64_immed(reg_Load96, reg_Load96, 25, 25, 1);
	}
	return ptr;
}

uint32_t * get_Save96(uint32_t *ptr)
{
	if (reg_Save96 == 0xff) {
		reg_Save96 = RA_AllocARMRegister(&ptr);
		uint32_t val = (uintptr_t)Store96bit;

		*ptr++ = mov_immed_u16(reg_Save96, val & 0xffff, 0);
		*ptr++ = movk_immed_u16(reg_Save96, val >> 16, 1);
		*ptr++ = orr64_immed(reg_Save96, reg_Save96, 25, 25, 1);
	}
	return ptr;
}

typedef union
{
	uint8_t  c[12];
	uint32_t i[3];
} packed_t;

double my_pow10(int exp);
int my_log10(double v);

double PackedToDouble(packed_t value)
{
	double ret = 0.0;
	int exp = 0;
	uint64_t integer = 0;

	for (int i=4; i < 12; i++) {
		integer = integer * 10 + (value.c[i] >> 4);
		integer = integer * 10 + (value.c[i] & 0x0f);
	}

	ret = (double)(value.c[3] & 0x0f) + (double)integer / 1e16;
	exp = 100 * (value.c[0] & 0x0f) + 10 * (value.c[1] >> 4) + (value.c[1] & 0x0f);

	if (value.c[0] & 0x80)
		ret = -ret;
	if (value.c[0] & 0x40)
		exp = -exp;

	ret = ret * exp10(exp);

	return ret;
}

packed_t DoubleToPacked(double value, int k)
{
	k = ((int8_t)k << 1) >> 1;

	int exp = 0;
	packed_t ret;
	uint8_t c;

	ret.i[0] = 0;
	ret.i[1] = 0;
	ret.i[2] = 0;

	int prec = k > 0 ? k - 1 : 4 - k;

	if (value < 0) {
		value = -value;
		ret.c[0] |= 0x80;
	}

	if (prec > 16)
		prec = 16;

	exp = my_log10(value);
	value /= my_pow10(exp);

	c = (int)value;
	ret.c[3] = c;

	for(int i=0; i < prec; i++) {

		value = (value - c) * 10;
		c = (int)value;
		if (i & 1)
			ret.c[4 + (i >> 1)] |= c;
		else
			ret.c[4 + (i >> 1)] |= c << 4;
	}

	if (exp < 0) {
		exp = -exp;
		ret.c[0] |= 0x40;
	}

	ret.c[1] = exp % 10;
	exp /= 10;
	ret.c[1] |= (exp % 10) << 4;
	exp /= 10;
	ret.c[0] |= exp % 10;
	exp /= 10;

	if (exp != 0) {
		ret.c[2] |= (exp) << 4;
	}

	return ret;
}

/*
	Returns reminder of absolute double number divided by 2, i.e. for any number it calculates result
	of number mod 2. Used by trigonometric functions
*/
double TrimDoubleRange(double a)
{
	union {
		uint64_t i;
		uint32_t i32[2];
		double d;
	} n, out;

	n.d = a;

	uint32_t exp = (n.i32[0] >> 20) & 0x7ff;
	uint64_t man = n.i & 0x000fffffffffffffULL;

	if (man && exp > 0x3ff && exp < (0x3ff + 52)) {

		man = (man << (exp - 0x3ff)) & 0x001fffffffffffffULL;
		exp = 0x3ff;

		if (man) {
			int d = __builtin_clzll(man) - 11;

			if (d) {
				man = (man << (d)) & 0x000fffffffffffffULL;
				exp = exp - d;
			}
		}
		else
			exp=0;
	}
	else if (!man && exp > 0x3ff)
		exp = 0;

	out.i = man & ~0x0010000000000000ULL;
	out.i32[0] |= exp << 20;

	return out.d;
}

enum FPUOpSize {
	SIZE_L = 0,
	SIZE_S = 1,
	SIZE_X = 2,
	SIZE_P = 3,
	SIZE_W = 4,
	SIZE_D = 5,
	SIZE_B = 6,
	SIZE_Pdyn = 7,
};

uint8_t FPUDataSize[] = {
	[SIZE_L] = 4,
	[SIZE_S] = 4,
	[SIZE_X] = 12,
	[SIZE_P] = 12,
	[SIZE_Pdyn] = 12,
	[SIZE_W] = 2,
	[SIZE_D] = 8,
	[SIZE_B] = 1
};

int FPSR_Update_Needed(uint16_t *ptr, int level) {

	int cnt = 0;

	while((cache_read_16(ICACHE, (uintptr_t)ptr) & 0xfe00) != 0xf200) {
		if (cnt++ > 15)
			return 1;
		if (M68K_IsBranch(ptr))
			return 1;

		int len = M68K_GetINSNLength(ptr);

		if (len <= 0)
			return 1;
		ptr += len;
	}

	uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)&ptr[0]);
	uint16_t opcode2 = cache_read_16(ICACHE, (uintptr_t)&ptr[1]);

	/* In case of FNOP check subsequent instruction */
	if (opcode == 0xf280 && opcode2 == 0x0000) {
		if (level == 5)
			return 1;
		else
			return FPSR_Update_Needed(ptr + 2, level + 1);
	}

	/*
		Update FPSR condition codes only if subsequent FPU instruction
		is one of following: FBcc, FDBcc, FMOVEM, FScc, FTRAPcc
	*/

	if ((opcode & 0xff80) == 0xf280)	/* FBcc */
		return 1;
	if ((opcode & 0xfff8) == 0xf248 && (opcode2 & 0xffc0) == 0) /* FDBcc */
		return 1;
	if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xc700) == 0xc000) /* FMOVEM */
		return 1;
	if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xc3ff) == 0x8000) /* FMOVEM special */
		return 1;
	if ((opcode & 0xffc0) == 0xf240 && (opcode2 & 0xffc0) == 0) /* FScc */
		return 1;
	if ((opcode & 0xfff8) == 0xf278 && (opcode2 & 0xffc0) == 0) /* FTRAPcc */
		return 1;
	if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xe000) == 0x6000) /* FMOVE to MEM */
		return 1;
	if ((opcode & 0xffc0) == 0xf340) /* FRESTORE */
		return 1;
	if ((opcode & 0xffc0) == 0xf300) /* FSAVE */
		return 1;

	return 0;
}

/* Allocates FPU register and fetches data according to the R/M field of the FPU opcode */
uint32_t *FPU_FetchData(uint32_t *ptr, uint16_t **m68k_ptr, uint8_t *reg, uint16_t opcode,
		uint16_t opcode2, uint8_t *ext_count, uint8_t single)
{
	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	(void)single;

	/* IF R/M is zero, then source identifier is FPU reg number. */
	if ((opcode2 & 0x4000) == 0)
		*reg = RA_MapFPURegister(&ptr, (opcode2 >> 10) & 7);
	else {
		/*
			R/M was set to 1, the source is defined by EA stored in first part of the
			opcode. Source identifier specifies the data length.

			Get EA, eventually (in case of mode 000 - Dn) perform simple data transfer.
			Otherwise get address from EA and fetch data here
		*/

		/* The regtister was not yet assigned? assign it to a temporary reg now */
		if (*reg == 0xff)
			*reg = RA_AllocFPURegister(&ptr);

		uint8_t ea = opcode & 0x3f;
		enum FPUOpSize size = (opcode2 >> 10) & 7;
		int32_t imm_offset = 0;

		/* Case 1: mode 000 - Dn */
		if ((ea & 0x38) == 0) {

			uint8_t int_reg = 0xff;

			switch (size) {
				/* Single - move to single half of the reg, convert to double */
				case SIZE_S:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 4, &int_reg, ea, *m68k_ptr, ext_count, 1, NULL);
					*ptr++ = fmsr(*reg, int_reg);
					*ptr++ = fcvtds(*reg, *reg);
					RA_FreeARMRegister(&ptr, int_reg);
					break;

				case SIZE_L:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 4, &int_reg, ea, *m68k_ptr, ext_count, 1, NULL);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					RA_FreeARMRegister(&ptr, int_reg);
					break;

				case SIZE_W:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 0x80 | 2, &int_reg, ea, *m68k_ptr, ext_count, 1, NULL);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					RA_FreeARMRegister(&ptr, int_reg);
					break;

				case SIZE_B:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 0x80 | 1, &int_reg, ea, *m68k_ptr, ext_count, 1, NULL);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					RA_FreeARMRegister(&ptr, int_reg);
					break;

				default:
					kprintf("[JIT] LineF: wrong argument size %d for Dn access at %08x\n", (int)size, *m68k_ptr-1);
			}
		}
		/* Case 2: mode 111:100 - immediate */
		else if (ea == 0x3c) {
			/* Fetch data *or* pointer to data into int_reg */
			uint8_t int_reg = 0xff;
			int not_yet_done = 0;

			switch (size) {

				case SIZE_S: {
					int8_t off = 4;
					ptr = EMIT_GetOffsetPC(ptr, &off);
					*ptr++ = flds(*reg, REG_PC, off);
					*ptr++ = fcvtds(*reg, *reg);
					*ext_count += 2;
					break;
				}
				case SIZE_L:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 4, &int_reg, ea, *m68k_ptr, ext_count, 0, NULL);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					break;

				case SIZE_W: {
					int_reg = RA_AllocARMRegister(&ptr);
					int16_t imm = (int16_t)cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[1]);
					*ptr++ = movw_immed_u16(int_reg, imm & 0xffff);
					if (imm < 0)
						*ptr++ = movt_immed_u16(int_reg, 0xffff);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					*ext_count += 1;
					break;
				}

				case SIZE_B: {
					int_reg = RA_AllocARMRegister(&ptr);
					int8_t imm = (int8_t)cache_read_16(ICACHE, (uintptr_t)&(*m68k_ptr)[1]);
					*ptr++ = mov_immed_s8(int_reg, imm);
					*ptr++ = scvtf_32toD(*reg, int_reg);
					*ext_count += 1;
					break;
				}

				case SIZE_D: {
					int8_t off = 4;
					ptr = EMIT_GetOffsetPC(ptr, &off);
					*ptr++ = fldd(*reg, REG_PC, off);
					*ext_count += 4;
					break;
				}

				default:
					ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &int_reg, ea, *m68k_ptr, ext_count, 0, NULL);
					not_yet_done = 1;
					break;
			}

			/* if data not yet in the reg, use the address to load it into FPU register */
			if (not_yet_done) {
				switch(size) {

					case SIZE_D:
						*ptr++ = fldd(*reg, int_reg, 0);
						*ext_count += 4;
						break;

					case SIZE_X:
						ptr = get_Load96(ptr);
						*ptr++ = str64_offset_preindex(31, 30, -16);
						*ptr++ = mov_reg(1, int_reg);
						*ptr++ = blr(reg_Load96);
						*ptr++ = mov_reg_to_simd(*reg, TS_D, 0, 0);
						*ptr++ = ldr64_offset_postindex(31, 30, 16);
						*ext_count += 6;
						break;

					case SIZE_P:
						u.u64 = (uintptr_t)PackedToDouble;

						ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 7));

						*ptr++ = ldr64_offset(int_reg, 0, 0);
						*ptr++ = ldr64_offset(int_reg, 1, 8);
						*ptr++ = adr(30, 20);
						*ptr++ = ldr64_pcrel(2, 2);
						*ptr++ = br(2);

						*ptr++ = u.u32[0];
						*ptr++ = u.u32[1];

						*ptr++ = fcpyd(*reg, 0);

						ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 7));
						*ext_count += 6;
						break;

					default:
						break;
				}
			}

			RA_FreeARMRegister(&ptr, int_reg);
		}
		/* Case 3: get pointer to data (EA) and fetch yourself */
		else {

			uint8_t int_reg = 0xff;
			uint8_t val_reg = 0xff;
			uint8_t mode = (opcode & 0x0038) >> 3;
			int8_t pre_sz = 0;
			int8_t post_sz = 0;

			if (mode == 4 || mode == 3)
				ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &int_reg, opcode & 0x3f, *m68k_ptr, ext_count, 0, NULL);
			else
				ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &int_reg, opcode & 0x3f, *m68k_ptr, ext_count, 1, &imm_offset);

			/* Pre index? Adjust base register accordingly */
			if (mode == 4) {
				pre_sz = FPUDataSize[size];

				if ((pre_sz == 1) && ((opcode & 7) == 7))
					pre_sz = 2;

				pre_sz = -pre_sz;
			}
			/* Post index? Adjust base register accordingly */
			else if (mode == 3) {
				post_sz = FPUDataSize[size];

				if ((post_sz == 1) && ((opcode & 7) == 7))
					post_sz = 2;
			}

			switch (size) {

				case SIZE_P:
					if (pre_sz)
						*ptr++ = sub_immed(int_reg, int_reg, -pre_sz);
					if (imm_offset < -255 || imm_offset > 251) {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						RA_FreeARMRegister(&ptr, int_reg);
						int_reg = off;
						imm_offset = 0;
					}

					u.u64 = (uintptr_t)PackedToDouble;

					ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 7));

					*ptr++ = ldur64_offset(int_reg, 0, imm_offset);
					*ptr++ = ldur64_offset(int_reg, 1, imm_offset + 8);
					*ptr++ = adr(30, 20);
					*ptr++ = ldr64_pcrel(2, 2);
					*ptr++ = br(2);

					*ptr++ = u.u32[0];
					*ptr++ = u.u32[1];

					*ptr++ = fcpyd(*reg, 0);

					ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 7));

					if (post_sz)
						*ptr++ = add_immed(int_reg, int_reg, post_sz);
				break;

				case SIZE_X:
					if (pre_sz)
						*ptr++ = sub_immed(int_reg, int_reg, -pre_sz);
					if (imm_offset < -255 || imm_offset > 251) {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						RA_FreeARMRegister(&ptr, int_reg);
						int_reg = off;
						imm_offset = 0;
					}

					ptr = get_Load96(ptr);
					*ptr++ = str64_offset_preindex(31, 30, -16);
					if (imm_offset < 0)
						*ptr++ = sub_immed(1, int_reg, -imm_offset);
					else
						*ptr++ = add_immed(1, int_reg, imm_offset);
					*ptr++ = blr(reg_Load96);
					*ptr++ = mov_reg_to_simd(*reg, TS_D, 0, 0);
					*ptr++ = ldr64_offset_postindex(31, 30, 16);

					//ptr = EMIT_Load96bitFP(ptr, *reg, int_reg, imm_offset);

					if (post_sz)
						*ptr++ = add_immed(int_reg, int_reg, post_sz);
				break;

				case SIZE_D:
					if (pre_sz)
						*ptr++ = fldd_preindex(*reg, int_reg, pre_sz);
					else if (post_sz)
						*ptr++ = fldd_postindex(*reg, int_reg, post_sz);
					else if (imm_offset >= -255 && imm_offset <= 255)
						*ptr++ = fldd(*reg, int_reg, imm_offset);
					else if (imm_offset >= 0 && imm_offset < 32760 && !(imm_offset & 7))
						*ptr++ = fldd_pimm(*reg, int_reg, imm_offset >> 3);
					else {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						*ptr++ = fldd(*reg, off, 0);
						RA_FreeARMRegister(&ptr, off);
					}
				break;

				case SIZE_S:
					if (pre_sz)
						*ptr++ = flds_preindex(*reg, int_reg, pre_sz);
					else if (post_sz)
						*ptr++ = flds_postindex(*reg, int_reg, post_sz);
					else if (imm_offset >= -255 && imm_offset <= 255)
						*ptr++ = flds(*reg, int_reg, imm_offset);
					else if (imm_offset >= 0 && imm_offset < 16380 && !(imm_offset & 3))
						*ptr++ = flds_pimm(*reg, int_reg, imm_offset >> 2);
					else {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						*ptr++ = flds(*reg, off, 0);
						RA_FreeARMRegister(&ptr, off);
					}
					*ptr++ = fcvtds(*reg, *reg);
				break;

				case SIZE_L:
					val_reg = RA_AllocARMRegister(&ptr);

					if (pre_sz)
						*ptr++ = ldr_offset_preindex(int_reg, val_reg, pre_sz);
					else if (post_sz)
						*ptr++ = ldr_offset_postindex(int_reg, val_reg, post_sz);
					else if (imm_offset >= -255 && imm_offset <= 255)
						*ptr++ = ldur_offset(int_reg, val_reg, imm_offset);
					else if (imm_offset >= 0 && imm_offset < 16380 && !(imm_offset & 3))
						*ptr++ = ldr_offset(int_reg, val_reg, imm_offset);
					else {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						*ptr++ = ldr_offset(off, val_reg, 0);
						RA_FreeARMRegister(&ptr, off);
					}
					*ptr++ = scvtf_32toD(*reg, val_reg);
				break;

				case SIZE_W:
					val_reg = RA_AllocARMRegister(&ptr);

					if (pre_sz)
						*ptr++ = ldrsh_offset_preindex(int_reg, val_reg, pre_sz);
					else if (post_sz)
						*ptr++ = ldrsh_offset_postindex(int_reg, val_reg, post_sz);
					else if (imm_offset >= -255 && imm_offset <= 255)
						*ptr++ = ldursh_offset(int_reg, val_reg, imm_offset);
					else if (imm_offset >= 0 && imm_offset < 8190 && !(imm_offset & 1))
						*ptr++ = ldrsh_offset(int_reg, val_reg, imm_offset);
					else {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						*ptr++ = ldrsh_offset(off, val_reg, 0);
						RA_FreeARMRegister(&ptr, off);
					}
					*ptr++ = scvtf_32toD(*reg, val_reg);
				break;

				case SIZE_B:
					val_reg = RA_AllocARMRegister(&ptr);

					if (pre_sz)
						*ptr++ = ldrsb_offset_preindex(int_reg, val_reg, pre_sz);
					else if (post_sz)
						*ptr++ = ldrsb_offset_postindex(int_reg, val_reg, post_sz);
					else if (imm_offset >= -255 && imm_offset <= 255)
						*ptr++ = ldursb_offset(int_reg, val_reg, imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = ldrsb_offset(int_reg, val_reg, imm_offset);
					else {

						uint8_t off = RA_AllocARMRegister(&ptr);

						if (imm_offset > -4096 && imm_offset < 0)
							*ptr++ = sub_immed(off, int_reg, -imm_offset);
						else if (imm_offset >= 0 && imm_offset < 4096)
							*ptr++ = add_immed(off, int_reg, imm_offset);
						else {
							*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
							imm_offset >>= 16;
							if (imm_offset)
								*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
							*ptr++ = add_reg(off, int_reg, off, LSL, 0);
						}
						*ptr++ = ldrsb_offset(off, val_reg, 0);
						RA_FreeARMRegister(&ptr, off);
					}
					*ptr++ = scvtf_32toD(*reg, val_reg);
				break;

				default:
				break;
			}
			if ((mode == 4) || (mode == 3))
				RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));

			RA_FreeARMRegister(&ptr, int_reg);
			RA_FreeARMRegister(&ptr, val_reg);
		}
	}
	return ptr;
}

/* Allocates FPU register and fetches data according to the R/M field of the FPU opcode */
uint32_t *FPU_StoreData(uint32_t *ptr, uint16_t **m68k_ptr, uint8_t reg, uint16_t opcode,
		uint16_t opcode2, uint8_t *ext_count) {
	/*
		Store is always to memory, R/M was set to 1, the source is FPU register, target is
		defined by EA stored in first part of the opcode. Destination identifier specifies
		the data length.

		Get EA, eventually (in case of mode 000 - Dn) perform simple data transfer.
		Otherwise get address from EA and store data here
	*/

	uint8_t ea = opcode & 0x3f;
	enum FPUOpSize size = (opcode2 >> 10) & 7;
	int32_t imm_offset = 0;

	/* Case 1: mode 000 - Dn */
	if ((ea & 077) == 0) {

		uint8_t int_reg = 0xff;
		uint8_t tmp_reg = 0xff;
		uint8_t tmp_reg_2 = 0xff;
		uint8_t vfp_reg = RA_AllocFPURegister(&ptr);

		switch (size) {

			case SIZE_S:
				int_reg = RA_MapM68kRegisterForWrite(&ptr, ea & 7); // Destination for write only, discard contents
				*ptr++ = fcvtsd(vfp_reg, reg);				  // Convert double to single
				*ptr++ = fmrs(int_reg, vfp_reg);				// Move single to destination ARM reg
				RA_FreeARMRegister(&ptr, int_reg);
				break;

			case SIZE_L:
				int_reg = RA_MapM68kRegisterForWrite(&ptr, ea & 7); // Destination for write only, discard contents
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(int_reg, vfp_reg);
				RA_FreeARMRegister(&ptr, int_reg);
				break;

			case SIZE_W:
				int_reg = RA_MapM68kRegister(&ptr, ea & 7);
				tmp_reg = RA_AllocARMRegister(&ptr);
				tmp_reg_2 = RA_AllocARMRegister(&ptr);
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(tmp_reg, vfp_reg);
				/* Saturate the result to match in 16 bits */
				*ptr++ = cmn_immed_lsl12(tmp_reg, 8);
				*ptr++ = movn_immed_u16(tmp_reg_2, 0x7fff, 0);
				*ptr++ = csel(tmp_reg, tmp_reg, tmp_reg_2, A64_CC_GE);
				*ptr++ = mov_immed_u16(tmp_reg_2, 0x7fff, 0);
				*ptr++ = cmp_reg(tmp_reg, tmp_reg_2, LSL, 0);
				*ptr++ = csel(tmp_reg, tmp_reg, tmp_reg_2, A64_CC_LE);
				*ptr++ = bfi(int_reg, tmp_reg, 0, 16);
				RA_SetDirtyM68kRegister(&ptr, ea & 7);
				RA_FreeARMRegister(&ptr, tmp_reg);
				RA_FreeARMRegister(&ptr, tmp_reg_2);
				RA_FreeARMRegister(&ptr, int_reg);
				break;

			case SIZE_B:
				int_reg = RA_MapM68kRegister(&ptr, ea & 7);
				tmp_reg = RA_AllocARMRegister(&ptr);
				tmp_reg_2 = RA_AllocARMRegister(&ptr);
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(tmp_reg, vfp_reg);
				/* Saturate the result to match in 16 bits */
				*ptr++ = cmn_immed(tmp_reg, 128);
				*ptr++ = movn_immed_u16(tmp_reg_2, 0x7f, 0);
				*ptr++ = csel(tmp_reg, tmp_reg, tmp_reg_2, A64_CC_GE);
				*ptr++ = mov_immed_u16(tmp_reg_2, 0x7f, 0);
				*ptr++ = cmp_immed(tmp_reg, 127);
				*ptr++ = csel(tmp_reg, tmp_reg, tmp_reg_2, A64_CC_LE);
				*ptr++ = bfi(int_reg, tmp_reg, 0, 8);
				RA_SetDirtyM68kRegister(&ptr, ea & 7);
				RA_FreeARMRegister(&ptr, tmp_reg);
				RA_FreeARMRegister(&ptr, int_reg);
				RA_FreeARMRegister(&ptr, tmp_reg_2);
				break;

			default:
				kprintf("[JIT] LineF: wrong argument size %d for Dn access\n", (int)size);
		}
		RA_FreeFPURegister(&ptr, vfp_reg);
	}
	/* Case 2: get pointer to data (EA) and store yourself */
	else {

		uint8_t int_reg = 0xff;
		uint8_t val_reg = 0xff;
		uint8_t mode = (opcode & 0x0038) >> 3;
		uint8_t vfp_reg = RA_AllocFPURegister(&ptr);
		int8_t pre_sz = 0;
		int8_t post_sz = 0;
		int8_t k = 0;
		uint8_t tmp32 = RA_AllocARMRegister(&ptr);
		union {
			uint64_t u64;
			uint32_t u32[2];
		} u;

		if (mode == 4 || mode == 3)
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &int_reg, opcode & 0x3f, *m68k_ptr, ext_count, 0, NULL);
		else
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &int_reg, opcode & 0x3f, *m68k_ptr, ext_count, 1, &imm_offset);

		/* Pre index? Adjust base register accordingly */
		if (mode == 4) {
			pre_sz = FPUDataSize[size];

			if ((pre_sz == 1) && ((opcode & 7) == 7))
				pre_sz = 2;

			pre_sz = -pre_sz;
		}
		/* Post index? Adjust base register accordingly */
		else if (mode == 3) {
			post_sz = FPUDataSize[size];

			if ((post_sz == 1) && ((opcode & 7) == 7))
				post_sz = 2;
		}

		switch (size) {

			case SIZE_P:
				u.u64 = (uintptr_t)DoubleToPacked;
				k = opcode2 & 0x7f;

				if (pre_sz)
					*ptr++ = sub_immed(int_reg, int_reg, -pre_sz);

				if (reg != 0)
					*ptr++ = fcpyd(0, reg);

				if (imm_offset >= -255 && imm_offset <= 251) {
					ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));

					*ptr++ = mov_reg(19, int_reg);
					*ptr++ = mov_immed_s8(0, k);
					*ptr++ = adr(30, 20);
					*ptr++ = ldr64_pcrel(1, 2);
					*ptr++ = br(1);

					*ptr++ = u.u32[0];
					*ptr++ = u.u32[1];

					*ptr++ = ror64(1, 1, 32);
					*ptr++ = stur64_offset(19, 0, imm_offset);
					*ptr++ = stur_offset(19, 1, imm_offset + 8);

					ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));
				}
				else {

					uint8_t off = 19;

					ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}

					*ptr++ = mov_immed_s8(0, k);
					*ptr++ = adr(30, 20);
					*ptr++ = ldr64_pcrel(1, 2);
					*ptr++ = br(1);

					*ptr++ = u.u32[0];
					*ptr++ = u.u32[1];

					*ptr++ = ror64(1, 1, 32);
					*ptr++ = stur64_offset(19, 0, 0);
					*ptr++ = stur_offset(19, 1, 8);

					ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));
				}

				if (post_sz)
					*ptr++ = add_immed(int_reg, int_reg, post_sz);
				break;

			case SIZE_Pdyn:

				u.u64 = (uintptr_t)DoubleToPacked;
				k = RA_MapM68kRegister(&ptr, (opcode2 >> 4) & 7);

				if (pre_sz)
					*ptr++ = sub_immed(int_reg, int_reg, -pre_sz);

				if (reg != 0)
					*ptr++ = fcpyd(0, reg);

				if (imm_offset >= -255 && imm_offset <= 251) {
					ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));

					*ptr++ = mov_reg(0, k);
					*ptr++ = mov_reg(19, int_reg);
					*ptr++ = adr(30, 20);
					*ptr++ = ldr64_pcrel(1, 2);
					*ptr++ = br(1);

					*ptr++ = u.u32[0];
					*ptr++ = u.u32[1];

					*ptr++ = ror64(1, 1, 32);
					*ptr++ = stur64_offset(19, 0, imm_offset);
					*ptr++ = stur_offset(19, 1, imm_offset + 8);

					ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));
				}
				else {

					uint8_t off = 19;

					ptr = EMIT_SaveRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));

					*ptr++ = mov_reg(0, k);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}

					*ptr++ = adr(30, 20);
					*ptr++ = ldr64_pcrel(1, 2);
					*ptr++ = br(1);

					*ptr++ = u.u32[0];
					*ptr++ = u.u32[1];

					*ptr++ = ror64(1, 1, 32);
					*ptr++ = stur64_offset(19, 0, 0);
					*ptr++ = stur_offset(19, 1, 8);

					ptr = EMIT_RestoreRegFrame(ptr, (RA_GetTempAllocMask() | REG_PROTECT | 3 | (1 << 19)));
				}

				if (post_sz)
					*ptr++ = add_immed(int_reg, int_reg, post_sz);
				break;

			case SIZE_X:

				if (pre_sz)
					*ptr++ = sub_immed(int_reg, int_reg, -pre_sz);
				if (imm_offset >= -255 && imm_offset <= 251) {
					ptr = get_Save96(ptr);
					*ptr++ = str64_offset_preindex(31, 30, -16);
					if (imm_offset < 0)
						*ptr++ = sub_immed(1, int_reg, -imm_offset);
					else
						*ptr++ = add_immed(1, int_reg, imm_offset);
					*ptr++ = mov_simd_to_reg(0, reg, TS_D, 0);
					*ptr++ = blr(reg_Save96);
					*ptr++ = ldr64_offset_postindex(31, 30, 16);
					//ptr = EMIT_Store96bitFP(ptr, reg, int_reg, imm_offset);
				}
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					ptr = get_Save96(ptr);
					*ptr++ = str64_offset_preindex(31, 30, -16);
					*ptr++ = mov_reg(1, off);
					*ptr++ = mov_simd_to_reg(0, reg, TS_D, 0);
					*ptr++ = blr(reg_Save96);
					*ptr++ = ldr64_offset_postindex(31, 30, 16);
					//ptr = EMIT_Store96bitFP(ptr, reg, off, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				if (post_sz)
					*ptr++ = add_immed(int_reg, int_reg, post_sz);
				break;

			case SIZE_D:
				if (pre_sz)
					*ptr++ = fstd_preindex(reg, int_reg, pre_sz);
				else if (post_sz)
					*ptr++ = fstd_postindex(reg, int_reg, post_sz);
				else if (imm_offset >= -255 && imm_offset <= 255)
					*ptr++ = fstd(reg, int_reg, imm_offset);
				else if (imm_offset >= 0 && imm_offset < 32760 && !(imm_offset & 7))
					*ptr++ = fstd_pimm(reg, int_reg, imm_offset >> 3);
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					*ptr++ = fstd(reg, off, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				break;

			case SIZE_S:
				*ptr++ = fcvtsd(vfp_reg, reg);

				if (pre_sz)
					*ptr++ = fsts_preindex(vfp_reg, int_reg, pre_sz);
				else if (post_sz)
					*ptr++ = fsts_postindex(vfp_reg, int_reg, post_sz);
				else if (imm_offset >= -255 && imm_offset <= 255)
					*ptr++ = fsts(vfp_reg, int_reg, imm_offset);
				else if (imm_offset >= 0 && imm_offset < 16380 && !(imm_offset & 3))
					*ptr++ = fsts_pimm(vfp_reg, int_reg, imm_offset >> 2);
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					*ptr++ = fsts(vfp_reg, off, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				break;
			case SIZE_L:
				val_reg = RA_AllocARMRegister(&ptr);
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(val_reg, vfp_reg);

				if (pre_sz)
					*ptr++ = str_offset_preindex(int_reg, val_reg, pre_sz);
				else if (post_sz)
					*ptr++ = str_offset_postindex(int_reg, val_reg, post_sz);
				else if (imm_offset >= -255 && imm_offset <= 255)
					*ptr++ = stur_offset(int_reg, val_reg, imm_offset);
				else if (imm_offset >= 0 && imm_offset < 16380 && !(imm_offset & 3))
					*ptr++ = str_offset(int_reg, val_reg, imm_offset);
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					*ptr++ = str_offset(off, val_reg, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				break;
			case SIZE_W:
				val_reg = RA_AllocARMRegister(&ptr);
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(val_reg, vfp_reg);

				/* Saturate the result to match in 16 bits */
				*ptr++ = cmn_immed_lsl12(val_reg, 8);
				*ptr++ = movn_immed_u16(tmp32, 0x7fff, 0);
				*ptr++ = csel(val_reg, val_reg, tmp32, A64_CC_GE);
				*ptr++ = mov_immed_u16(tmp32, 0x7fff, 0);
				*ptr++ = cmp_reg(val_reg, tmp32, LSL, 0);
				*ptr++ = csel(val_reg, val_reg, tmp32, A64_CC_LE);

				if (pre_sz)
					*ptr++ = strh_offset_preindex(int_reg, val_reg, pre_sz);
				else if (post_sz)
					*ptr++ = strh_offset_postindex(int_reg, val_reg, post_sz);
				else if (imm_offset >= -255 && imm_offset <= 255)
					*ptr++ = sturh_offset(int_reg, val_reg, imm_offset);
				else if (imm_offset >= 0 && imm_offset < 8190 && !(imm_offset & 1))
					*ptr++ = strh_offset(int_reg, val_reg, imm_offset);
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					*ptr++ = strh_offset(off, val_reg, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				break;
			case SIZE_B:
				val_reg = RA_AllocARMRegister(&ptr);
				*ptr++ = frint64x(vfp_reg, reg);
				*ptr++ = fcvtzs_Dto32(val_reg, vfp_reg);

				/* Saturate the result to match in 16 bits */
				*ptr++ = cmn_immed(val_reg, 128);
				*ptr++ = movn_immed_u16(tmp32, 0x7f, 0);
				*ptr++ = csel(val_reg, val_reg, tmp32, A64_CC_GE);
				*ptr++ = mov_immed_u16(tmp32, 0x7f, 0);
				*ptr++ = cmp_immed(val_reg, 127);
				*ptr++ = csel(val_reg, val_reg, tmp32, A64_CC_LE);

				if (pre_sz)
					*ptr++ = strb_offset_preindex(int_reg, val_reg, pre_sz);
				else if (post_sz)
					*ptr++ = strb_offset_postindex(int_reg, val_reg, post_sz);
				else if (imm_offset >= -255 && imm_offset <= 255)
					*ptr++ = sturb_offset(int_reg, val_reg, imm_offset);
				else if (imm_offset >= 0 && imm_offset < 4096)
					*ptr++ = strb_offset(int_reg, val_reg, imm_offset);
				else {

					uint8_t off = RA_AllocARMRegister(&ptr);

					if (imm_offset > -4096 && imm_offset < 0)
						*ptr++ = sub_immed(off, int_reg, -imm_offset);
					else if (imm_offset >= 0 && imm_offset < 4096)
						*ptr++ = add_immed(off, int_reg, imm_offset);
					else {
						*ptr++ = movw_immed_u16(off, (imm_offset) & 0xffff);
						imm_offset >>= 16;
						if (imm_offset)
							*ptr++ = movt_immed_u16(off, (imm_offset) & 0xffff);
						*ptr++ = add_reg(off, int_reg, off, LSL, 0);
					}
					*ptr++ = strb_offset(off, val_reg, 0);
					RA_FreeARMRegister(&ptr, off);
				}
				break;
			default:
				break;
		}
		if ((mode == 4) || (mode == 3))
			RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));

		RA_FreeARMRegister(&ptr, tmp32);
		RA_FreeFPURegister(&ptr, vfp_reg);
		RA_FreeARMRegister(&ptr, int_reg);
		RA_FreeARMRegister(&ptr, val_reg);
	}
	return ptr;
}


/* MOVES */

/* FMOVECR

Operation: ROM Constant → FPn
Assembler Syntax: FMOVECR.X #<ccc>,FPn
Opcode Map: [0xf200],[010111b 15::10][FPn 9::7][ROM offset 6::0]
Attributes: Format = (Extended)
FPSR Affected: 0x0f000208
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FMOVECR(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {
	/* FMOVECR only pulls extended-precision constants to a FP register */
	enum {
		C_PI = 0,
		C_PI_2,
		C_PI_4,
		C_1_PI,
		C_2_PI,
		C_2_SQRTPI,
		C_1_2PI,
		C_SQRT2,
		C_SQRT1_2,
		C_0_5,
		C_1_5,
		C_LOG10_2 = 0x0b,
		C_E,
		C_LOG2E,
		C_LOG10E,
		C_ZERO,
		C_SIN_COEFF = 0x10,  /* 21-poly for sine approximation - error margin within double precision */
		C_COS_COEFF = 0x20,  /* 20-poly for cosine approximation -error margin within double precision */

		C_SIN_COEFF_SINGLE = 0x1a,
		C_COS_COEFF_SINGLE = 0x2a,

		C_LN2 = 0x30,
		C_LN10,
		C_10P0,
		C_10P1,
		C_10P2,
		C_10P4,
		C_10P8,
		C_10P16,
		C_10P32,
		C_10P64,
		C_10P128,
		C_10P256,
		C_10P512,
		C_10P1024,
		C_10P2048,
		C_10P4096,

		C_TWO54,
		C_LN2HI,
		C_LN2LO,
		C_LG1,
		C_LG2,
		C_LG3,
		C_LG4,
		C_LG5,
		C_LG6,
		C_LG7
	};

	static double const __attribute__((used)) constants[128] = {
		[C_PI] =		3.14159265358979323846264338327950288, /* Official */
		[C_PI_2] =	  1.57079632679489661923132169163975144,
		[C_PI_4] =	  0.785398163397448309615660845819875721,
		[C_1_PI] =	  0.318309886183790671537767526745028724,
		[C_2_PI] =	  0.636619772367581343075535053490057448,
		[C_2_SQRTPI] =  1.12837916709551257389615890312154517,
		[C_1_2PI] =	 0.1591549430918953357688837633725143620,
		[C_SQRT2] =	 1.41421356237309504880168872420969808,
		[C_SQRT1_2] =   0.707106781186547524400844362104849039,
		[C_0_5] =	   0.5,
		[C_1_5] =	   1.5,
		[C_LOG10_2] =   0.301029995663981195214, /* Official - Log10(2) */
		[C_E] =		 2.71828182845904523536028747135266250,				/* Official */
		[C_LOG2E] =	 1.44269504088896340735992468100189214,			/* Official */
		[C_LOG10E] =	0.434294481903251827651128918916605082,		   /* Official */
		[C_ZERO] =	  0.0,				/* Official */

		/* Polynom coefficients for sin(x*Pi), x=0..0.5*/

		[C_SIN_COEFF] = -2.11100178050346585936E-5,
						4.65963708473294521719E-4,
						-7.37035513524020578156E-3,
						8.21458769726032277098E-2,
						-5.99264528627362954518E-1,
						2.55016403985097679243,
						-5.16771278004952168888,
						3.14159265358979102647,

		/* Reduced number of polynom coefficients for sin(x*Pi), x=0..0.5 */

		[C_SIN_COEFF_SINGLE] =
						7.74455095806670556524E-2,
						-5.98160819620617657839E-1,
						2.55005088882843729408,
						-5.1677080762924026306,
						3.14159259939191476447,

		/* Polynom coefficients for cos(x*Pi), x=0..0.5 */

		[C_COS_COEFF] = 4.15383875943350535407E-6,
						-1.04570624685965272291E-4,
						1.92955784205552168426E-3,
						-2.58068890507489103003E-2,
						2.35330630164104256943E-1,
						-1.33526276884550367708,
						4.05871212641655666324,
						-4.93480220054467742126,
						9.99999999999999997244E-1,

		/* Reduced number of polynom coefficients for cos(x*Pi), x=0..0.5 */
		[C_COS_COEFF_SINGLE] =
						2.20485796302921884119E-1,
						-1.33223541188749370639,
						4.058461009872062766402,
						-4.93479497666537363458,
						9.99999967245121125386E-1,

		[C_LN2] =	   0.693147180559945309417232121458176568,			  /* Official */
		[C_LN10] =	  2.30258509299404568401799145468436421,			 /* Official */
		[C_10P0] =	  1.0,				/* Official */
		[C_10P1] =	  1E1,				/* Official */
		[C_10P2] =	  1E2,				/* Official */
		[C_10P4] =	  1E4,				/* Official */
		[C_10P8] =	  1E8,				/* Official */
		[C_10P16] =	 1E16,			   /* Official */
		[C_10P32] =	 1E32,			   /* Official */
		[C_10P64] =	 1E64,			   /* Official */
		[C_10P128] =	1E128,			  /* Official */
		[C_10P256] =	1E256,			  /* Official */
		[C_10P512] =	0x1.fffffep+127f,		   /* Official 1E512 - too large for double! */
		[C_10P1024] =   0x1.fffffep+127f,		   /* Official 1E1024 - too large for double! */
		[C_10P2048] =   0x1.fffffep+127f,		   /* Official 1E2048 - too large for double! */
		[C_10P4096] =   0x1.fffffep+127f,		   /* Official 1E4096 - too large for double! */

		[C_TWO54] =	 1.80143985094819840000e+16,
		[C_LN2HI] =	 6.93147180369123816490e-01,
		[C_LN2LO] =	 1.90821492927058770002e-10,

		[C_LG1] =	   6.666666666666735130e-01,
		[C_LG2] =	   3.999999999940941908e-01,
		[C_LG3] =	   2.857142874366239149e-01,
		[C_LG4] =	   2.222219843214978396e-01,
		[C_LG5] =	   1.818357216161805012e-01,
		[C_LG6] =	   1.531383769920937332e-01,
		[C_LG7] =	   1.479819860511658591e-01,
	};

	static int shown = 0;
	if (!shown) {
		kprintf("FMOVECR\n");
		shown = 1;
	}

	union {
		double d;
		uint32_t u32[2];
	} u;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t offset = opcode2 & 0x7f;

	/* Alloc destination FP register for write */
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	/*
		Load pointer to constants into base register, then load the value from table into
		destination VFP register, finally skip the base address (which is not an ARM INSN)
	*/
	if (offset == C_10P0) {
		*ptr++ = fmov_1(fp_dst);
	}
	else if (offset == C_ZERO) {
		*ptr++ = fmov_0(fp_dst);
	}
	else {
		u.d = constants[offset];
		*ptr++ = fldd_pcrel(fp_dst, 2);
		*ptr++ = b(3);
		*ptr++ = u.u32[0];
		*ptr++ = u.u32[1];
	}
	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		if (offset == C_ZERO) {
			*ptr++ = bic_immed(fpsr, fpsr, 4, 32 - FPSRB_NAN);
			*ptr++ = orr_immed(fpsr, fpsr, 1, 32 - FPSRB_Z);
		}
		else if (offset < C_ZERO || offset >= C_LN2)
			*ptr++ = bic_immed(fpsr, fpsr, 4, 32 - FPSRB_NAN);
		else {
			*ptr++ = fcmpzd(fp_dst);
			ptr = EMIT_GetFPUFlags(ptr, fpsr);
		}
	}
	return ptr;
}

/* FMOVE

Operation: Src → Dest
Assembler Syntax: FMOVE.X FPm,FPn
Opcode Map: [0xf200],[00 15::13][FPm 12::10][FPn 9::7][0x00 6::0] //in theory FSMOVE and FDMOVE can be done here too!
Assembler Syntax: FMOVE.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][FPn 9::7][0x00 6::0]
FPSR Affected: 0x0f004b28
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FMOVE_FP(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {
	return ptr;
}
/* FMOVE

Operation: Src → Dest
Assembler Syntax: FMOVE.<fmt> FPn,<ea>([Dn|k]) //please don't use Packed format!
Opcode Map: [0xf][010 11::6][ea 5::0],[03 15::13][fmt 12::10][FPn 9::7][k-Factor 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f007b68
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FMOVE(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	return ptr;
}

/* FSMOVE

Operation: Src → Dest
Assembler Syntax: FSMOVE.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][FPn 9::7][0x40 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f004b28
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FMOVE_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	return ptr;
}

/* FDMOVE

Operation: Src → Dest
Assembler Syntax: FDMOVE.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][FPn 9::7][0x44 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f004b28
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FMOVE_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	return ptr;
}

/* FMOVEM & FMOVE
Operation: Src → Dest
Assembler Syntax: FMOVE.L <ea>,FPCR
Opcode Map: [0xf][010 11::6][ea 5::0],[04 15::13][reg 12::10][0x000 9::0]
Assembler Syntax: FMOVE.L FPCR,<ea>
Opcode Map: [0xf][010 11::6][ea 5::0],[05 15::13][reg 12::10][0x000 9::0]
Assembler Syntax: FMOVEM.L <ea>,<list>
Opcode Map: [0xf][010 11::6][ea 5::0],[04 15::13][reg 12::10][0x000 9::0]
Assembler Syntax: FMOVEM.L <list>,<ea>
Opcode Map: [0xf][010 11::6][ea 5::0],[05 15::13][reg 12::10][0x000 9::0]
Attributes: Format = (Long)
FPSR Affected: if mem -> reg & bit [11] = 1
FPSR Cleared: 0x00000000
*/
//this instruction should not update FPIAR when executed, this is always Long
uint32_t *EMIT_FMOVEM_L(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	uint8_t direction = ((opcode2 >> 13) & 1);

	return ptr;
}

/* FMOVEM

Operation: Src → Dest
Assembler Syntax: FMOVEM.X (Dn|<list>),<ea>
Opcode Map: [0xf][010 11::6][ea 5::0],[06 15::13][mode 12::11][0x0 10::8][register list field 7::0]
Assembler Syntax: FMOVEM.X <ea>,(Dn|<list>)
Opcode Map: [0xf][010 11::6][ea 5::0],[07 15::13][mode 12::11][0x0 10::8][register list field 7::0]
Attributes: Format = (Extended)
FPSR Affected: none
FPSR Cleared: 0x00000000
*/
//this instruction should not update FPIAR when executed, this is always extended
uint32_t *EMIT_FMOVEM(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FMOVEM\n");
		shown = 1;
	}

	char dir = (opcode2 >> 13) & 1;
	uint8_t base_reg = 0xff;
	uint8_t ext_count = 1;

	if (dir) {
		/* FPn to memory */
		uint8_t mode = (opcode & 0x0038) >> 3;

		if (mode == 4 || mode == 3)
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &base_reg, opcode & 077, *m68k_ptr, &ext_count, 0, NULL);
		else
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &base_reg, opcode & 077, *m68k_ptr, &ext_count, 1, NULL);

			/* Pre index? Note - dynamic mode not supported yet! using double mode instead of extended! */
		if (mode == 4) {

				int size = 0;
				int cnt = 0;
				for (int i=0; i < 8; i++)
					if ((opcode2 & (1 << i)))
						size++;
				*ptr++ = sub_immed(base_reg, base_reg, 12*size);

				ptr = get_Save96(ptr);
				*ptr++ = str64_offset_preindex(31, 30, -16);

				for (int i=0; i < 8; i++) {
					if ((opcode2 & (1 << i)) != 0) {
						uint8_t fp_reg = RA_MapFPURegister(&ptr, i);
						//*ptr++ = sub_immed(base_reg, base_reg, 12);

						*ptr++ = add_immed(1, base_reg, 12*cnt);
						*ptr++ = mov_simd_to_reg(0, fp_reg, TS_D, 0);
						*ptr++ = blr(reg_Save96);
						//ptr = EMIT_Store96bitFP(ptr, fp_reg, base_reg, 12*cnt++);
						//*ptr++ = fstd(fp_reg, base_reg, 12*cnt++);

						cnt++;
						RA_FreeFPURegister(&ptr, fp_reg);
					}
				}
				*ptr++ = ldr64_offset_postindex(31, 30, 16);
				RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));
			} else if (mode == 3) {
				kprintf("[JIT] Unsupported FMOVEM operation (REG to MEM postindex)\n");
			} else {
				ptr = get_Save96(ptr);
				*ptr++ = str64_offset_preindex(31, 30, -16);

				int cnt = 0;
				for (int i=0; i < 8; i++) {
					if ((opcode2 & (0x80 >> i)) != 0) {
						uint8_t fp_reg = RA_MapFPURegister(&ptr, i);

						*ptr++ = add_immed(1, base_reg, 12*cnt);
						*ptr++ = mov_simd_to_reg(0, fp_reg, TS_D, 0);
						*ptr++ = blr(reg_Save96);

						//ptr = EMIT_Store96bitFP(ptr, fp_reg, base_reg, 12*cnt);
						//*ptr++ = fstd(fp_reg, base_reg, cnt*12);
						cnt++;
						RA_FreeFPURegister(&ptr, fp_reg);
					}
				}

				*ptr++ = ldr64_offset_postindex(31, 30, 16);
			}
		} else { /* memory to FPn */
			uint8_t mode = (opcode & 0x0038) >> 3;

			if (mode == 4 || mode == 3)
				ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &base_reg, opcode & 0x3f, *m68k_ptr, &ext_count, 0, NULL);
			else
				ptr = EMIT_LoadFromEffectiveAddress(ptr, 0, &base_reg, opcode & 0x3f, *m68k_ptr, &ext_count, 1, NULL);

			/* Post index? Note - dynamic mode not supported yet! using double mode instead of extended! */
			if (mode == 3) {
				ptr = get_Load96(ptr);
				*ptr++ = str64_offset_preindex(31, 30, -16);

				int cnt = 0;
				for (int i=0; i < 8; i++) {
					if ((opcode2 & (0x80 >> i)) != 0) {
						uint8_t fp_reg = RA_MapFPURegisterForWrite(&ptr, i);

						*ptr++ = add_immed(1, base_reg, 12*cnt);
						*ptr++ = blr(reg_Load96);
						*ptr++ = mov_reg_to_simd(fp_reg, TS_D, 0, 0);

						//ptr = EMIT_Load96bitFP(ptr, fp_reg, base_reg, 12*cnt++);
						//*ptr++ = fldd(fp_reg, base_reg, 12*cnt++);
						//*ptr++ = add_immed(base_reg, base_reg, 12);
						RA_FreeFPURegister(&ptr, fp_reg);
						cnt++;
					}
				}

				*ptr++ = ldr64_offset_postindex(31, 30, 16);

				*ptr++ = add_immed(base_reg, base_reg, 12*cnt);
				RA_SetDirtyM68kRegister(&ptr, 8 + (opcode & 7));
			} else if (mode == 4) {
				kprintf("[JIT] Unsupported FMOVEM operation (REG to MEM preindex)\n");
			} else {
				ptr = get_Load96(ptr);
				*ptr++ = str64_offset_preindex(31, 30, -16);

				int cnt = 0;
				for (int i=0; i < 8; i++) {
					if ((opcode2 & (0x80 >> i)) != 0) {
						uint8_t fp_reg = RA_MapFPURegisterForWrite(&ptr, i);

						*ptr++ = add_immed(1, base_reg, 12*cnt);
						*ptr++ = blr(reg_Load96);
						*ptr++ = mov_reg_to_simd(fp_reg, TS_D, 0, 0);
						//ptr = EMIT_Load96bitFP(ptr, fp_reg, base_reg, 12*cnt);
						//*ptr++ = fldd(fp_reg, base_reg, cnt*12);
						cnt++;
						RA_FreeFPURegister(&ptr, fp_reg);
					}
				}

				*ptr++ = ldr64_offset_postindex(31, 30, 16);
			}
		}

		RA_FreeARMRegister(&ptr, base_reg);

		ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
		(*m68k_ptr) += ext_count;

		return ptr;
}

/* FPU Monadic Operations */
/* FABS

Operation: Absolute Value of Src → FPn
Assembler Syntax: FABS.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x18 6::0]
Assembler Syntax: FABS.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x18 6::0]
Assembler Syntax: FrABS.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x5(8|c) 6::0]
Assembler Syntax: FrABS.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x5(8|c) 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f0049
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FABS(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FABS_S")));
uint32_t *EMIT_FABS_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FABS_D")));
uint32_t *EMIT_FABS_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FABS\n");
		shown = 1;
	}

	val_FPIAR = (uintptr_t)&(*m68k_ptr)[-1];

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = fabsd(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FACOS

Operation: Arc Cosine of Src → FPn
Assembler Syntax: FACOS.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x1c 6::0]
Assembler Syntax: FACOS.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x1c 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f006388
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FACOS(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FACOS\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)acos;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FASIN

Operation: Arc Sine of Src → FPn
Assembler Syntax: FASIN.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x0c 6::0]
Assembler Syntax: FASIN.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x0c 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f006b
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FASIN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FASIN\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)asin;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FATAN

Operation: Arc Tangent of Src → FPn
Assembler Syntax: FATAN.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x0a 6::0]
Assembler Syntax: FATAN.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x0a 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f004b
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FATAN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FATAN\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)atan;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FATANH

Operation: Hyperbolic Arc Tangent of Src → FPn
Assembler Syntax: FATANH.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x0d 6::0]
Assembler Syntax: FATANH.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x0d 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f006f
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FATANH(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FATANH\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)atanh;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FCOS

Operation: Cosine of Src → FPn
Assembler Syntax: FCOS.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x1d 6::0]
Assembler Syntax: FCOS.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x1d 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f0063
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FCOS(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FCOS\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)cos;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FCOSH

Operation: Hyperbolic Cosine of Src → FPn
Assembler Syntax: FCOSH.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x19 6::0]
Assembler Syntax: FCOSH.X (FPm,)FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x19 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f0053
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FCOSH(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FCOSH\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)cosh;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FETOX(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FETOX\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)exp;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FETOXM1(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FETOXM1\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)expm1;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FGETEXP(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FGETEXP\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t tmp = RA_AllocARMRegister(&ptr);

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = mov_simd_to_reg(tmp, fp_src, TS_D, 0);
	*ptr++ = ror64(tmp, tmp, 52);
	*ptr++ = and_immed(tmp, tmp, 11, 0);
	*ptr++ = sub_immed(tmp, tmp, 0x3ff);
	*ptr++ = scvtf_32toD(fp_dst, tmp);

	RA_FreeFPURegister(&ptr, fp_src);
	RA_FreeARMRegister(&ptr, tmp);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FGETMAN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FGETMAN\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t tmp = RA_AllocARMRegister(&ptr);

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = mov_simd_to_reg(tmp, fp_src, TS_D, 0);
	*ptr++ = bic64_immed(tmp, tmp, 11, 12, 1);
	*ptr++ = orr64_immed(tmp, tmp, 10, 12, 1);
	*ptr++ = mov_reg_to_simd(fp_dst, TS_D, 0, tmp);

	RA_FreeFPURegister(&ptr, fp_src);
	RA_FreeARMRegister(&ptr, tmp);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FINT(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FINT\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = frint64x(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FINTRM(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	ptr = EMIT_InjectDebugString(ptr, "[JIT] FINTRM at %08x not implemented\n", *m68k_ptr - 1);
	ptr = EMIT_InjectPrintContext(ptr);
	*ptr++ = udf(opcode);

	return ptr;
}

uint32_t *EMIT_FINTRP(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	ptr = EMIT_InjectDebugString(ptr, "[JIT] FINTRP at %08x not implemented\n", *m68k_ptr - 1);
	ptr = EMIT_InjectPrintContext(ptr);
	*ptr++ = udf(opcode);

	return ptr;
}

uint32_t *EMIT_FINTRN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	ptr = EMIT_InjectDebugString(ptr, "[JIT] FINTRN at %08x not implemented\n", *m68k_ptr - 1);
	ptr = EMIT_InjectPrintContext(ptr);
	*ptr++ = udf(opcode);

	return ptr;
}

uint32_t *EMIT_FINTRZ(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FINTRZ\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = frint64z(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FLOGN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FLOGN\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)log;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FLOGNP1(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FLOGNP1\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)log1p;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FLOG10(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FLOG10\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)log10;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FLOG2(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FLOG2\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)log2;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FNEG(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FNEG_S")));
uint32_t *EMIT_FNEG_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FNEG_D")));
uint32_t *EMIT_FNEG_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FNEG\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = fnegd(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSIN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSIN\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)sin;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSINCOS(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSINCOS\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst_sin = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t fp_dst_cos = opcode2 & 7;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst_sin = RA_MapFPURegisterForWrite(&ptr, fp_dst_sin);
	fp_dst_cos = RA_MapFPURegisterForWrite(&ptr, fp_dst_cos);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)sincos;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst_cos, 1);
	*ptr++ = fcpyd(fp_dst_sin, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst_sin);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSINH(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSINH\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)sinh;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSQRT(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FSQRT_S")));
uint32_t *EMIT_FSQRT_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FSQRT_D")));
uint32_t *EMIT_FSQRT_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSQRT\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = fsqrtd(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}


uint32_t *EMIT_FTAN(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FATAN\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)tan;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FTANH(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FTANH\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)tanh;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}

	return ptr;
}

uint32_t *EMIT_FTENTOX(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FTENTOX\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)exp10;

	if (fp_src != 0) {
		*ptr++ = fcpyd(0, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FTST(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

    static int shown = 0;
	if (!shown) {
		kprintf("FTST\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);

	*ptr++ = fcmpzd(fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FTWOTOX(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FTWOTOX\n");
		shown = 1;
	}
	uint8_t fp_src = 0;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)exp2;

	if (fp_src != 0)
		*ptr++ = fcpyd(0, fp_src);

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	*ptr++ = fcpyd(fp_dst, 0);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FPU Dyadic Operations */
/* FADD

Operation: Src + FPn → FPn
Assembler Syntax: FADD.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x22 6::0]
Assembler Syntax: FADD.X FPm,FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x22 6::0]
Assembler Syntax: FrADD.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x6(2|6) 6::0]
Assembler Syntax: FrADD.X FPm,FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x6(2|6) 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f007b68
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FADD(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FADD_S")));
uint32_t *EMIT_FADD_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FADD_D")));
uint32_t *EMIT_FADD_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FADD\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = faddd(fp_dst, fp_dst, fp_src);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

/* FCMP

Operation: FPn - Src
Assembler Syntax: FCMP.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[02 15::13][fmt 12::10][dest 9::7][0x3 6::0]
Assembler Syntax: FCMP.X FPm,FPn
Opcode Map: [0xf200],[00 15::13][src 12::10][dest 9::7][0x38 6::0]
Attributes: Format = (Byte, Word, Long, Single, Double, Extended, Packed)
FPSR Affected: 0x0f0041
FPSR Cleared: 0x0000ff00
*/
uint32_t *EMIT_FCMP(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FCMP\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fcmpd(fp_dst, fp_src);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}
/* FDIV

Operation: FPn ÷ Src → FPn
Assembler Syntax: FDIV.<fmt> <ea>,FPn
Opcode Map: [0xf][010 11::6][ea 5::0],[020 15::13][fmt 12::10][dest 9::7][0x20 6::0]

*/
uint32_t *EMIT_FDIV(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FDIV_S")));
uint32_t *EMIT_FDIV_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FDIV_D")));
uint32_t *EMIT_FDIV_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FDIV\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fdivd(fp_dst, fp_dst, fp_src);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FMOD(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FMOD\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t tmp = RA_AllocARMRegister(&ptr);

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	/* Need to check if this method is working... */
	/* Compute FPn / Source */
	*ptr++ = fdivd(0, fp_dst, fp_src);
	/* Round to zero the result -> N */
	*ptr++ = frint64z(0, 0);
	/* And store for later */
	*ptr++ = fcvtzs_Dto64(tmp, 0);
	/* Get Source * N */
	*ptr++ = fmuld(1, 0, fp_src);
	/* Calculate reminder */
	*ptr++ = fsubd(fp_dst, fp_dst, 1);
	/* Test sign of result */
	*ptr++ = fcmpzd(0);
	*ptr++ = bic_immed(1, 0, 25, 25);
	*ptr++ = orr_immed(0, 0, 25, 25);
	*ptr++ = csel(0, 1, 0, A64_CC_PL);

	uint8_t fpsr = RA_ModifyFPSR(&ptr);
	*ptr++ = bfi(fpsr, 0, 16, 8);

	RA_FreeFPURegister(&ptr, fp_src);
	RA_FreeARMRegister(&ptr, tmp);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FMUL(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FMUL_S")));
uint32_t *EMIT_FMUL_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FMUL_D")));
uint32_t *EMIT_FMUL_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
	   kprintf("FMUL\n");
		shown = 1;
	}
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fmuld(fp_dst, fp_dst, fp_src);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FREM(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FREM\n");
		shown = 1;
	}

	uint8_t fp_src = 1;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = fcpyd(0, fp_dst);

	union {
		uint64_t u64;
		uint32_t u32[2];
	} u;

	u.u64 = (uintptr_t)remquo;

	if (fp_src != 1) {
		*ptr++ = fcpyd(1, fp_src);
	}

	ptr = EMIT_SaveRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	*ptr++ = adr(30, 20);
	*ptr++ = ldr64_pcrel(0, 2);
	*ptr++ = br(0);

	*ptr++ = u.u32[0];
	*ptr++ = u.u32[1];

	// Clear V0
	*ptr++ = fmov_0(0);
	// The result of remquo is not in FPU regiser, but rather integer. Put it into destination now
	*ptr++ = mov_reg_to_simd(fp_dst, TS_D, 0, 0);
	// Put quotient byte to the v0 first, before restoring register frame
	*ptr++ = mov_reg_to_simd(0, TS_B, 2, 1);

	ptr = EMIT_RestoreRegFrame(ptr, RA_GetTempAllocMask() | REG_PROTECT);

	uint8_t fpsr = RA_ModifyFPSR(&ptr);
	*ptr++ = bic_immed(fpsr, fpsr, 8, 16);

	// Once frame is resotred, get quotient byte and put it into FPSR
	uint8_t tmp_quot = RA_AllocARMRegister(&ptr);
	*ptr++ = mov_simd_to_reg(tmp_quot, 0, TS_S, 0);
	*ptr++ = orr_reg(fpsr, fpsr, tmp_quot, LSL, 0);
	RA_FreeARMRegister(&ptr, tmp_quot);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {
		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSCALE(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSCALE\n");
		shown = 1;
	}

	uint8_t int_src = 0xff;
	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	switch ((opcode2 >> 10) & 7) {
		case 0:
			fp_src = RA_AllocFPURegister(&ptr);
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 4, &int_src, opcode & 0x3f, *m68k_ptr, &ext_count, 0, NULL);
			break;
		case 4:
			fp_src = RA_AllocFPURegister(&ptr);
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0x80 | 2, &int_src, opcode & 0x3f, *m68k_ptr, &ext_count, 0, NULL);
			break;
		case 6:
			fp_src = RA_AllocFPURegister(&ptr);
			ptr = EMIT_LoadFromEffectiveAddress(ptr, 0x80 | 1, &int_src, opcode & 0x3f, *m68k_ptr, &ext_count, 0, NULL);
			break;
		default:
			int_src = RA_AllocARMRegister(&ptr);
			ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
			*ptr++ = fcvtzs_Dto32(int_src, fp_src);
			break;
	}

	fp_dst = RA_MapFPURegisterForWrite(&ptr, fp_dst);

	*ptr++ = add_immed(int_src, int_src, 0x3ff);
	*ptr++ = lsl64(int_src, int_src, 52);

	*ptr++ = bic64_immed(int_src, int_src, 1, 1, 1);
	*ptr++ = mov_reg_to_simd(fp_src, TS_D, 0, int_src);
	*ptr++ = fmuld(fp_dst, fp_dst, fp_src);

	RA_FreeARMRegister(&ptr, int_src);
	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSUB(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FSUB_S")));
uint32_t *EMIT_FSUB_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) __attribute__((alias("EMIT_FSUB_D")));
uint32_t *EMIT_FSUB_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSUB\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;
	uint8_t precision = 0;

	if (opcode2 & 0x0040) {
		if (opcode2 & 0x0004)
			precision = 8;
		else
			precision = 4;
	}

	(void)precision;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fsubd(fp_dst, fp_dst, fp_src);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSGLDIV(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSGLDIV\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fdivd(fp_dst, fp_dst, fp_src);
	*ptr++ = fcvtsd(fp_dst, fp_dst);
	*ptr++ = fcvtds(fp_dst, fp_dst);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}

uint32_t *EMIT_FSGMUL(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	static int shown = 0;
	if (!shown) {
		kprintf("FSGLMUL\n");
		shown = 1;
	}

	uint8_t fp_src = 0xff;
	uint8_t fp_dst = (opcode2 >> 7) & 7;
	uint8_t ext_count = 1;

	ptr = FPU_FetchData(ptr, m68k_ptr, &fp_src, opcode, opcode2, &ext_count, 0);
	fp_dst = RA_MapFPURegister(&ptr, fp_dst);

	*ptr++ = fmuld(fp_dst, fp_dst, fp_src);
	*ptr++ = fcvtsd(fp_dst, fp_dst);
	*ptr++ = fcvtds(fp_dst, fp_dst);

	RA_SetDirtyFPURegister(&ptr, fp_dst);

	RA_FreeFPURegister(&ptr, fp_src);

	ptr = EMIT_AdvancePC(ptr, 2 * (ext_count + 1));
	(*m68k_ptr) += ext_count;

	if (FPSR_Update_Needed(*m68k_ptr, 0)) {

		uint8_t fpsr = RA_ModifyFPSR(&ptr);

		*ptr++ = fcmpzd(fp_dst);
		ptr = EMIT_GetFPUFlags(ptr, fpsr);
	}
	return ptr;
}
/* FPU Instructions, Table 2; bits[5:0]*///bit 6 always assumed 0
static struct FPUFormatDef JumpTableOp[128] = {
	[0x00]		    = { { EMIT_FMOVE_FP }, NULL, FPCR_RND, FPEB }, // the Format should give the last 3 values
	[0x01]		    = { { EMIT_FINT },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x02]		    = { { EMIT_FSINH },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x03]		    = { { EMIT_FINTRZ },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x04]		    = { { EMIT_FSQRT },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x06]		    = { { EMIT_FLOGNP1 },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x08]		    = { { EMIT_FETOXM1 },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x09]		    = { { EMIT_FTANH },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0A]		    = { { EMIT_FATAN },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0B]		    = { { EMIT_FINTRN },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0C]		    = { { EMIT_FASIN },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0D]		    = { { EMIT_FATANH },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0E]		    = { { EMIT_FSIN },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x0F]		    = { { EMIT_FTAN },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x10]		    = { { EMIT_FETOX },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x11]		    = { { EMIT_FTWOTOX },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x12]		    = { { EMIT_FTENTOX },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x13]		    = { { EMIT_FINTRP },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x14]		    = { { EMIT_FLOGN },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x15]		    = { { EMIT_FLOG10 },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x16]		    = { { EMIT_FLOG2 },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x18]		    = { { EMIT_FABS },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x19]		    = { { EMIT_FCOSH },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1A]		    = { { EMIT_FNEG },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1B]		    = { { EMIT_FINTRM },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1C]		    = { { EMIT_FACOS },	NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1D]		    = { { EMIT_FCOS },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1E]		    = { { EMIT_FGETEXP },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x1F]		    = { { EMIT_FGETMAN },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x20]		    = { { EMIT_FDIV },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x21]		    = { { EMIT_FMOD },	 NULL, FPCR_PREC, FPSR_ALL },
	[0x22]		    = { { EMIT_FADD },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x23]		    = { { EMIT_FMUL },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x24]		    = { { EMIT_FSGLDIV },  NULL, 0, FPCC | FPEB },  //single precision
	[0x25]		    = { { EMIT_FREM },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x26]		    = { { EMIT_FSCALE },   NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x27]		    = { { EMIT_FSGMUL },   NULL, 0, (FPCC | FPEB) },  //single precision
	[0x28]		    = { { EMIT_FSUB },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x2D]		    = { { EMIT_FMOD },	 NULL, FPCR_PREC, FPSR_ALL },
	[0x30 ... 0x37] = { { EMIT_FSINCOS },  NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x38]		    = { { EMIT_FCMP },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x3A]		    = { { EMIT_FTST },	 NULL, FPCR_PREC, (FPCC | FPEB) },
	[0x40]		    = { { EMIT_FMOVE_S },  NULL, (FPCR_RND, FPSRB_INEX1) },  //rounded to single
	[0x41]		    = { { EMIT_FSQRT_S },  NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x44]		    = { { EMIT_FMOVE_D },  NULL, FPCR_RND, FPSRB_INEX1 },  //rounded to double
	[0x45]		    = { { EMIT_FSQRT_D },  NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x58]		    = { { EMIT_FABS_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x5A]		    = { { EMIT_FNEG_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x5C]		    = { { EMIT_FABS_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x5E]		    = { { EMIT_FNEG_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x60]		    = { { EMIT_FDIV_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x62]		    = { { EMIT_FADD_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x63]		    = { { EMIT_FMUL_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x64]		    = { { EMIT_FDIV_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x66]		    = { { EMIT_FADD_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x67]		    = { { EMIT_FMUL_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
	[0x68]		    = { { EMIT_FSUB_S },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to single
	[0x6C]		    = { { EMIT_FSUB_D },   NULL, 0, (FPCC | FPSRB_INEX1) },  //rounded to double
};
/* Any format function should preload specified registers according to format and jump to FPU Instruction table. */
uint32_t *EMIT_FORMAT(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **m68k_ptr) {

	if (JumpTableOp[opcode2 & 0x7f].od_Emit)
		ptr = JumpTableOp[opcode2 & 0x7f].od_Emit(ptr, opcode, opcode2, m68k_ptr);
	else {
		ptr = EMIT_FlushPC(ptr);
		ptr = EMIT_InjectDebugString(ptr, "[JIT] opcode %04x at %08x not implemented\n", opcode, *m68k_ptr - 1);
		ptr = EMIT_Exception(ptr, VECTOR_LINE_F, 0);
		*ptr++ = INSN_TO_LE(0xffffffff);
	}
	return ptr;
}
/* opcode2, JumpTables; bits[15:10] */
static struct FPUFMTDef FPU[64] = {
	[000 ...007] = { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 12 },
	[010 ...017] = { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 12 },
	[020 ...021] = { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 4 },
	[024]		= { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 2 },
	[025]		= { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 8 },
	[026]		= { { EMIT_FORMAT }, NULL, 0, 0, 2, 0, 1 },
	[027]		= { { EMIT_FMOVECR }, NULL, 0, 0, 2, 0, 12 },
	[030 ...031] = { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 4 },
	[032 ...033] = { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 12 },
	[034]		= { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 2 },
	[035]		= { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 8 },
	[036]		= { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 1 },
	[037]		= { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 12 },
	[041 ...047] = { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 4 },
	[051 ...057] = { { EMIT_FMOVE }, NULL, 0, 0, 2, 0, 4 },
	[060 ...066] = { { EMIT_FMOVEM }, NULL, 0, 0, 2, 0, 12 },
	[070 ...076] = { { EMIT_FMOVEM }, NULL, 0, 0, 2, 0, 12 }
};

uint32_t *EMIT_FPU(uint32_t *ptr, uint16_t **m68k_ptr, uint16_t *insn_consumed) {

	uint16_t opcode = BE16((*m68k_ptr)[0]);
	uint16_t opcode2 = BE16((*m68k_ptr)[1]);

	if (FPU[opcode2 >> 10 & 077].od_Emit) {
		ptr = FPU[opcode2 >> 10 & 077].od_Emit(ptr, opcode, opcode2, m68k_ptr);
	}
	else {
		ptr = EMIT_FlushPC(ptr);
		ptr = EMIT_InjectDebugString(ptr, "[JIT] opcode %04x at %08x not implemented\n", opcode, *m68k_ptr -1);
		ptr = EMIT_Exception(ptr, VECTOR_LINE_F, 0);//What should be passed to this exception?
		*ptr++ = INSN_TO_LE(0xffffffff);
	}
	return ptr;
}

int M68K_GetLineFLength(uint16_t *insn_stream) {

	uint16_t opcode = cache_read_16(ICACHE, (uintptr_t)insn_stream[0]);
	uint16_t opcode2 = cache_read_16(ICACHE, (uintptr_t)insn_stream[1]);

	uint8_t length = 0;
	uint8_t need_ea = 0;
	uint8_t opsize = 0;

	if(FPU[opcode2 >> 10 & 077].od_Emit) {
		length = FPU[opcode2 & 077].od_BaseLength;
		need_ea = FPU[opcode2 & 077].od_HasEA;
		opsize = FPU[opcode2 & 077].od_OpSize;
	}
	if(need_ea) {
		length += SR_GetEALength(&insn_stream[length], opcode & 077, opsize);
	}
	return length;
}
