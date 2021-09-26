/*
    Copyright © 2019 Michal Schulz <michal.schulz@gmx.de>
    https://github.com/michalsc

    This Source Code Form is subject to the terms of the
    Mozilla Public License, v. 2.0. If a copy of the MPL was not distributed
    with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "support.h"
#include "M68k.h"
#include "EmuFeatures.h"

struct SRMaskEntry {
    uint16_t me_OpcodeMask;
    uint16_t me_Opcode;
    uint8_t  me_BaseLength;
    uint8_t  me_HasEA;
    uint8_t  me_Type;
    uint16_t me_SRNeeds;	/* Consider the full Supervisor register */
    uint16_t me_SRSets;		/* CCR */
    uint8_t  (*me_TestFunction)(uint16_t *stream, uint32_t nest_level);
};

#define SME_MASK    1
#define SME_FUNC    2
#define SME_END     255

#if 0
static uint8_t SR_TestBranch(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcode16B(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcode32B(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcode48B(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcodeEA(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcodeMOVEA(uint16_t *insn_stream, uint32_t nest_level);
static uint8_t SR_TestOpcodeADDA(uint16_t *insn_stream, uint32_t nest_level);
#else
#define SR_TestBranch       NULL
#define SR_TestOpcode16B    NULL
#define SR_TestOpcode32B    NULL
#define SR_TestOpcode48B    NULL
#define SR_TestOpcodeEA     NULL
#define SR_TestOpcodeMOVEA  NULL
#define SR_TestOpcodeADDA   NULL
#endif

static struct SRMaskEntry Line0_Map[] = {
    { 0xffbf, 0x003c, 2, 0, SME_MASK, SR_CCR | SR_S, SR_CCR, NULL },				/* ORI to CCR/SR - needs all flags, sets all flags */
    { 0xff80, 0x0000, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* ORI.B / ORI.W */
    { 0xffc0, 0x0080, 3, 1, SME_MASK, 0, SR_NZVC, NULL },							/* ORI.L */
    { 0xffbf, 0x023c, 2, 0, SME_MASK, SR_CCR | SR_S, SR_CCR, NULL },				/* ANDI to CCR/SR - needs all flags, sets all flags */
    { 0xff80, 0x0200, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* ANDI.B / ANDI.W */
    { 0xffc0, 0x0280, 3, 1, SME_MASK, 0, SR_NZVC, NULL },							/* ANDI.L */
    { 0xff80, 0x0400, 2, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUBI.B / SUBI.W */
    { 0xffc0, 0x0480, 3, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUBI.L */
    { 0xffc0, 0x06c0, 1, 0, SME_MASK, 0, 0, NULL },									/* RTM/CALLM, RTM sets the CCR accordingly */
    { 0xff80, 0x0600, 2, 1, SME_MASK, 0, SR_CCR, NULL },							/* ADDI.B / ADDI.W */
    { 0xffc0, 0x0680, 3, 1, SME_MASK, 0, SR_CCR, NULL },							/* ADDI.L */
    { 0xf9c0, 0x00c0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CHK2/CMP2 */
    { 0xffbf, 0x0a3c, 2, 0, SME_MASK, SR_CCR | SR_S, SR_CCR, NULL },				/* EORI to CCR/SR - they rely on current CC! */
    { 0xff80, 0x0a00, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* EORI.B / EORI.W */
    { 0xffc0, 0x0a80, 3, 1, SME_MASK, 0, SR_NZVC, NULL },							/* EORI.L */
    { 0xff80, 0x0c00, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CMPI.B / CMPI.W */
    { 0xffc0, 0x0c80, 3, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CMPI.L */
    { 0xff00, 0x0800, 1, 1, SME_MASK, 0, SR_Z, NULL },								/* BTST/BSET/BCLR/BCHG - imm */
	{ 0xff80, 0x0e00, 2, 1, SME_MASK, SR_S, 0, NULL },								/* MOVES.B MOVES.W - no effect to CCR flags */
	{ 0xffc0, 0x0e80, 2, 1, SME_MASK, SR_S, 0, NULL },								/* MOVES.L - no effect to CCR flags */
    { 0xf9ff, 0x08fc, 3, 0, SME_MASK, 0, SR_NZVC, NULL },							/* CAS2 */
    { 0xf9c0, 0x08c0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CAS */
    { 0xf100, 0x0100, 2, 1, SME_MASK, 0, SR_Z, NULL },								/* BTST/BSET/BCLR/BCHG - reg */
	{ 0xf138, 0x0108, 2, 0, SME_MASK, 0, 0, NULL },									/* MOVEP.W MOVEP.L - no effect to flags here for completion */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line1_Map[] = {
    { 0xe1c0, 0x2040, 1, 2, SME_FUNC, 0, 0, SR_TestOpcodeMOVEA },					/* MOVEA case - destination is An; Word & Long ONLY!!! */
    { 0xc000, 0x0000, 1, 2, SME_MASK, 0, SR_NZVC, NULL },							/* All other moves change CC */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line2_Map[sizeof(Line1_Map)/sizeof(struct SRMaskEntry)] __attribute__((alias("Line1_Map")));
static struct SRMaskEntry Line3_Map[sizeof(Line1_Map)/sizeof(struct SRMaskEntry)] __attribute__((alias("Line1_Map")));

static struct SRMaskEntry Line4_Map[] = {
    { 0xfdc0, 0x40c0, 1, 1, SME_MASK, SR_CCR | SR_S, 0, NULL },						/* MOVE from CCR/SR *///careful pick which CPU generation is intended
    { 0xff80, 0x4000, 1, 1, SME_MASK, SR_X, SR_CCR, NULL },							/* NEGX.B|.W */
    { 0xffc0, 0x4080, 1, 1, SME_MASK, SR_X, SR_CCR, NULL },							/* NEGX.l */
	{ 0xff80, 0x4200, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CLR.B|.W */
	{ 0xffc0, 0x4280, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CLR.L */
    { 0xffc0, 0x44c0, 1, 1, SME_MASK, 0, SR_CCR | SR_S, NULL },						/* MOVE to CCR */
    { 0xff80, 0x4400, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* NEG.B|.W */
	{ 0xffc0, 0x4480, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* NEG.L */
    { 0xffc0, 0x46c0, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* MOVE to SR *///this one should check other SR_ flags as well
    { 0xff80, 0x4600, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* NOT */
	{ 0xffc0, 0x4680, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* NOT */
    { 0xfeb8, 0x4880, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* EXT/EXTB */
    { 0xfff8, 0x4808, 3, 0, SME_FUNC, 0, 0, SR_TestOpcode48B },						/* LINK.L */
    { 0xffc0, 0x4800, 1, 1, SME_MASK, SR_X, SR_CCR, NULL },							/* NBCD */
    { 0xfff8, 0x4840, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* SWAP */
    { 0xfff8, 0x4848, 1, 0, SME_MASK, SR_CCR, 0, NULL },							/* BKPT */
    { 0xffc0, 0x4840, 1, 1, SME_FUNC, 0, 0, SR_TestOpcodeEA },						/* PEA */
    { 0xffff, 0x4afc, 1, 0, SME_MASK, SR_CCR, 0, NULL },							/* ILLEGAL */
    { 0xffc0, 0x4ac0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* TAS */
	{ 0xff80, 0x4a00, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* TST,B|.W */
	{ 0xffc0, 0x4a80, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* TST.L */
    { 0xffc0, 0x4c00, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* MULU/MULS */
    { 0xffc0, 0x4c40, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* DIVU/DIVS */
    { 0xfff0, 0x4e40, 1, 0, SME_MASK, SR_CCR, SR_S, NULL },							/* TRAP */
    { 0xfff8, 0x4e50, 2, 0, SME_FUNC, 0, 0, SR_TestOpcode32B },						/* LINK.W */
    { 0xfff8, 0x4e58, 1, 0, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* UNLK */
    { 0xfff0, 0x4e60, 1, 0, SME_MASK, SR_CCR | SR_S, 0, NULL },						/* MOVE USP */
    { 0xffff, 0x4e70, 1, 0, SME_MASK, SR_S, 0, NULL },								/* RESET *///Does not effect internal processor state Except PC which is incremented. After a 'grace period' (CPU model dependent), continue execution at next instruction.
    { 0xffff, 0x4e71, 1, 0, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* NOP */
    { 0xffff, 0x4e72, 2, 0, SME_MASK, SR_CCR | SR_S, SR_CCR, NULL },				/* STOP *///The Imm effects the full SR
    { 0xffff, 0x4e73, 1, 0, SME_MASK, SR_S, SR_CCR, NULL },							/* RTE *///Restores SR from (MSP)
    { 0xffff, 0x4e74, 2, 0, SME_MASK, 0, 0, NULL },									/* RTD */
    { 0xffff, 0x4e75, 1, 0, SME_MASK, 0, 0, NULL },									/* RTS */
    { 0xffff, 0x4e76, 1, 0, SME_MASK, SR_V, 0, NULL },								/* TRAPV */
    { 0xffff, 0x4e77, 1, 0, SME_MASK, 0, SR_CCR, NULL },							/* RTR */
    { 0xfffe, 0x4e7a, 2, 0, SME_MASK, SR_CCR | SR_S, 0, NULL },						/* MOVEC */
    { 0xffc0, 0x4e80, 1, 1, SME_MASK, 0, 0, NULL },									/* JSR */
    { 0xffc0, 0x4ec0, 1, 1, SME_MASK, 0, 0, NULL },									/* JMP */
    { 0xfb80, 0x4880, 2, 1, SME_FUNC, 0, 0, SR_TestOpcode32B },						/* MOVEM */
    { 0xf1c0, 0x41c0, 1, 1, SME_FUNC, 0, 0, SR_TestOpcodeEA },						/* LEA */
    { 0xf140, 0x4000, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CHK.B|.W */
	{ 0xf1c0, 0x4100, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CHK.L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line5_Map[] = {
    { 0xf0f8, 0x50c8, 2, 0, SME_MASK, SR_NZVC, 0, NULL },							/* DBcc - subtracting 1 from Dn will eventually trigger SR_Z*/
    { 0xf0ff, 0x50fc, 1, 0, SME_MASK, SR_NZVC, SR_S, NULL },						/* TRAPcc */
    { 0xf0ff, 0x50fa, 2, 0, SME_MASK, SR_NZVC, SR_S, NULL },						/* TRAPcc.W */
    { 0xf0ff, 0x50fb, 2, 0, SME_MASK, SR_NZVC, SR_S, NULL },						/* TRAPcc.L */
    { 0xf0c0, 0x50c0, 1, 1, SME_MASK, SR_NZVC, 0, NULL },							/* Scc */
    { 0xf078, 0x5048, 1, 1, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* SUBQ/ADDQ.W with An */
	{ 0xf0f8, 0x5088, 1, 1, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* SUBQ/ADDQ.L with An */
    { 0xf080, 0x5000, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUBQ/ADDQ .B|.W */
	{ 0xf0c0, 0x5080, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUBQ/ADDQ .L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line6_Map[] = {
    { 0xfeff, 0x6000, 2, 0, SME_FUNC,  0, 0, SR_TestBranch },						/* BRA.W/BSR.W */
    { 0xfeff, 0x60ff, 3, 0, SME_FUNC,  0, 0, SR_TestBranch },						/* BRA.L/BSR.L */
    { 0xfe00, 0x6000, 1, 0, SME_FUNC,  0, 0, SR_TestBranch },						/* BRA.B/BSR.B */
    { 0xfeff, 0x6600, 2, 0, SME_MASK,  SR_Z, 0, NULL },								/* BNE.W/BEQ.W */
    { 0xfeff, 0x66ff, 3, 0, SME_MASK,  SR_Z, 0, NULL },								/* BNE.L/BEQ.L */
    { 0xfe00, 0x6600, 1, 0, SME_MASK,  SR_Z, 0, NULL },								/* BNE.B/BEQ.B */
    { 0xfeff, 0x6400, 2, 0, SME_MASK,  SR_C, 0, NULL },								/* BCC.W/BCS.W */
    { 0xfeff, 0x64ff, 3, 0, SME_MASK,  SR_C, 0, NULL },								/* BCC.L/BCS.L */
    { 0xfe00, 0x6400, 1, 0, SME_MASK,  SR_C, 0, NULL },								/* BCC.B/BCS.B */
    { 0xfeff, 0x6400, 2, 0, SME_MASK,  SR_N, 0, NULL },								/* BPL.W/BMI.W */
    { 0xfeff, 0x64ff, 3, 0, SME_MASK,  SR_N, 0, NULL },								/* BPL.L/BMI.L */
    { 0xfe00, 0x6400, 1, 0, SME_MASK,  SR_N, 0, NULL },								/* BPL.B/BMI.B */
    { 0xfeff, 0x6800, 2, 0, SME_MASK,  SR_V, 0, NULL },								/* BVC.W/BVS.W */
    { 0xfeff, 0x68ff, 3, 0, SME_MASK,  SR_V, 0, NULL },								/* BVC.L/BVS.L */
    { 0xfe00, 0x6800, 1, 0, SME_MASK,  SR_V, 0, NULL },								/* BVC.B/BVS.B */
    { 0xf0ff, 0x6000, 2, 0, SME_MASK,  SR_NZVC, 0, NULL },							/* Bcc.W */
    { 0xf0ff, 0x60ff, 3, 0, SME_MASK,  SR_NZVC, 0, NULL },							/* Bcc.L */
    { 0xf000, 0x6000, 1, 0, SME_MASK,  SR_NZVC, 0, NULL },							/* Bcc.B */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line7_Map[] = {
    { 0xf000, 0x7000, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* MOVEQ */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line8_Map[] = {
    { 0xf1c0, 0x80c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* DIVU.W <ea>,Dn */
    { 0xf1f0, 0x8100, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* SBCD */
    { 0xf1f0, 0x8140, 2, 0, SME_MASK, 0, 0, NULL },									/* PACK */
    { 0xf1f0, 0x8180, 2, 0, SME_MASK, 0, 0, NULL },									/* UNPK */
    { 0xf1c0, 0x81c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* DIVS.W <ea>,Dn */
    { 0xf080, 0x8000, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* OR.(B|W) *///PACK & UNPK are located in this space.
    { 0xf0c0, 0x8080, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* OR.L */
	{ 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry Line9_Map[] = {
    { 0xf0c0, 0x90c0, 1, 1, SME_FUNC, 0, 0, SR_TestOpcodeADDA },					/* SUBA */
    { 0xf1b0, 0x9100, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* SUBX.(B|W) */
	{ 0xf1f0, 0x9180, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* SUBX.L */
    { 0xf080, 0x9000, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUB.(B|W) */
	{ 0xf0c0, 0x9080, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* SUB.L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineA_Map[] = {
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineB_Map[] = {
    { 0xf1b8, 0xb108, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* CMPM.(B|W) */
    { 0xf1f8, 0xb188, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* CMPM.L */
    { 0xf0c0, 0xb0c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CMPA */
    { 0xf180, 0xb000, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CMP.(B|W) */
    { 0xf1c0, 0xb080, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* CMP.L */
    { 0xf180, 0xb100, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* EOR.(B|W) */
	{ 0xf1c0, 0xb180, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* EOR.L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineC_Map[] = {
    { 0xf1c0, 0xc0c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* MULU.W */
    { 0xf1f0, 0xc100, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* ABCD */
    { 0xf1c0, 0xc1c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* MULS.W */
    { 0xf1f0, 0xc140, 1, 0, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* EXG Dx,Dy / EXG Ax,Ay */
    { 0xf1f8, 0xc188, 1, 0, SME_FUNC, 0, 0, SR_TestOpcode16B },						/* EXG Dx,Ay */
    { 0xf080, 0xc000, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* AND.(B|W) */
	{ 0xf0c0, 0xc080, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* AND.L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineD_Map[] = {
    { 0xf0c0, 0xd0c0, 1, 1, SME_FUNC, 0, 0, SR_TestOpcodeADDA },					/* ADDA */
    { 0xf1b0, 0xd100, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* ADDX.(B|W) - reqires X and modifies X! */
    { 0xf1f0, 0xd180, 1, 0, SME_MASK, SR_X, SR_CCR, NULL },							/* ADDX.L */
	{ 0xf080, 0xd000, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* ADD>(B|W) */
	{ 0xf0c0, 0xd080, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* ADD.L */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineE_Map[] = {
    { 0xfec0, 0xe0c0, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* ASL/ASR */
    { 0xfec0, 0xe2c0, 1, 1, SME_MASK, 0, SR_CCR, NULL },							/* LSL/LSR */
    { 0xfec0, 0xe4c0, 1, 1, SME_MASK, SR_X, SR_CCR, NULL },							/* ROXL/ROXR */
    { 0xfec0, 0xe6c0, 1, 1, SME_MASK, 0, SR_NZVC, NULL },							/* ROL/ROR */
    { 0xffc0, 0xe8c0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFTST */
    { 0xffc0, 0xe9c0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFEXTU */
    { 0xffc0, 0xeac0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFCHG */
    { 0xffc0, 0xebc0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFEXTS */
    { 0xffc0, 0xecc0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFCLR */
    { 0xffc0, 0xedc0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFFFO */
    { 0xffc0, 0xeec0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFSET */
    { 0xffc0, 0xefc0, 2, 1, SME_MASK, 0, SR_NZVC, NULL },							/* BFINS */
    { 0xf018, 0xe000, 1, 0, SME_MASK, 0, SR_CCR, NULL },							/* ASL/ASR */
    { 0xf018, 0xe008, 1, 0, SME_MASK, 0, SR_CCR, NULL },							/* LSL/LSR */
    { 0xf018, 0xe010, 1, 0, SME_MASK, SR_X, SR_NZVC, NULL },						/* ROXL/ROXR */
    { 0xf018, 0xe018, 1, 0, SME_MASK, 0, SR_NZVC, NULL },							/* ROL/ROR */
    { 0x0000, 0x0000, 0, 0, SME_END,  0, 0, NULL }
};

static struct SRMaskEntry LineF_Map[] = {
    { 0xf000, 0xf000, 2, 0, SME_MASK, 0, 0, NULL }, /* LineF opcodes do not need these flags, and do not set them */
    { 0x0000, 0x0000, 0, 0, SME_END, 0, 0, NULL }
};

static struct SRMaskEntry *OpcodeMap[16] = {
    Line0_Map,
    Line1_Map,
    Line2_Map,
    Line3_Map,
    Line4_Map,
    Line5_Map,
    Line6_Map,
    Line7_Map,
    Line8_Map,
    Line9_Map,
    LineA_Map,
    LineB_Map,
    LineC_Map,
    LineD_Map,
    LineE_Map,
    LineF_Map
};

static uint8_t SR_GetEALength(uint16_t *insn_stream, uint8_t ea, uint8_t imm_size)
{
    uint8_t word_count = 0;
    uint8_t mode, reg;

    mode = (ea >> 3) & 7;
    reg = ea & 7;

    /* modes 0, 1, 2, 3 and 4 do not have extra words */
    if (mode > 4)
    {
        if (mode == 5)      /* 16-bit offset in next opcode */
            word_count++;
        else if (mode == 6 || (mode == 7 && reg == 3))
        {
            /* Reg- or PC-relative addressing mode */
            uint16_t brief = BE16(insn_stream[0]);

            /* Brief word is here */
            word_count++;

            if (brief & 0x100)
            {
                /* Full brief format */
                switch (brief & 3)
                {
                    case 2:
                        word_count++;       /* Word outer displacement */
                        break;
                    case 3:
                        word_count += 2;    /* Long outer displacement */
                        break;
                }

                switch (brief & 0x30)
                {
                    case 0x20:
                        word_count++;       /* Word base displacement */
                        break;
                    case 0x30:
                        word_count += 2;    /* Long base displacement */
                        break;
                }
            }
        }
        else if (mode == 7)
        {
            if (reg == 2) /* PC-relative with 16-bit offset in next opcode */
                word_count++;
            else if (reg == 0)  /* Absolute word */
                word_count++;
            else if (reg == 1)  /* Absolute long */
                word_count += 2;
            else if (reg == 4)  /* Immediate */
            {
                switch (imm_size)
                {
                    case 1:
                        word_count++;
                        break;
                    case 2:
                        word_count++;
                        break;
                    case 4:
                        word_count+=2;
                        break;
                    case 8:
                        word_count+=4;
                        break;
                    case 12:
                        word_count+=6;
                    default:
                        break;
                }
            }
        }
    }

    return word_count;
}
#if 0
static uint8_t SR_TestOpcodeEA(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t next_opcode;
    uint8_t mask = 0;
    uint8_t word_count;

    /* First calculate the EA length */
    word_count = 1 + SR_GetEALength(insn_stream + 1, BE16(*insn_stream) & 0x3f, 0);

    /* Get the opcode past current 2-byte instruction */
    next_opcode = BE16(insn_stream[word_count]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[word_count], nest_level+1);
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestOpcodeMOVEA(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t opcode;
    uint16_t next_opcode;
    uint8_t mask = 0;
    uint8_t word_count;

    opcode = BE16(insn_stream[0]);

    /* First calculate the EA length */
    switch (opcode & 0x3000)
    {
        case 0x3000:
            word_count = 1 + SR_GetEALength(insn_stream + 1, opcode & 0x3f, 2);
            break;
        case 0x2000:
            word_count = 1 + SR_GetEALength(insn_stream + 1, opcode & 0x3f, 4);
            break;
        default:
            return 0;
    }

    /* Get the opcode past current 2-byte instruction */
    next_opcode = BE16(insn_stream[word_count]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[word_count], nest_level+1);

            break;
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestOpcodeADDA(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t opcode;
    uint16_t next_opcode;
    uint8_t mask = 0;
    uint8_t word_count;

    opcode = BE16(insn_stream[0]);

    /* First calculate the EA length */
    switch (opcode & 0x01c0)
    {
        case 0x00c0:
            word_count = 1 + SR_GetEALength(insn_stream + 1, opcode & 0x3f, 2);
            break;
        case 0x01c0:
            word_count = 1 + SR_GetEALength(insn_stream + 1, opcode & 0x3f, 4);
            break;
        default:
            return 0;
    }

    /* Get the opcode past current 2-byte instruction */
    next_opcode = BE16(insn_stream[word_count]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[word_count], nest_level+1);

            break;
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestOpcode16B(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t next_opcode;
    uint8_t mask = 0;

    /* Get the opcode past current 2-byte instruction */
    next_opcode = BE16(insn_stream[1]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[1], nest_level+1);
            break;
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestOpcode32B(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t next_opcode;
    uint8_t mask = 0;

    /* Get the opcode past current 4-byte instruction */
    next_opcode = BE16(insn_stream[2]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[2], nest_level+1);
            break;
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestOpcode48B(uint16_t *insn_stream, uint32_t nest_level)
{
    uint16_t next_opcode;
    uint8_t mask = 0;

    /* Get the opcode past current 4-byte instruction */
    next_opcode = BE16(insn_stream[3]);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(&insn_stream[3], nest_level+1);
            break;
        }
        e++;
    }

    return mask;
}

static uint8_t SR_TestBranch(uint16_t *insn_stream, uint32_t nest_level)
{
    /*
        At this point insn_stream points to the branch opcode.
        Check what's at the target of this branch
    */
    uint8_t mask = 0;
    uint16_t opcode = BE16(*insn_stream);
    int32_t bra_off = 0;

    /* Advance stream 1 word past BRA */
    insn_stream++;

    /* use 16-bit offset */
    if ((opcode & 0x00ff) == 0x00)
    {
        bra_off = (int16_t)(BE16(insn_stream[0]));
    }
    /* use 32-bit offset */
    else if ((opcode & 0x00ff) == 0xff)
    {
        bra_off = (int32_t)(BE32(*(uint32_t*)insn_stream));
    }
    else
    /* otherwise use 8-bit offset */
    {
        bra_off = (int8_t)(opcode & 0xff);
    }

    /* Advance instruction stream accordingly */
    insn_stream = (uint16_t *)((intptr_t)insn_stream + bra_off);

    /* Fetch new opcode and test it */
    uint16_t next_opcode = BE16(*insn_stream);

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[next_opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((next_opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Don't nest. Check only the SME_MASK type */
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC && nest_level < Options.M68K_TRANSLATION_DEPTH)
                mask = e->me_TestFunction(insn_stream, nest_level+1);
            break;
        }
        e++;
    }

    return mask;
}

/* Get the mask of status flags changed by the instruction specified by the opcode */
uint8_t M68K_GetSRMask(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    uint8_t mask = 0;

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[opcode >> 12];

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            if (e->me_Type == SME_MASK)
                mask = e->me_SRMask;
            else if (e->me_Type == SME_FUNC) {
                mask = e->me_TestFunction(insn_stream, 0);
            }
            break;
        }
        e++;
    }

    return mask;
}

#endif

static int M68K_GetLine0Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 0;
    int opsize = 4;

    /* 00000000xxxxxxxx - ORI to CCR, ORI to SR, ORI */
    if ((opcode & 0xff00) == 0x0000 && (opcode & 0x00c0) != 0x00c0)   /* 00000000xxxxxxxx - ORI to CCR, ORI to SR, ORI */
    {
        if (
            (opcode & 0x00ff) == 0x003c ||
            (opcode & 0x00ff) == 0x007c
        ) 
        {
            length = 2;
            need_ea = 0;
        }
        else
        {
            need_ea = 1;
            switch (opcode & 0x00c0)
            {
                case 0x0000:
                    opsize = 1;
                    length = 2;
                    break;
                case 0x0040:
                    opsize = 2;
                    length = 2;
                    break;
                case 0x0080:
                    opsize = 4;
                    length = 3;
                    break;
            }
        }
    }
    /* 00000010xxxxxxxx - ANDI to CCR, ANDI to SR, ANDI */
    else if ((opcode & 0xff00) == 0x0200)   
    {
        if (
            (opcode & 0x00ff) == 0x003c ||
            (opcode & 0x00ff) == 0x007c
        )
        {
            length = 2;
            need_ea = 0;
        }
        else
        {
            need_ea = 1;
            switch (opcode & 0x00c0)
            {
                case 0x0000:
                    opsize = 1;
                    length = 2;
                    break;
                case 0x0040:
                    opsize = 2;
                    length = 2;
                    break;
                case 0x0080:
                    opsize = 4;
                    length = 3;
                    break;
            }
        }   
    }
    /* 00000100xxxxxxxx - SUBI */
    else if ((opcode & 0xff00) == 0x0400)   
    {
        need_ea = 1;
        switch (opcode & 0x00c0)
        {
            case 0x0000:
                opsize = 1;
                length = 2;
                break;
            case 0x0040:
                opsize = 2;
                length = 2;
                break;
            case 0x0080:
                opsize = 4;
                length = 3;
                break;
        }
    }
    /* 00000110xxxxxxxx - ADDI */
    else if ((opcode & 0xff00) == 0x0600 && (opcode & 0x00c0) != 0x00c0)   
    {
        need_ea = 1;
        switch (opcode & 0x00c0)
        {
            case 0x0000:
                opsize = 1;
                length = 2;
                break;
            case 0x0040:
                opsize = 2;
                length = 2;
                break;
            case 0x0080:
                opsize = 4;
                length = 3;
                break;
        }
    }
    /* 00000xx011xxxxxx - CMP2, CHK2 */
    else if ((opcode & 0xf9c0) == 0x00c0)   
    {
        length = 2;
        need_ea = 1;
        opsize = 0;
    }
    /* 00001010xxxxxxxx - EORI to CCR, EORI to SR, EORI */
    else if ((opcode & 0xff00) == 0x0a00)   
    {
        if (
            (opcode & 0x00ff) == 0x003c ||
            (opcode & 0x00ff) == 0x007c
        )
        {
            length = 2;
            need_ea = 0;
        }
        else
        {
            need_ea = 1;
            switch (opcode & 0x00c0)
            {
                case 0x0000:
                    opsize = 1;
                    length = 2;
                    break;
                case 0x0040:
                    opsize = 2;
                    length = 2;
                    break;
                case 0x0080:
                    opsize = 4;
                    length = 3;
                    break;
            }
        }   
    }
    /* 00001100xxxxxxxx - CMPI */
    else if ((opcode & 0xff00) == 0x0c00)   
    {
        need_ea = 1;
        switch (opcode & 0x00c0)
        {
            case 0x0000:
                opsize = 1;
                length = 2;
                break;
            case 0x0040:
                opsize = 2;
                length = 2;
                break;
            case 0x0080:
                opsize = 4;
                length = 3;
                break;
        }
    }
    else if ((opcode & 0xffc0) == 0x0800)   /* 0000100000xxxxxx - BTST */
    {
        length = 2;
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xffc0) == 0x0840)   /* 0000100001xxxxxx - BCHG */
    {
        length = 2;
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xffc0) == 0x0880)   /* 0000100010xxxxxx - BCLR */
    {
        length = 2;
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xffc0) == 0x08c0)   /* 0000100011xxxxxx - BSET */
    {
        length = 2;
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xff00) == 0x0e00)   /* 00001110xxxxxxxx - MOVES */
    {
        length = 2;
        need_ea = 1;
        opsize = 0;
    }
    else if ((opcode & 0xf9c0) == 0x08c0)   /* 00001xx011xxxxxx - CAS, CAS2 */
    {
        need_ea = 1;
        opsize = 0;
        if ((opcode & 0x3f) == 0x3c)
        {
            length = 3;
        }
        else
        {
            length = 2;
        }
    }
    else if ((opcode & 0xf1c0) == 0x0100)   /* 0000xxx100xxxxxx - BTST */
    {
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xf1c0) == 0x0140)   /* 0000xxx101xxxxxx - BCHG */
    {
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xf1c0) == 0x0180)   /* 0000xxx110xxxxxx - BCLR */
    {
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xf1c0) == 0x01c0)   /* 0000xxx111xxxxxx - BSET */
    {
        need_ea = 1;
        opsize = 1;
    }
    else if ((opcode & 0xf038) == 0x0008)   /* 0000xxxxxx001xxx - MOVEP */
    {
        need_ea = 0;
        length = 2;
    }

    if (need_ea)
    {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLineELength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 0;

    if (
        (opcode & 0xf8c0) == 0xe0c0     // memory shift/rotate
    )
    {
        length = 1;
        need_ea = 1;
    }
    else if (
        (opcode & 0xf8c0) == 0xe8c0     // bf* instructions
    )
    {
        length = 2;
        need_ea = 1;
    }
    else
    {
        length = 1;
        need_ea = 0;
    }

    if (need_ea)
    {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, 0);
    }

    return length;
}

int M68K_GetLine6Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    
    if ((opcode & 0xff) == 0) {
        length = 2;
    }
    else if ((opcode & 0xff) == 0xff) {
        length = 3;
    }

    return length;
}

int M68K_GetLine8Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    if (
        (opcode & 0xf0c0) == 0x80c0     // div word size
    )
    {
        length = 1;
        opsize = 2;
        need_ea = 1;
    }
    else if (
        (opcode & 0xf1f0) == 0x8100     // sbcd
    )
    {
        length = 1;
        need_ea = 0;
    }
    else if (
        (opcode & 0xf1f0) == 0x8140 ||  // pack
        (opcode & 0xf1f0) == 0x8180     // unpk
    )
    {
        length = 2;
        need_ea = 0;
    }
    else {
        need_ea = 1;
        opsize = 1 << ((opcode >> 6) & 3);
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLine9Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* SUBA */
    if ((opcode & 0xf0c0) == 0x90c0)
    {
        opsize = (opcode & 0x0100) == 0x0100 ? 4 : 2;
    }
    /* SUBX */
    else if ((opcode & 0xf130) == 0x9100)
    {
        need_ea = 0;
    }
    /* SUB */
    else
    {
        opsize = 1 << ((opcode >> 6) & 3);
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLineBLength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* 1011xxxx11xxxxxx - CMPA */
    if ((opcode & 0xf0c0) == 0xb0c0)
    {
        opsize = ((opcode >> 8) & 1) ? 4 : 2;
    }
    /* 1011xxx1xx001xxx - CMPM */
    else if ((opcode & 0xf138) == 0xb108)
    {
        need_ea = 0;
    }
    /* 1011xxx0xxxxxxxx - CMP */
    else if ((opcode & 0xf100) == 0xb000)
    {
        opsize = 1 << ((opcode >> 6) & 3);
    }
    /* 1011xxxxxxxxxxxx - EOR */
    else if ((opcode & 0xf000) == 0xb000)
    {
        opsize = 1 << ((opcode >> 6) & 3);
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLineCLength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* 1100xxx011xxxxxx - MULU */
    if ((opcode & 0xf1c0) == 0xc0c0)
    {
        opsize = 2;
        need_ea = 1;
    }
    /* 1100xxx10000xxxx - ABCD */
    else if ((opcode & 0xf1f0) == 0xc100)
    {
        need_ea = 0;
    }
    /* 1100xxx111xxxxxx - MULS */
    else if ((opcode & 0xf1c0) == 0xc1c0)
    {
        opsize = 2;
        need_ea = 1;
    }
    /* 1100xxx1xx00xxxx - EXG */
    else if ((opcode & 0xf130) == 0xc100)
    {
        need_ea = 0;
    }
    /* 1100xxxxxxxxxxxx - AND */
    else if ((opcode & 0xf000) == 0xc000)
    {
        opsize = 1 << ((opcode >> 6) & 3);
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLineDLength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* ADDA */
    if ((opcode & 0xf0c0) == 0xd0c0)
    {
        opsize = (opcode & 0x0100) == 0x0100 ? 4 : 2;
    }
    /* ADDX */
    else if ((opcode & 0xf130) == 0xd100)
    {
        need_ea = 0;
    }
    /* ADD */
    else
    {
        opsize = 1 << ((opcode >> 6) & 3);
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLine5Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* Scc/TRAPcc/DBcc */
    if ((opcode & 0xf0c0) == 0x50c0)
    {
        /* DBcc */
        if ((opcode & 0x38) == 0x08)
        {
            length = 2;
            need_ea = 0;
        }
        /* TRAPcc */
        else if ((opcode & 0x38) == 0x38)
        {
            need_ea = 0;
            switch (opcode & 7)
            {
                case 4:
                    length = 1;
                    break;
                case 2:
                    length = 2;
                    break;
                case 3:
                    length = 3;
                    break;
            }
        }
        /* Scc */
        else
        {
            need_ea = 1;
            opsize = 1;
        }   
    }
    /* SUBQ */
    else if ((opcode & 0xf100) == 0x5100)
    {
        need_ea = 1;
        switch ((opcode >> 6) & 3)
        {
            case 0:
                opsize = 1;
                break;
            case 1:
                opsize = 2;
                break;
            case 2:
                opsize = 4;
                break;
        }
    }
    /* ADDQ */
    else if ((opcode & 0xf100) == 0x5000)
    {
        need_ea = 1;
        switch ((opcode >> 6) & 3)
        {
            case 0:
                opsize = 1;
                break;
            case 1:
                opsize = 2;
                break;
            case 2:
                opsize = 4;
                break;
        }
    }  

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

int M68K_GetLine4Length(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int length = 1;
    int need_ea = 1;
    int opsize = 2;

    /* 0100000011xxxxxx - MOVE from SR */
    if ((opcode & 0xffc0) == 0x40c0)
    {
        need_ea = 1;
        opsize = 2;
    }
    /* 0100001011xxxxxx - MOVE from CCR */
    else if ((opcode &0xffc0) == 0x42c0)
    {
        need_ea = 1;
        opsize = 2;
    }
    /* 01000000ssxxxxxx - NEGX */
    else if ((opcode & 0xff00) == 0x4000 && (opcode & 0xc0) != 0xc0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 01000010ssxxxxxx - CLR */
    else if ((opcode & 0xff00) == 0x4200 && (opcode & 0xc0) != 0xc0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100010011xxxxxx - MOVE to CCR */
    else if ((opcode &0xffc0) == 0x44c0)
    {
        need_ea = 1;
        opsize = 2;
    }
    /* 01000100ssxxxxxx - NEG */
    else if ((opcode &0xff00) == 0x4400 && (opcode & 0xc0) != 0xc0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100011011xxxxxx - MOVE to SR */
    else if ((opcode &0xffc0) == 0x46c0)
    {
        need_ea = 1;
        opsize = 2;
    }
    /* 01000110ssxxxxxx - NOT */
    else if ((opcode &0xff00) == 0x4600 && (opcode & 0xc0) != 0xc0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100100xxx000xxx - EXT, EXTB */
    else if ((opcode & 0xfeb8) == 0x4880)
    {
        need_ea = 0;
    }
    /* 0100100000001xxx - LINK - 32 bit offset */
    else if ((opcode & 0xfff8) == 0x4808)
    {
        need_ea = 0;
        length = 3;
    }
    /* 0100100000xxxxxx - NBCD */
    else if ((opcode & 0xffc0) == 0x4800 && (opcode & 0x08) != 0x08)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100100001000xxx - SWAP */
    else if ((opcode & 0xfff8) == 0x4840)
    {
        need_ea = 0;
    }
    /* 0100100001001xxx - BKPT */
    else if ((opcode & 0xfff8) == 0x4848)
    {
        need_ea = 0;
    }
    /* 0100100001xxxxxx - PEA */
    else if ((opcode & 0xffc0) == 0x4840 && (opcode & 0x38) != 0x08)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100101011111100 - ILLEGAL */
    else if (opcode == 0x4afc)
    {
        need_ea = 0;
    }
    /* 0100101011xxxxxx - TAS */
    else if ((opcode & 0xffc0) == 0x4ac0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100101011xxxxxx - TST */
    else if ((opcode & 0xff00) == 0x4a00 && (opcode & 0xc0) != 0xc0)
    {
        need_ea = 1;
        switch (opcode & 0x00c0)
        {
            case 0x0000:    /* Byte operation */
                opsize = 1;
                break;
            case 0x0040:    /* Short operation */
                opsize = 2;
                break;
            case 0x0080:    /* Long operation */
                opsize = 4;
                break;
        }
    }
    /* 0100110000xxxxxx - MULU, MULS, DIVU, DIVUL, DIVS, DIVSL */
    else if ((opcode & 0xff80) == 0x4c00 || (opcode == 0x83c0))
    {
        length = 2;
        opsize = 4;
        need_ea = 1;
    }
    /* 010011100100xxxx - TRAP */
    else if ((opcode & 0xfff0) == 0x4e40)
    {
        need_ea = 0;
    }
    /* 0100111001010xxx - LINK */
    else if ((opcode & 0xfff8) == 0x4e50)
    {
        need_ea = 0;
        length = 2;
    }
    /* 0100111001011xxx - UNLK */
    else if ((opcode & 0xfff8) == 0x4e58)
    {
        need_ea = 0;
    }
    /* 010011100110xxxx - MOVE USP */
    else if ((opcode & 0xfff0) == 0x4e60)
    {
        need_ea = 0;
    }
    /* 0100111001110000 - RESET */
    else if (opcode == 0x4e70)
    {
        need_ea = 0;
    }
    /* 0100111001110000 - NOP */
    else if (opcode == 0x4e71)
    {
        need_ea = 0;
    }
    /* 0100111001110010 - STOP */
    else if (opcode == 0x4e72)
    {
        need_ea = 0;
        length = 2;
    }
    /* 0100111001110011 - RTE */
    else if (opcode == 0x4e73)
    {
        need_ea = 0;
    }
    /* 0100111001110100 - RTD */
    else if (opcode == 0x4e74)
    {
        need_ea = 0;
        length = 2;
    }
    /* 0100111001110101 - RTS */
    else if (opcode == 0x4e75)
    {
        need_ea = 0;
    }
    /* 0100111001110110 - TRAPV */
    else if (opcode == 0x4e76)
    {
        need_ea = 0;
    }
    /* 0100111001110111 - RTR */
    else if (opcode == 0x4e77)
    {
        need_ea = 0;
    }
    /* 010011100111101x - MOVEC */
    else if ((opcode & 0xfffe) == 0x4e7a)
    {
        need_ea = 0;
        length = 2;
    }
    /* 0100111010xxxxxx - JSR */
    else if ((opcode & 0xffc0) == 0x4e80)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100111011xxxxxx - JMP */
    else if ((opcode & 0xffc0) == 0x4ec0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 01001x001xxxxxxx - MOVEM */
    else if ((opcode & 0xfb80) == 0x4880)
    {
        need_ea = 0;
        length = 2;
    }
    /* 0100xxx111xxxxxx - LEA */
    else if ((opcode & 0xf1c0) == 0x41c0)
    {
        need_ea = 1;
        opsize = 0;
    }
    /* 0100xxx1x0xxxxxx - CHK */
    else if ((opcode & 0xf140) == 0x4100)
    {
        need_ea = 1;
        opsize = (opcode & 0x80) ? 2 : 4;
    }

    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize);
    }

    return length;
}

/* Check if opcode is of branch kind or may result in a */
int M68K_IsBranch(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);

    if (
        opcode == 0x007c            ||
        opcode == 0x027c            ||
        opcode == 0x0a7c            ||
        (opcode & 0xffc0) == 0x40c0 ||
        (opcode & 0xffc0) == 0x46c0 ||
        (opcode & 0xfff8) == 0x4848 ||
        opcode == 0x4afc            ||
        (opcode & 0xfff0) == 0x4e40 ||
        (opcode & 0xfff0) == 0x4e60 ||
        opcode == 0x4e70            ||
        opcode == 0x4e72            ||
        opcode == 0x4e73            ||
        opcode == 0x4e74            ||
        opcode == 0x4e75            ||
        opcode == 0x4e76            ||
        opcode == 0x4e77            ||
        (opcode & 0xfffe) == 0x4e7a ||
        (opcode & 0xff80) == 0x4e80 ||
        (opcode & 0xf0f8) == 0x50c8 ||
        (opcode & 0xf000) == 0x6000
    )
        return 1;
    else
        return 0;
}

int M68K_GetMoveLength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int size = 0;
    int length = 1;
    uint8_t ea = opcode & 0x3f;

    if ((opcode & 0x3000) == 0x1000)
        size = 1;
    else if ((opcode & 0x3000) == 0x2000)
        size = 4;
    else
        size = 2;

    length += SR_GetEALength(&insn_stream[length], ea & 0x3f, size);

    ea = (opcode >> 3) & 0x38;
    ea |= (opcode >> 9) & 0x7;

    length += SR_GetEALength(&insn_stream[length], ea, size);

    return length;    
}

int M68K_GetLineFLength(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(insn_stream[0]);
    uint16_t opcode2 = BE16(insn_stream[1]);
    int length = 1;
    int need_ea = 0;
    int opsize = 0;

    /* MOVE16 (Ax)+, (Ay)+ */
    if ((opcode & 0xfff8) == 0xf620 && (opcode2 & 0x8fff) == 0x8000)
    {
        length = 2;
    }
    /* MOVE16 other variations */
    else if ((opcode & 0xffe0) == 0xf600)
    {
        length = 3;
    }
    /* CINV */
    else if ((opcode & 0xff20) == 0xf400)
    {
        length = 1;
    }
    /* CPUSH */
    else if ((opcode & 0xff20) == 0xf420)
    {
        length = 1;
    }
    /* FMOVECR reg */
    else if (opcode == 0xf200 && (opcode2 & 0xfc00) == 0x5c00)
    {
        length = 2;
    }
    /* FABS */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0018 || (opcode2 & 0xa07b) == 0x0058))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FADD */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0022 || (opcode2 & 0xa07b) == 0x0062))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FBcc */
    else if ((opcode & 0xff80) == 0xf280)
    {
        if (opcode & (1 << 6))
        {
            length = 3;
        }
        else
        {
            length = 2;
        }
    }
    /* FCMP */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x0038)
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FDIV */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0020 || (opcode2 & 0xa07b) == 0x0060))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FINT */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x0001)
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FINTRZ */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x0003)
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FLOGN */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x0014)
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FMOVE to REG */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0000 || (opcode2 & 0xa07b) == 0x0040))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FMOVE to MEM */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xe07f) == 0x6000)
    {
        length = 2;
        need_ea = 1;
        switch ((opcode2 >> 10) & 7)
        {
            case 0:
            case 1:
                opsize = 2;
                break;
            
            case 2:
                opsize = 6;
                break;
            
            case 3:     // Packed!!
            case 7:
                opsize = 6;
                break;
            
            case 4:
            case 6:
                opsize = 1;
                break;

            case 5:
                opsize = 4;
                break;
        }
    }
    /* FMOVE from special */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xe3ff) == 0xa000)
    {
        length = 2;
        need_ea = 1;
        opsize = 2;
    }
    /* FMOVE to special */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xe3ff) == 0x8000)
    {
        length = 2;
        need_ea = 1;
        opsize = 2;
    }
    /* FMOVEM */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xc700) == 0xc000)
    {
        length = 2;
        need_ea = 1;
    }
    /* FMUL */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0023 || (opcode2 & 0xa07b) == 0x0063))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FNEG */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x001a || (opcode2 & 0xa07b) == 0x005a))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FTST */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x003a)
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FScc */
    else if ((opcode & 0xffc0) == 0xf240 && (opcode2 & 0xffc0) == 0)
    {
        need_ea = 1;
        opsize = 1;
        length = 2;
    }
    /* FSQRT */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0004 || (opcode2 & 0xa07b) == 0x0041))
    {
        length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FSUB */
    else if ((opcode & 0xffc0) == 0xf200 && ((opcode2 & 0xa07f) == 0x0028 || (opcode2 & 0xa07b) == 0x0068))
    {
                length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FSIN */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x000e)
    {
                length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FCOS */
    else if ((opcode & 0xffc0) == 0xf200 && (opcode2 & 0xa07f) == 0x001d)
    {
                length = 2;
        if (opcode2 & 0x4000)
        {
            need_ea = 1;
            switch ((opcode2 >> 10) & 7)
            {
                case 0:
                case 1:
                    opsize = 2;
                    break;
                
                case 2:
                    opsize = 6;
                    break;
                
                case 3:     // Packed!!
                    opsize = 6;
                    break;
                
                case 4:
                case 6:
                    opsize = 1;
                    break;

                case 5:
                    opsize = 4;
                    break;
            }
        }
    }
    /* FNOP */
    else if (opcode == 0xf280 && opcode2 == 0)
    {
        length = 2;
    }
    else
    {
        length = 2;
    }


    if (need_ea) {
        length += SR_GetEALength(&insn_stream[length], opcode & 0x3f, opsize * 2);
    }

    return length;
}

/* Get number of 16-bit words this instruction occupies */
int M68K_GetINSNLength(uint16_t *insn_stream)
{
    uint16_t opcode = *insn_stream;
    int length = 0;

    switch(opcode & 0xf000)
    {
        case 0x0000:
            length = M68K_GetLine0Length(insn_stream);
            break;
        case 0x1000: /* Fallthrough */
        case 0x2000: /* Fallthrough */
        case 0x3000:
            length = M68K_GetMoveLength(insn_stream);
            break;
        case 0x4000:
            length = M68K_GetLine4Length(insn_stream);
            break;
        case 0x5000:
            length = M68K_GetLine5Length(insn_stream);
            break;
        case 0x6000:
            length = M68K_GetLine6Length(insn_stream);
            break;
        case 0x7000:
            length = 1;
            break;
        case 0x8000:
            length = M68K_GetLine8Length(insn_stream);
            break;
        case 0x9000:
            length = M68K_GetLine9Length(insn_stream);
            break;
        case 0xa000:
            length = 1;
            break;
        case 0xb000:
            length = M68K_GetLineBLength(insn_stream);
            break;
        case 0xc000:
            length = M68K_GetLineCLength(insn_stream);
            break;
        case 0xd000:
            length = M68K_GetLineDLength(insn_stream);
            break;
        case 0xe000:
            length = M68K_GetLineELength(insn_stream);
            break;
        case 0xf000:
            length = M68K_GetLineFLength(insn_stream);
            break;
        default:
            break;
    }

//    kprintf(" = %d\n", length);

    return length;
}

#define D(x) /* x */

/* Get the mask of status flags changed by the instruction specified by the opcode */
uint8_t M68K_GetSRMask(uint16_t *insn_stream)
{
    uint16_t opcode = BE16(*insn_stream);
    int scan_depth = 0;
    const int max_scan_depth = 20;
    uint8_t mask = 0;
    uint8_t needed = 0;
    int found = 0;

    /* Fetch correct table baset on bits 12..15 of the opcode */
    struct SRMaskEntry *e = OpcodeMap[opcode >> 12];

    D(kprintf("[JIT] GetSRMask, opcode %04x @ %08x, ", opcode, insn_stream));

    /* Search within table until SME_END is found */
    while (e->me_Type != SME_END)
    {
        if ((opcode & e->me_OpcodeMask) == e->me_Opcode)
        {
            /* Opcode found. Check which flags it sets */
            found = 1;
            mask = e->me_SRSets;
            break;
        }
        e++;
    }

    if (!found) {
        D(kprintf("opcode not found!\n"));
        return 0;
    }

    D(kprintf(" SRNeeds = %x, SRSets = %x\n", e->me_SRNeeds, mask));

    /*
        Check as long as there are still some flags to be set by the opcode and the depth
        of scan is not exceeded
    */
    while(mask && scan_depth < max_scan_depth)
    {
        /* Increase scan depth level */
        scan_depth++;

        /* If instruction is a branch break the scan */
        if (M68K_IsBranch(insn_stream))
        {
            /* Check if BRA/BSR and follow if possible */
            if ((opcode & 0xfe00) == 0x6000)
            {
                int32_t branch_offset = (int8_t)(opcode & 0xff);

                if ((opcode & 0xff) == 0) {
                    branch_offset = (int16_t)BE16(insn_stream[1]);
                } else if ((opcode & 0xff) == 0xff) {
                    uint16_t lo16, hi16;
                    hi16 = BE16(insn_stream[1]);
                    lo16 = BE16(insn_stream[2]);
                    branch_offset = lo16 | (hi16 << 16);
                }

                insn_stream = insn_stream + 1 + (branch_offset >> 1);

                D(kprintf("[JIT]   %02d: PC-relative jump by %d bytes to %08x\n", scan_depth, branch_offset, insn_stream));
            }
            /* Check if JMP/JSR and follow if possible */
            else if ((opcode & 0xffbe) == 0x4eb8)
            {
                if (opcode & 1) {
                    uint16_t lo16, hi16;
                    hi16 = BE16(insn_stream[1]);
                    lo16 = BE16(insn_stream[2]);
                    insn_stream = (uint16_t*)(uintptr_t)(lo16 | (hi16 << 16));
                } else {
                    insn_stream = (uint16_t*)(uintptr_t)((uint32_t)BE16(insn_stream[1]));
                }

                D(kprintf("[JIT]   %02d: Absolute jump to %08x\n", scan_depth, insn_stream));
            }
            else if ((opcode & 0xf000) == 0x6000)
            {
                int32_t branch_offset = (int8_t)(opcode & 0xff);
                uint16_t *insn_stream_2 = insn_stream + 1;
                uint8_t condition = (opcode >> 9) & 7;
                // List of masks which the condition code needs by itself
                const uint8_t masks[] = {
                    0,                  // T, F
                    SR_C | SR_Z,        // HI, LS
                    SR_C,               // CC, CS
                    SR_Z,               // NE, EQ
                    SR_V,               // VC, VS
                    SR_N,               // MI, PL
                    SR_N | SR_V,        // GE, LT
                    SR_N | SR_V | SR_Z  // GT, LE
                };

                // Mark the flags which conditional jump needs by itself
                needed |= mask & masks[condition];

                if ((opcode & 0xff) == 0) {
                    branch_offset = (int16_t)BE16(insn_stream[1]);
                    insn_stream_2++;
                } else if ((opcode & 0xff) == 0xff) {
                    uint16_t lo16, hi16;
                    hi16 = BE16(insn_stream[1]);
                    lo16 = BE16(insn_stream[2]);
                    branch_offset = lo16 | (hi16 << 16);
                    insn_stream_2+=2;
                }

                insn_stream = insn_stream + 1 + (branch_offset >> 1);

                D(kprintf("[JIT]   %02d: Splitting into two paths %08x and %08x\n", scan_depth, insn_stream, insn_stream_2));

                uint8_t mask1 = mask;
                uint8_t mask2 = mask;
                uint8_t needed1 = needed;
                uint8_t needed2 = needed;
                scan_depth = max_scan_depth - 1 - (max_scan_depth - scan_depth) / 2;
                int scan_depth_tmp = scan_depth;

                while(mask1 && scan_depth < max_scan_depth)
                {
                    scan_depth++;

                    /* If instruction is a branch break the scan */
                    if (M68K_IsBranch(insn_stream))
                        break;

                    /* Get opcode */
                    opcode = BE16(*insn_stream);

                    D(kprintf("[JIT]   %02d.1: opcode=%04x @ %08x ", scan_depth, opcode, insn_stream));

                    e = OpcodeMap[opcode >> 12];
                    found = 0;

                    /* Search within table until SME_END is found */
                    while (e->me_Type != SME_END)
                    {
                        if ((opcode & e->me_OpcodeMask) == e->me_Opcode)
                        {
                            found = 1;
                            D(kprintf("SRneeds=%x, SRSets=%x\n", e->me_SRNeeds, e->me_SRSets));

                            /* If instruction *needs* one of flags from current opcode, break the check and return mask */
                            if (mask1 & e->me_SRNeeds) {
                                needed1 |= (mask1 & e->me_SRNeeds);
                            }

                            /* Clear flags which this instruction sets */
                            mask1 = mask1 & ~e->me_SRSets;
                            
                            break;
                        }

                        e++;
                    }

                    if (!found)
                    {
                        D(kprintf("opcode not found!\n"));
                        break;
                    }

                    /* Advance to subsequent instruction */
                    insn_stream += M68K_GetINSNLength(insn_stream);
                }

                scan_depth = scan_depth_tmp;

                while(mask2 && scan_depth < max_scan_depth)
                {
                    scan_depth++;

                    /* If instruction is a branch break the scan */
                    if (M68K_IsBranch(insn_stream_2))
                        break;

                    /* Get opcode */
                    opcode = BE16(*insn_stream_2);

                    D(kprintf("[JIT]   %02d.2: opcode=%04x @ %08x ", scan_depth, opcode, insn_stream_2));

                    e = OpcodeMap[opcode >> 12];
                    found = 0;

                    /* Search within table until SME_END is found */
                    while (e->me_Type != SME_END)
                    {
                        if ((opcode & e->me_OpcodeMask) == e->me_Opcode)
                        {
                            found = 1;
                            D(kprintf("SRneeds=%x, SRSets=%x\n", e->me_SRNeeds, e->me_SRSets));

                            /* If instruction *needs* one of flags from current opcode, break the check and return mask */
                            if (mask2 & e->me_SRNeeds) {
                                needed2 |= (mask2 & e->me_SRNeeds);
                            }

                            /* Clear flags which this instruction sets */
                            mask2 = mask2 & ~e->me_SRSets;
                            
                            break;
                        }

                        e++;
                    }

                    if (!found)
                    {
                        D(kprintf("opcode not found!\n"));
                        break;
                    }

                    /* Advance to subsequent instruction */
                    insn_stream_2 += M68K_GetINSNLength(insn_stream_2);
                }

                D(kprintf("[JIT]   joining masks %x and %x to %x\n", mask1 | needed1, mask2 | needed2, mask1 | needed1 | mask2 | needed2));

                return mask1 | needed1 | mask2 | needed2;
            }
            else 
            {
                D(kprintf("[JIT]   %02d: check breaks on branch\n", scan_depth));
                break;
            }
        }
        else          
        {
            /* Advance to subsequent instruction */
            insn_stream += M68K_GetINSNLength(insn_stream);
        }
        
        /* Get opcode */
        opcode = BE16(*insn_stream);
        D(kprintf("[JIT]   %02d: opcode=%04x @ %08x ", scan_depth, opcode, insn_stream));

        e = OpcodeMap[opcode >> 12];
        found = 0;

        /* Search within table until SME_END is found */
        while (e->me_Type != SME_END)
        {
            if ((opcode & e->me_OpcodeMask) == e->me_Opcode)
            {
                found = 1;
                D(kprintf("SRneeds=%x, SRSets=%x\n", e->me_SRNeeds, e->me_SRSets));

                /* If instruction *needs* one of flags from current opcode, break the check and return mask */
                if (mask & e->me_SRNeeds) {
                    needed |= (mask & e->me_SRNeeds);
                }

                /* Clear flags which this instruction sets */
                mask = mask & ~e->me_SRSets;
                
                break;
            }

            e++;
        }

        if (!found)
        {
            D(kprintf("opcode not found!\n"));
            break;
        }
    }

    return mask | needed;
}
