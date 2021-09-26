/****************************************************/
/* FPU Instruction opcodes and condition fields.	*/
/*													*/
/* This part holds all tables,						*/
/* Header according to the following format.		*/
/* Table Name.										*/
/* Assumed depth.									*/
/* The bits used [x:y]								*/
/*  												*/
/****************************************************/

#include "M68k.h"

/* 
Index
	EA decoding & format,
		D0
		D1
		D2
		D3
		D4
		D5
		D6
		D7
		(A0)
		(A1)
		(A2)
		(A3)
		(A4)
		(A5)
		(A6)
		(A7)
		(A0)+
		(A1)+
		(A2)+
		(A3)+
		(A4)+
		(A5)+
		(A6)+
		(A7)+
		-(A0)
		-(A1)
		-(A2)
		-(A3)
		-(A4)
		-(A5)
		-(A6)
		-(A7)
		(d16,A0)
		(d16,A1)
		(d16,A2)
		(d16,A3)
		(d16,A4)
		(d16,A5)
		(d16,A6)
		(d16,A7)
		(A0,Xn)
		(A1,Xn)
		(A2,Xn)
		(A3,Xn)
		(A4,Xn)
		(A5,Xn)
		(A6,Xn)
		(A7,Xn)
		(xxx).W
		(xxx).L
		(d16,PC)
		(PC,Xn)
		#<data>
	
	FPU Instructions,
	CC Instructions,
	Branch,
		Misc tests
		IEEE Aware tests
		IEEE Nonaware tests
*/		
/*EA decoding & format, Table 1; bits[22:10]*/
static EMIT_Funtion JumpTableCase0[4096] = {
[00000 ... 00007] = EMIT_FORMAT_X,	//FPm,FPn
//D0
[00020] = EMIT_FORMAT_L,	//Dn,FPn
[00021] = EMIT_FORMAT_S,	//Dn,FPn
[00024] = EMIT_FORMAT_W,	//Dn,FPn
[00026] = EMIT_FORMAT_B,	//Dn,FPn
[00027] = EMIT_FMOVECR,
[00030] = EMIT_FMOVE_L,	//FPn,Dn
[00031] = EMIT_FMOVE_S,	//FPn,Dn
[00034] = EMIT_FMOVE_W,	//FPn,Dn
[00036] = EMIT_FMOVE_B,	//FPn,Dn
[00041] = EMIT_FMOVE_L,	//Dn,FPIAR
[00042] = EMIT_FMOVE_L,	//Dn,FPSR
[00044] = EMIT_FMOVE_L,	//Dn,FPCR
[00051] = EMIT_FMOVE_L,	//FPIAR,Dn
[00052] = EMIT_FMOVE_L,	//FPSR,Dn
[00054] = EMIT_FMOVE_L,	//FPCR,Dn
//D1
[00120] = EMIT_FORMAT_L,	//Dn,FPn
[00121] = EMIT_FORMAT_S,	//Dn,FPn
[00124] = EMIT_FORMAT_W,	//Dn,FPn
[00126] = EMIT_FORMAT_B,	//Dn,FPn
[00130] = EMIT_FMOVE_L,	//FPn,Dn
[00131] = EMIT_FMOVE_S,	//FPn,Dn
[00134] = EMIT_FMOVE_W,	//FPn,Dn
[00136] = EMIT_FMOVE_B,	//FPn,Dn
[00141] = EMIT_FMOVE_L,	//Dn,FPIAR
[00142] = EMIT_FMOVE_L,	//Dn,FPSR
[00144] = EMIT_FMOVE_L,	//Dn,FPCR
[00151] = EMIT_FMOVE_L,	//FPIAR,Dn
[00152] = EMIT_FMOVE_L,	//FPSR,Dn
[00154] = EMIT_FMOVE_L,	//FPCR,Dn
//D2
[00220] = EMIT_FORMAT_L,	//Dn,FPn
[00221] = EMIT_FORMAT_S,	//Dn,FPn
[00224] = EMIT_FORMAT_W,	//Dn,FPn
[00226] = EMIT_FORMAT_B,	//Dn,FPn
[00230] = EMIT_FMOVE_L,	//FPn,Dn
[00231] = EMIT_FMOVE_S,	//FPn,Dn
[00234] = EMIT_FMOVE_W,	//FPn,Dn
[00236] = EMIT_FMOVE_B,	//FPn,Dn
[00241] = EMIT_FMOVE_L,	//Dn,FPIAR
[00242] = EMIT_FMOVE_L,	//Dn,FPSR
[00244] = EMIT_FMOVE_L,	//Dn,FPCR
[00251] = EMIT_FMOVE_L,	//FPIAR,Dn
[00252] = EMIT_FMOVE_L,	//FPSR,Dn
[00254] = EMIT_FMOVE_L,	//FPCR,Dn
//D3
[00320] = EMIT_FORMAT_L,	//Dn,FPn
[00321] = EMIT_FORMAT_S,	//Dn,FPn
[00324] = EMIT_FORMAT_W,	//Dn,FPn
[00326] = EMIT_FORMAT_B,	//Dn,FPn
[00330] = EMIT_FMOVE_L,	//FPn,Dn
[00331] = EMIT_FMOVE_S,	//FPn,Dn
[00334] = EMIT_FMOVE_W,	//FPn,Dn
[00336] = EMIT_FMOVE_B,	//FPn,Dn
[00341] = EMIT_FMOVE_L,	//Dn,FPIAR
[00342] = EMIT_FMOVE_L,	//Dn,FPSR
[00344] = EMIT_FMOVE_L,	//Dn,FPCR
[00351] = EMIT_FMOVE_L,	//FPIAR,Dn
[00352] = EMIT_FMOVE_L,	//FPSR,Dn
[00354] = EMIT_FMOVE_L,	//FPCR,Dn
//D4
[00420] = EMIT_FORMAT_L,	//Dn,FPn
[00421] = EMIT_FORMAT_S,	//Dn,FPn
[00424] = EMIT_FORMAT_W,	//Dn,FPn
[00426] = EMIT_FORMAT_B,	//Dn,FPn
[00430] = EMIT_FMOVE_L,	//FPn,Dn
[00431] = EMIT_FMOVE_S,	//FPn,Dn
[00434] = EMIT_FMOVE_W,	//FPn,Dn
[00436] = EMIT_FMOVE_B,	//FPn,Dn
[00441] = EMIT_FMOVE_L,	//Dn,FPIAR
[00442] = EMIT_FMOVE_L,	//Dn,FPSR
[00444] = EMIT_FMOVE_L,	//Dn,FPCR
[00451] = EMIT_FMOVE_L,	//FPIAR,Dn
[00452] = EMIT_FMOVE_L,	//FPSR,Dn
[00454] = EMIT_FMOVE_L,	//FPCR,Dn
//D5
[00520] = EMIT_FORMAT_L,	//Dn,FPn
[00521] = EMIT_FORMAT_S,	//Dn,FPn
[00524] = EMIT_FORMAT_W,	//Dn,FPn
[00526] = EMIT_FORMAT_B,	//Dn,FPn
[00530] = EMIT_FMOVE_L,	//FPn,Dn
[00531] = EMIT_FMOVE_S,	//FPn,Dn
[00534] = EMIT_FMOVE_W,	//FPn,Dn
[00536] = EMIT_FMOVE_B,	//FPn,Dn
[00541] = EMIT_FMOVE_L,	//Dn,FPIAR
[00542] = EMIT_FMOVE_L,	//Dn,FPSR
[00544] = EMIT_FMOVE_L,	//Dn,FPCR
[00551] = EMIT_FMOVE_L,	//FPIAR,Dn
[00552] = EMIT_FMOVE_L,	//FPSR,Dn
[00554] = EMIT_FMOVE_L,	//FPCR,Dn
//D6
[00620] = EMIT_FORMAT_L,	//Dn,FPn
[00621] = EMIT_FORMAT_S,	//Dn,FPn
[00624] = EMIT_FORMAT_W,	//Dn,FPn
[00626] = EMIT_FORMAT_B,	//Dn,FPn
[00630] = EMIT_FMOVE_L,	//FPn,Dn
[00631] = EMIT_FMOVE_S,	//FPn,Dn
[00634] = EMIT_FMOVE_W,	//FPn,Dn
[00636] = EMIT_FMOVE_B,	//FPn,Dn
[00641] = EMIT_FMOVE_L,	//Dn,FPIAR
[00642] = EMIT_FMOVE_L,	//Dn,FPSR
[00644] = EMIT_FMOVE_L,	//Dn,FPCR
[00651] = EMIT_FMOVE_L,	//FPIAR,Dn
[00652] = EMIT_FMOVE_L,	//FPSR,Dn
[00654] = EMIT_FMOVE_L,	//FPCR,Dn
//D7
[00720] = EMIT_FORMAT_L,	//Dn,FPn
[00721] = EMIT_FORMAT_S,	//Dn,FPn
[00724] = EMIT_FORMAT_W,	//Dn,FPn
[00726] = EMIT_FORMAT_B,	//Dn,FPn
[00730] = EMIT_FMOVE_L,	//FPn,Dn
[00731] = EMIT_FMOVE_S,	//FPn,Dn
[00734] = EMIT_FMOVE_W,	//FPn,Dn
[00736] = EMIT_FMOVE_B,	//FPn,Dn
[00741] = EMIT_FMOVE_L,	//Dn,FPIAR
[00742] = EMIT_FMOVE_L,	//Dn,FPSR
[00744] = EMIT_FMOVE_L,	//Dn,FPCR
[00751] = EMIT_FMOVE_L,	//FPIAR,Dn
[00752] = EMIT_FMOVE_L,	//FPSR,Dn
[00754] = EMIT_FMOVE_L,	//FPCR,Dn
//FMOVE.L An,FPIAR and FMOVE.L FPIAR,An 
[01041] = EMIT_FMOVE_L,	//FMOVE.L A0,FPIAR 
[01051] = EMIT_FMOVE_L,	//FMOVE>L FPIAR,A0
[01141] = EMIT_FMOVE_L,	//FMOVE.L A1,FPIAR
[01151] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A1
[01241] = EMIT_FMOVE_L,	//FMOVE.L A2,FPIAR
[01251] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A2
[01341] = EMIT_FMOVE_L,	//FMOVE.L A3,FPIAR
[01351] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A3
[01441] = EMIT_FMOVE_L,	//FMOVE.L A4,FPIAR
[01451] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A4
[01541] = EMIT_FMOVE_L,	//FMOVE.L A5,FPIAR
[01551] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A5
[01641] = EMIT_FMOVE_L,	//FMOVE.L A6,FPIAR
[01651] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A6
[01741] = EMIT_FMOVE_L,	//FMOVE.L A7,FPIAR
[01751] = EMIT_FMOVE_L,	//FMOVE.L FPIAR,A7
//(A0)
[02020] = EMIT_FORMAT_L_mem,
[02021] = EMIT_FORMAT_S_mem,
[02022] = EMIT_FORMAT_X_mem,
[02023] = EMIT_FORMAT_P_mem,
[02024] = EMIT_FORMAT_W_mem,
[02025] = EMIT_FORMAT_D_mem,
[02026] = EMIT_FORMAT_B_mem,
[02030] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02031] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02022] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02023] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02034] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02025] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02036] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02041] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02042] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02043] = EMIT_FMOVEM_L_mem,
[02044] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02045 ... 02047] = EMIT_FMOVEM_L_mem,
[02051] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02052] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02053] = EMIT_FMOVEM_L_mem,
[02054] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02055 ... 02057] = EMIT_FMOVEM_L_mem,
[02060] = EMIT_FMOVEM,	//static, predecrement
[02062] = EMIT_FMOVEM,	//dynamic, predecrement
[02064] = EMIT_FMOVEM,	//static, postincrement
[02066] = EMIT_FMOVEM,	//dynamic. postincrement
[02070] = EMIT_FMOVEM,	//static, predecrement
[02072] = EMIT_FMOVEM,	//dynamic, predecrement
[02074] = EMIT_FMOVEM,	//static, postincrement
[02076] = EMIT_FMOVEM,	//dynamic. postincrement
//(A1)
[02120] = EMIT_FORMAT_L_mem,
[02121] = EMIT_FORMAT_S_mem,
[02122] = EMIT_FORMAT_X_mem,
[02123] = EMIT_FORMAT_P_mem,
[02124] = EMIT_FORMAT_W_mem,
[02125] = EMIT_FORMAT_D_mem,
[02126] = EMIT_FORMAT_B_mem,
[02130] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02131] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02122] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02123] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02134] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02125] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02136] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02141] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02142] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02143] = EMIT_FMOVEM_L_mem,
[02144] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02145 ... 02147] = EMIT_FMOVEM_L_mem,
[02151] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02152] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02153] = EMIT_FMOVEM_L_mem,
[02154] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02155 ... 02157] = EMIT_FMOVEM_L_mem,
[02160] = EMIT_FMOVEM,	//static, predecrement
[02162] = EMIT_FMOVEM,	//dynamic, predecrement
[02164] = EMIT_FMOVEM,	//static, postincrement
[02166] = EMIT_FMOVEM,	//dynamic. postincrement
[02170] = EMIT_FMOVEM,	//static, predecrement
[02172] = EMIT_FMOVEM,	//dynamic, predecrement
[02174] = EMIT_FMOVEM,	//static, postincrement
[02176] = EMIT_FMOVEM,	//dynamic. postincrement
//(A2)
[02220] = EMIT_FORMAT_L_mem,
[02221] = EMIT_FORMAT_S_mem,
[02222] = EMIT_FORMAT_X_mem,
[02223] = EMIT_FORMAT_P_mem,
[02224] = EMIT_FORMAT_W_mem,
[02225] = EMIT_FORMAT_D_mem,
[02226] = EMIT_FORMAT_B_mem,
[02230] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02231] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02222] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02223] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02234] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02225] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02236] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02241] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02242] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02243] = EMIT_FMOVEM_L_mem,
[02244] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02245 ... 02247] = EMIT_FMOVEM_L_mem,
[02251] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02252] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02253] = EMIT_FMOVEM_L_mem,
[02254] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02255 ... 02257] = EMIT_FMOVEM_L_mem,
[02260] = EMIT_FMOVEM,	//static, predecrement
[02262] = EMIT_FMOVEM,	//dynamic, predecrement
[02264] = EMIT_FMOVEM,	//static, postincrement
[02266] = EMIT_FMOVEM,	//dynamic. postincrement
[02270] = EMIT_FMOVEM,	//static, predecrement
[02272] = EMIT_FMOVEM,	//dynamic, predecrement
[02274] = EMIT_FMOVEM,	//static, postincrement
[02276] = EMIT_FMOVEM,	//dynamic. postincrement
//(A3)
[02320] = EMIT_FORMAT_L_mem,
[02321] = EMIT_FORMAT_S_mem,
[02322] = EMIT_FORMAT_X_mem,
[02323] = EMIT_FORMAT_P_mem,
[02324] = EMIT_FORMAT_W_mem,
[02325] = EMIT_FORMAT_D_mem,
[02326] = EMIT_FORMAT_B_mem,
[02330] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02331] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02322] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02323] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02334] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02325] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02336] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02341] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02342] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02343] = EMIT_FMOVEM_L_mem,
[02344] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02345 ... 02347] = EMIT_FMOVEM_L_mem,
[02351] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02352] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02353] = EMIT_FMOVEM_L_mem,
[02354] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02355 ... 02357] = EMIT_FMOVEM_L_mem,
[02360] = EMIT_FMOVEM,	//static, predecrement
[02362] = EMIT_FMOVEM,	//dynamic, predecrement
[02364] = EMIT_FMOVEM,	//static, postincrement
[02366] = EMIT_FMOVEM,	//dynamic. postincrement
[02370] = EMIT_FMOVEM,	//static, predecrement
[02372] = EMIT_FMOVEM,	//dynamic, predecrement
[02374] = EMIT_FMOVEM,	//static, postincrement
[02376] = EMIT_FMOVEM,	//dynamic. postincrement
//(A4)
[02420] = EMIT_FORMAT_L_mem,
[02421] = EMIT_FORMAT_S_mem,
[02422] = EMIT_FORMAT_X_mem,
[02423] = EMIT_FORMAT_P_mem,
[02424] = EMIT_FORMAT_W_mem,
[02425] = EMIT_FORMAT_D_mem,
[02426] = EMIT_FORMAT_B_mem,
[02430] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02431] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02422] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02423] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02434] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02425] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02436] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02441] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02442] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02443] = EMIT_FMOVEM_L_mem,
[02444] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02445 ... 02447] = EMIT_FMOVEM_L_mem,
[02451] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02452] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02453] = EMIT_FMOVEM_L_mem,
[02454] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02455 ... 02457] = EMIT_FMOVEM_L_mem,
[02460] = EMIT_FMOVEM,	//static, predecrement
[02462] = EMIT_FMOVEM,	//dynamic, predecrement
[02464] = EMIT_FMOVEM,	//static, postincrement
[02466] = EMIT_FMOVEM,	//dynamic. postincrement
[02470] = EMIT_FMOVEM,	//static, predecrement
[02472] = EMIT_FMOVEM,	//dynamic, predecrement
[02474] = EMIT_FMOVEM,	//static, postincrement
[02476] = EMIT_FMOVEM,	//dynamic. postincrement
//(A5)
[02520] = EMIT_FORMAT_L_mem,
[02521] = EMIT_FORMAT_S_mem,
[02522] = EMIT_FORMAT_X_mem,
[02523] = EMIT_FORMAT_P_mem,
[02524] = EMIT_FORMAT_W_mem,
[02525] = EMIT_FORMAT_D_mem,
[02526] = EMIT_FORMAT_B_mem,
[02530] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02531] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02522] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02523] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02534] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02525] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02536] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02541] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02542] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02543] = EMIT_FMOVEM_L_mem,
[02544] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02545 ... 02547] = EMIT_FMOVEM_L_mem,
[02551] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02552] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02553] = EMIT_FMOVEM_L_mem,
[02554] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02555 ... 02557] = EMIT_FMOVEM_L_mem,
[02560] = EMIT_FMOVEM,	//static, predecrement
[02562] = EMIT_FMOVEM,	//dynamic, predecrement
[02564] = EMIT_FMOVEM,	//static, postincrement
[02566] = EMIT_FMOVEM,	//dynamic. postincrement
[02570] = EMIT_FMOVEM,	//static, predecrement
[02572] = EMIT_FMOVEM,	//dynamic, predecrement
[02574] = EMIT_FMOVEM,	//static, postincrement
[02576] = EMIT_FMOVEM,	//dynamic. postincrement
//(A6)
[02620] = EMIT_FORMAT_L_mem,
[02621] = EMIT_FORMAT_S_mem,
[02622] = EMIT_FORMAT_X_mem,
[02623] = EMIT_FORMAT_P_mem,
[02624] = EMIT_FORMAT_W_mem,
[02625] = EMIT_FORMAT_D_mem,
[02626] = EMIT_FORMAT_B_mem,
[02630] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02631] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02622] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02623] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02634] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02625] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02636] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02641] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02642] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02643] = EMIT_FMOVEM_L_mem,
[02644] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02645 ... 02647] = EMIT_FMOVEM_L_mem,
[02651] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02652] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02653] = EMIT_FMOVEM_L_mem,
[02654] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02655 ... 02657] = EMIT_FMOVEM_L_mem,
[02660] = EMIT_FMOVEM,	//static, predecrement
[02662] = EMIT_FMOVEM,	//dynamic, predecrement
[02664] = EMIT_FMOVEM,	//static, postincrement
[02666] = EMIT_FMOVEM,	//dynamic. postincrement
[02670] = EMIT_FMOVEM,	//static, predecrement
[02672] = EMIT_FMOVEM,	//dynamic, predecrement
[02674] = EMIT_FMOVEM,	//static, postincrement
[02676] = EMIT_FMOVEM,	//dynamic. postincrement
//(A7)
[02720] = EMIT_FORMAT_L_mem,
[02721] = EMIT_FORMAT_S_mem,
[02722] = EMIT_FORMAT_X_mem,
[02723] = EMIT_FORMAT_P_mem,
[02724] = EMIT_FORMAT_W_mem,
[02725] = EMIT_FORMAT_D_mem,
[02726] = EMIT_FORMAT_B_mem,
[02730] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[02731] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[02722] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[02723] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[02734] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[02725] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[02736] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[02741] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[02742] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[02743] = EMIT_FMOVEM_L_mem,
[02744] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[02745 ... 02747] = EMIT_FMOVEM_L_mem,
[02751] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[02752] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[02753] = EMIT_FMOVEM_L_mem,
[02754] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[02755 ... 02757] = EMIT_FMOVEM_L_mem,
[02760] = EMIT_FMOVEM,	//static, predecrement
[02762] = EMIT_FMOVEM,	//dynamic, predecrement
[02764] = EMIT_FMOVEM,	//static, postincrement
[02766] = EMIT_FMOVEM,	//dynamic. postincrement
[02770] = EMIT_FMOVEM,	//static, predecrement
[02772] = EMIT_FMOVEM,	//dynamic, predecrement
[02774] = EMIT_FMOVEM,	//static, postincrement
[02776] = EMIT_FMOVEM,	//dynamic. postincrement
//(A0)+
[03020] = EMIT_FORMAT_L_mem,
[03021] = EMIT_FORMAT_S_mem,
[03022] = EMIT_FORMAT_X_mem,
[03023] = EMIT_FORMAT_P_mem,
[03024] = EMIT_FORMAT_W_mem,
[03025] = EMIT_FORMAT_D_mem,
[03026] = EMIT_FORMAT_B_mem,
[03030] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03031] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03022] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03023] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03034] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03025] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03036] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03041] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03042] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03043] = EMIT_FMOVEM_L_mem,
[03044] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03045 ... 03047] = EMIT_FMOVEM_L_mem,
[03060] = EMIT_FMOVEM,	//static, predecrement
[03062] = EMIT_FMOVEM,	//dynamic, predecrement
[03064] = EMIT_FMOVEM,	//static, postincrement
[03066] = EMIT_FMOVEM,	//dynamic. postincrement
//(A1)+
[03120] = EMIT_FORMAT_L_mem,
[03121] = EMIT_FORMAT_S_mem,
[03122] = EMIT_FORMAT_X_mem,
[03123] = EMIT_FORMAT_P_mem,
[03124] = EMIT_FORMAT_W_mem,
[03125] = EMIT_FORMAT_D_mem,
[03126] = EMIT_FORMAT_B_mem,
[03130] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03131] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03122] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03123] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03134] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03125] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03136] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03141] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03142] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03143] = EMIT_FMOVEM_L_mem,
[03144] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03145 ... 03147] = EMIT_FMOVEM_L_mem,
[03160] = EMIT_FMOVEM,	//static, predecrement
[03162] = EMIT_FMOVEM,	//dynamic, predecrement
[03164] = EMIT_FMOVEM,	//static, postincrement
[03166] = EMIT_FMOVEM,	//dynamic. postincrement
//(A2)+
[03220] = EMIT_FORMAT_L_mem,
[03221] = EMIT_FORMAT_S_mem,
[03222] = EMIT_FORMAT_X_mem,
[03223] = EMIT_FORMAT_P_mem,
[03224] = EMIT_FORMAT_W_mem,
[03225] = EMIT_FORMAT_D_mem,
[03226] = EMIT_FORMAT_B_mem,
[03230] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03231] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03222] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03223] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03234] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03225] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03236] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03241] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03242] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03243] = EMIT_FMOVEM_L_mem,
[03244] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03245 ... 03247] = EMIT_FMOVEM_L_mem,
[03260] = EMIT_FMOVEM,	//static, predecrement
[03262] = EMIT_FMOVEM,	//dynamic, predecrement
[03264] = EMIT_FMOVEM,	//static, postincrement
[03266] = EMIT_FMOVEM,	//dynamic. postincrement
//(A3)+
[03320] = EMIT_FORMAT_L_mem,
[03321] = EMIT_FORMAT_S_mem,
[03322] = EMIT_FORMAT_X_mem,
[03323] = EMIT_FORMAT_P_mem,
[03324] = EMIT_FORMAT_W_mem,
[03325] = EMIT_FORMAT_D_mem,
[03326] = EMIT_FORMAT_B_mem,
[03330] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03331] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03322] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03323] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03334] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03325] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03336] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03341] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03342] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03343] = EMIT_FMOVEM_L_mem,
[03344] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03345 ... 03347] = EMIT_FMOVEM_L_mem,
[03360] = EMIT_FMOVEM,	//static, predecrement
[03362] = EMIT_FMOVEM,	//dynamic, predecrement
[03364] = EMIT_FMOVEM,	//static, postincrement
[03366] = EMIT_FMOVEM,	//dynamic. postincrement
//(A4)+
[03420] = EMIT_FORMAT_L_mem,
[03421] = EMIT_FORMAT_S_mem,
[03422] = EMIT_FORMAT_X_mem,
[03423] = EMIT_FORMAT_P_mem,
[03424] = EMIT_FORMAT_W_mem,
[03425] = EMIT_FORMAT_D_mem,
[03426] = EMIT_FORMAT_B_mem,
[03430] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03431] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03422] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03423] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03434] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03425] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03436] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03441] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03442] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03443] = EMIT_FMOVEM_L_mem,
[03444] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03445 ... 03447] = EMIT_FMOVEM_L_mem,
[03460] = EMIT_FMOVEM,	//static, predecrement
[03462] = EMIT_FMOVEM,	//dynamic, predecrement
[03464] = EMIT_FMOVEM,	//static, postincrement
[03466] = EMIT_FMOVEM,	//dynamic. postincrement
//(A5)+
[03520] = EMIT_FORMAT_L_mem,
[03521] = EMIT_FORMAT_S_mem,
[03522] = EMIT_FORMAT_X_mem,
[03523] = EMIT_FORMAT_P_mem,
[03524] = EMIT_FORMAT_W_mem,
[03525] = EMIT_FORMAT_D_mem,
[03526] = EMIT_FORMAT_B_mem,
[03530] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03531] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03522] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03523] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03534] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03525] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03536] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03541] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03542] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03543] = EMIT_FMOVEM_L_mem,
[03544] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03545 ... 03547] = EMIT_FMOVEM_L_mem,
[03560] = EMIT_FMOVEM,	//static, predecrement
[03562] = EMIT_FMOVEM,	//dynamic, predecrement
[03564] = EMIT_FMOVEM,	//static, postincrement
[03566] = EMIT_FMOVEM,	//dynamic. postincrement
//(A6)+
[03620] = EMIT_FORMAT_L_mem,
[03621] = EMIT_FORMAT_S_mem,
[03622] = EMIT_FORMAT_X_mem,
[03623] = EMIT_FORMAT_P_mem,
[03624] = EMIT_FORMAT_W_mem,
[03625] = EMIT_FORMAT_D_mem,
[03626] = EMIT_FORMAT_B_mem,
[03630] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03631] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03622] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03623] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03634] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03625] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03636] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03641] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03642] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03643] = EMIT_FMOVEM_L_mem,
[03644] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03645 ... 03647] = EMIT_FMOVEM_L_mem,
[03660] = EMIT_FMOVEM,	//static, predecrement
[03662] = EMIT_FMOVEM,	//dynamic, predecrement
[03664] = EMIT_FMOVEM,	//static, postincrement
[03666] = EMIT_FMOVEM,	//dynamic. postincrement
//(A7)+
[03720] = EMIT_FORMAT_L_mem,
[03721] = EMIT_FORMAT_S_mem,
[03722] = EMIT_FORMAT_X_mem,
[03723] = EMIT_FORMAT_P_mem,
[03724] = EMIT_FORMAT_W_mem,
[03725] = EMIT_FORMAT_D_mem,
[03726] = EMIT_FORMAT_B_mem,
[03730] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[03731] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[03722] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[03723] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[03734] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[03725] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[03736] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[03741] = EMIT_FMOVE_L_mem,	//<ea>,FPIAR
[03742] = EMIT_FMOVE_L_mem,	//<ea>,FPSR
[03743] = EMIT_FMOVEM_L_mem,
[03744] = EMIT_FMOVE_L_mem,	//<ea>,FPCR
[03745 ... 03747] = EMIT_FMOVEM_L_mem,
[03760] = EMIT_FMOVEM,	//static, predecrement
[03762] = EMIT_FMOVEM,	//dynamic, predecrement
[03764] = EMIT_FMOVEM,	//static, postincrement
[03766] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A0)
[04020] = EMIT_FORMAT_L_mem,
[04021] = EMIT_FORMAT_S_mem,
[04022] = EMIT_FORMAT_X_mem,
[04023] = EMIT_FORMAT_P_mem,
[04024] = EMIT_FORMAT_W_mem,
[04025] = EMIT_FORMAT_D_mem,
[04026] = EMIT_FORMAT_B_mem,
[04030] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04031] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04022] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04023] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04034] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04025] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04036] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04051] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04052] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04053] = EMIT_FMOVEM_L_mem,
[04054] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04055 ... 04057] = EMIT_FMOVEM_L_mem,
[04070] = EMIT_FMOVEM,	//static, predecrement
[04072] = EMIT_FMOVEM,	//dynamic, predecrement
[04074] = EMIT_FMOVEM,	//static, postincrement
[04076] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A1)
[04120] = EMIT_FORMAT_L_mem,
[04121] = EMIT_FORMAT_S_mem,
[04122] = EMIT_FORMAT_X_mem,
[04123] = EMIT_FORMAT_P_mem,
[04124] = EMIT_FORMAT_W_mem,
[04125] = EMIT_FORMAT_D_mem,
[04126] = EMIT_FORMAT_B_mem,
[04130] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04131] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04122] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04123] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04134] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04125] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04136] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04151] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04152] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04153] = EMIT_FMOVEM_L_mem,
[04154] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04155 ... 04157] = EMIT_FMOVEM_L_mem,
[04170] = EMIT_FMOVEM,	//static, predecrement
[04172] = EMIT_FMOVEM,	//dynamic, predecrement
[04174] = EMIT_FMOVEM,	//static, postincrement
[04176] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A2)
[04220] = EMIT_FORMAT_L_mem,
[04221] = EMIT_FORMAT_S_mem,
[04222] = EMIT_FORMAT_X_mem,
[04223] = EMIT_FORMAT_P_mem,
[04224] = EMIT_FORMAT_W_mem,
[04225] = EMIT_FORMAT_D_mem,
[04226] = EMIT_FORMAT_B_mem,
[04230] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04231] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04222] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04223] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04234] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04225] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04236] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04251] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04252] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04253] = EMIT_FMOVEM_L_mem,
[04254] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04255 ... 04257] = EMIT_FMOVEM_L_mem,
[04270] = EMIT_FMOVEM,	//static, predecrement
[04272] = EMIT_FMOVEM,	//dynamic, predecrement
[04274] = EMIT_FMOVEM,	//static, postincrement
[04276] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A3)
[04320] = EMIT_FORMAT_L_mem,
[04321] = EMIT_FORMAT_S_mem,
[04322] = EMIT_FORMAT_X_mem,
[04323] = EMIT_FORMAT_P_mem,
[04324] = EMIT_FORMAT_W_mem,
[04325] = EMIT_FORMAT_D_mem,
[04326] = EMIT_FORMAT_B_mem,
[04330] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04331] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04322] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04323] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04334] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04325] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04336] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04351] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04352] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04353] = EMIT_FMOVEM_L_mem,
[04354] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04355 ... 04357] = EMIT_FMOVEM_L_mem,
[04370] = EMIT_FMOVEM,	//static, predecrement
[04372] = EMIT_FMOVEM,	//dynamic, predecrement
[04374] = EMIT_FMOVEM,	//static, postincrement
[04376] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A4)
[04420] = EMIT_FORMAT_L_mem,
[04421] = EMIT_FORMAT_S_mem,
[04422] = EMIT_FORMAT_X_mem,
[04423] = EMIT_FORMAT_P_mem,
[04424] = EMIT_FORMAT_W_mem,
[04425] = EMIT_FORMAT_D_mem,
[04426] = EMIT_FORMAT_B_mem,
[04430] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04431] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04422] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04423] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04434] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04425] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04436] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04451] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04452] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04453] = EMIT_FMOVEM_L_mem,
[04454] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04455 ... 04457] = EMIT_FMOVEM_L_mem,
[04470] = EMIT_FMOVEM,	//static, predecrement
[04472] = EMIT_FMOVEM,	//dynamic, predecrement
[04474] = EMIT_FMOVEM,	//static, postincrement
[04476] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A5)
[04520] = EMIT_FORMAT_L_mem,
[04521] = EMIT_FORMAT_S_mem,
[04522] = EMIT_FORMAT_X_mem,
[04523] = EMIT_FORMAT_P_mem,
[04524] = EMIT_FORMAT_W_mem,
[04525] = EMIT_FORMAT_D_mem,
[04526] = EMIT_FORMAT_B_mem,
[04530] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04531] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04522] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04523] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04534] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04525] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04536] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04551] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04552] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04553] = EMIT_FMOVEM_L_mem,
[04554] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04555 ... 04557] = EMIT_FMOVEM_L_mem,
[04570] = EMIT_FMOVEM,	//static, predecrement
[04572] = EMIT_FMOVEM,	//dynamic, predecrement
[04574] = EMIT_FMOVEM,	//static, postincrement
[04576] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A6)
[04620] = EMIT_FORMAT_L_mem,
[04621] = EMIT_FORMAT_S_mem,
[04622] = EMIT_FORMAT_X_mem,
[04623] = EMIT_FORMAT_P_mem,
[04624] = EMIT_FORMAT_W_mem,
[04625] = EMIT_FORMAT_D_mem,
[04626] = EMIT_FORMAT_B_mem,
[04630] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04631] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04622] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04623] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04634] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04625] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04636] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04651] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04652] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04653] = EMIT_FMOVEM_L_mem,
[04654] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04655 ... 04657] = EMIT_FMOVEM_L_mem,
[04670] = EMIT_FMOVEM,	//static, predecrement
[04672] = EMIT_FMOVEM,	//dynamic, predecrement
[04674] = EMIT_FMOVEM,	//static, postincrement
[04676] = EMIT_FMOVEM,	//dynamic. postincrement
//-(A7)
[04720] = EMIT_FORMAT_L_mem,
[04721] = EMIT_FORMAT_S_mem,
[04722] = EMIT_FORMAT_X_mem,
[04723] = EMIT_FORMAT_P_mem,
[04724] = EMIT_FORMAT_W_mem,
[04725] = EMIT_FORMAT_D_mem,
[04726] = EMIT_FORMAT_B_mem,
[04730] = EMIT_FMOVE_L_mem,	//FPn,<ea>
[04731] = EMIT_FMOVE_S_mem,	//FPn,<ea>
[04722] = EMIT_FMOVE_X_mem,	//FPn,<ea>
[04723] = EMIT_FMOVE_P_mem,	//FPn,<ea>
[04734] = EMIT_FMOVE_W_mem,	//FPn,<ea>
[04725] = EMIT_FMOVE_D_mem,	//FPn,<ea>
[04736] = EMIT_FMOVE_B_mem,	//FPn,<ea>
[04751] = EMIT_FMOVE_L_mem,	//FPIAR,<ea>
[04752] = EMIT_FMOVE_L_mem,	//FPSR,<ea>
[04753] = EMIT_FMOVEM_L_mem,
[04754] = EMIT_FMOVE_L_mem,	//FPCR,<ea>
[04755 ... 04757] = EMIT_FMOVEM_L_mem,
[04770] = EMIT_FMOVEM,	//static, predecrement
[04772] = EMIT_FMOVEM,	//dynamic, predecrement
[04774] = EMIT_FMOVEM,	//static, postincrement
[04776] = EMIT_FMOVEM,	//dynamic. postincrement
//(d16,A0)
[05020] = EMIT_FORMAT_L_ext,
[05021] = EMIT_FORMAT_S_ext,
[05022] = EMIT_FORMAT_X_ext,
[05023] = EMIT_FORMAT_P_ext,
[05024] = EMIT_FORMAT_W_ext,
[05025] = EMIT_FORMAT_D_ext,
[05026] = EMIT_FORMAT_B_ext,
[05030] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05031] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05022] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05023] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05034] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05025] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05036] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05041] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05042] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05043] = EMIT_FMOVEM_L_ext,
[05044] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05045 ... 05047] = EMIT_FMOVEM_L_ext,
[05051] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05052] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05053] = EMIT_FMOVEM_L_ext,
[05054] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05055 ... 05057] = EMIT_FMOVEM_L_ext,
[05060] = EMIT_FMOVEM_ext,	//static, predecrement
[05062] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05064] = EMIT_FMOVEM_ext,	//static, postincrement
[05066] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05070] = EMIT_FMOVEM_ext,	//static, predecrement
[05072] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05074] = EMIT_FMOVEM_ext,	//static, postincrement
[05076] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A1)
[05120] = EMIT_FORMAT_L_ext,
[05121] = EMIT_FORMAT_S_ext,
[05122] = EMIT_FORMAT_X_ext,
[05123] = EMIT_FORMAT_P_ext,
[05124] = EMIT_FORMAT_W_ext,
[05125] = EMIT_FORMAT_D_ext,
[05126] = EMIT_FORMAT_B_ext,
[05130] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05131] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05122] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05123] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05134] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05125] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05136] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05141] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05142] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05143] = EMIT_FMOVEM_L_ext,
[05144] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05145 ... 05147] = EMIT_FMOVEM_L_ext,
[05151] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05152] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05153] = EMIT_FMOVEM_L_ext,
[05154] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05155 ... 05157] = EMIT_FMOVEM_L_ext,
[05160] = EMIT_FMOVEM_ext,	//static, predecrement
[05162] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05164] = EMIT_FMOVEM_ext,	//static, postincrement
[05166] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05170] = EMIT_FMOVEM_ext,	//static, predecrement
[05172] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05174] = EMIT_FMOVEM_ext,	//static, postincrement
[05176] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A2)
[05220] = EMIT_FORMAT_L_ext,
[05221] = EMIT_FORMAT_S_ext,
[05222] = EMIT_FORMAT_X_ext,
[05223] = EMIT_FORMAT_P_ext,
[05224] = EMIT_FORMAT_W_ext,
[05225] = EMIT_FORMAT_D_ext,
[05226] = EMIT_FORMAT_B_ext,
[05230] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05231] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05222] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05223] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05234] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05225] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05236] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05241] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05242] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05243] = EMIT_FMOVEM_L_ext,
[05244] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05245 ... 05247] = EMIT_FMOVEM_L_ext,
[05251] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05252] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05253] = EMIT_FMOVEM_L_ext,
[05254] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05255 ... 05257] = EMIT_FMOVEM_L_ext,
[05260] = EMIT_FMOVEM_ext,	//static, predecrement
[05262] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05264] = EMIT_FMOVEM_ext,	//static, postincrement
[05266] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05270] = EMIT_FMOVEM_ext,	//static, predecrement
[05272] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05274] = EMIT_FMOVEM_ext,	//static, postincrement
[05276] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A3)
[05320] = EMIT_FORMAT_L_ext,
[05321] = EMIT_FORMAT_S_ext,
[05322] = EMIT_FORMAT_X_ext,
[05323] = EMIT_FORMAT_P_ext,
[05324] = EMIT_FORMAT_W_ext,
[05325] = EMIT_FORMAT_D_ext,
[05326] = EMIT_FORMAT_B_ext,
[05330] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05331] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05322] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05323] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05334] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05325] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05336] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05341] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05342] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05343] = EMIT_FMOVEM_L_ext,
[05344] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05345 ... 05347] = EMIT_FMOVEM_L_ext,
[05351] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05352] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05353] = EMIT_FMOVEM_L_ext,
[05354] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05355 ... 05357] = EMIT_FMOVEM_L_ext,
[05360] = EMIT_FMOVEM_ext,	//static, predecrement
[05362] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05364] = EMIT_FMOVEM_ext,	//static, postincrement
[05366] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05370] = EMIT_FMOVEM_ext,	//static, predecrement
[05372] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05374] = EMIT_FMOVEM_ext,	//static, postincrement
[05376] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A4)
[05420] = EMIT_FORMAT_L_ext,
[05421] = EMIT_FORMAT_S_ext,
[05422] = EMIT_FORMAT_X_ext,
[05423] = EMIT_FORMAT_P_ext,
[05424] = EMIT_FORMAT_W_ext,
[05425] = EMIT_FORMAT_D_ext,
[05426] = EMIT_FORMAT_B_ext,
[05430] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05431] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05422] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05423] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05434] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05425] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05436] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05441] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05442] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05443] = EMIT_FMOVEM_L_ext,
[05444] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05445 ... 05447] = EMIT_FMOVEM_L_ext,
[05451] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05452] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05453] = EMIT_FMOVEM_L_ext,
[05454] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05455 ... 05457] = EMIT_FMOVEM_L_ext,
[05460] = EMIT_FMOVEM_ext,	//static, predecrement
[05462] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05464] = EMIT_FMOVEM_ext,	//static, postincrement
[05466] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05470] = EMIT_FMOVEM_ext,	//static, predecrement
[05472] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05474] = EMIT_FMOVEM_ext,	//static, postincrement
[05476] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A5)
[05520] = EMIT_FORMAT_L_ext,
[05521] = EMIT_FORMAT_S_ext,
[05522] = EMIT_FORMAT_X_ext,
[05523] = EMIT_FORMAT_P_ext,
[05524] = EMIT_FORMAT_W_ext,
[05525] = EMIT_FORMAT_D_ext,
[05526] = EMIT_FORMAT_B_ext,
[05530] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05531] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05522] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05523] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05534] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05525] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05536] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05541] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05542] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05543] = EMIT_FMOVEM_L_ext,
[05544] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05545 ... 05547] = EMIT_FMOVEM_L_ext,
[05551] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05552] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05553] = EMIT_FMOVEM_L_ext,
[05554] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05555 ... 05557] = EMIT_FMOVEM_L_ext,
[05560] = EMIT_FMOVEM_ext,	//static, predecrement
[05562] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05564] = EMIT_FMOVEM_ext,	//static, postincrement
[05566] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05570] = EMIT_FMOVEM_ext,	//static, predecrement
[05572] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05574] = EMIT_FMOVEM_ext,	//static, postincrement
[05576] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A6)
[05620] = EMIT_FORMAT_L_ext,
[05621] = EMIT_FORMAT_S_ext,
[05622] = EMIT_FORMAT_X_ext,
[05623] = EMIT_FORMAT_P_ext,
[05624] = EMIT_FORMAT_W_ext,
[05625] = EMIT_FORMAT_D_ext,
[05626] = EMIT_FORMAT_B_ext,
[05630] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05631] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05622] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05623] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05634] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05625] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05636] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05641] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05642] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05643] = EMIT_FMOVEM_L_ext,
[05644] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05645 ... 05647] = EMIT_FMOVEM_L_ext,
[05651] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05652] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05653] = EMIT_FMOVEM_L_ext,
[05654] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05655 ... 05657] = EMIT_FMOVEM_L_ext,
[05660] = EMIT_FMOVEM_ext,	//static, predecrement
[05662] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05664] = EMIT_FMOVEM_ext,	//static, postincrement
[05666] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05670] = EMIT_FMOVEM_ext,	//static, predecrement
[05672] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05674] = EMIT_FMOVEM_ext,	//static, postincrement
[05676] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,A7)
[05720] = EMIT_FORMAT_L_ext,
[05721] = EMIT_FORMAT_S_ext,
[05722] = EMIT_FORMAT_X_ext,
[05723] = EMIT_FORMAT_P_ext,
[05724] = EMIT_FORMAT_W_ext,
[05725] = EMIT_FORMAT_D_ext,
[05726] = EMIT_FORMAT_B_ext,
[05730] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[05731] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[05722] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[05723] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[05734] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[05725] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[05736] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[05741] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[05742] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[05743] = EMIT_FMOVEM_L_ext,
[05744] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[05745 ... 05747] = EMIT_FMOVEM_L_ext,
[05751] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[05752] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[05753] = EMIT_FMOVEM_L_ext,
[05754] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[05755 ... 05757] = EMIT_FMOVEM_L_ext,
[05760] = EMIT_FMOVEM_ext,	//static, predecrement
[05762] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05764] = EMIT_FMOVEM_ext,	//static, postincrement
[05766] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[05770] = EMIT_FMOVEM_ext,	//static, predecrement
[05772] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[05774] = EMIT_FMOVEM_ext,	//static, postincrement
[05776] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A0,Xn)
[06020] = EMIT_FORMAT_L_ext,
[06021] = EMIT_FORMAT_S_ext,
[06022] = EMIT_FORMAT_X_ext,
[06023] = EMIT_FORMAT_P_ext,
[06024] = EMIT_FORMAT_W_ext,
[06025] = EMIT_FORMAT_D_ext,
[06026] = EMIT_FORMAT_B_ext,
[06030] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06031] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06022] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06023] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06034] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06025] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06036] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06041] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06042] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06043] = EMIT_FMOVEM_L_ext,
[06044] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06045 ... 06047] = EMIT_FMOVEM_L_ext,
[06051] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06052] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06053] = EMIT_FMOVEM_L_ext,
[06054] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06055 ... 06057] = EMIT_FMOVEM_L_ext,
[06060] = EMIT_FMOVEM_ext,	//static, predecrement
[06062] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06064] = EMIT_FMOVEM_ext,	//static, postincrement
[06066] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06070] = EMIT_FMOVEM_ext,	//static, predecrement
[06072] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06074] = EMIT_FMOVEM_ext,	//static, postincrement
[06076] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A1,Xn)
[06120] = EMIT_FORMAT_L_ext,
[06121] = EMIT_FORMAT_S_ext,
[06122] = EMIT_FORMAT_X_ext,
[06123] = EMIT_FORMAT_P_ext,
[06124] = EMIT_FORMAT_W_ext,
[06125] = EMIT_FORMAT_D_ext,
[06126] = EMIT_FORMAT_B_ext,
[06130] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06131] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06122] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06123] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06134] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06125] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06136] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06141] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06142] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06143] = EMIT_FMOVEM_L_ext,
[06144] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06145 ... 06147] = EMIT_FMOVEM_L_ext,
[06151] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06152] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06153] = EMIT_FMOVEM_L_ext,
[06154] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06155 ... 06157] = EMIT_FMOVEM_L_ext,
[06160] = EMIT_FMOVEM_ext,	//static, predecrement
[06162] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06164] = EMIT_FMOVEM_ext,	//static, postincrement
[06166] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06170] = EMIT_FMOVEM_ext,	//static, predecrement
[06172] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06174] = EMIT_FMOVEM_ext,	//static, postincrement
[06176] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A2,Xn)
[06220] = EMIT_FORMAT_L_ext,
[06221] = EMIT_FORMAT_S_ext,
[06222] = EMIT_FORMAT_X_ext,
[06223] = EMIT_FORMAT_P_ext,
[06224] = EMIT_FORMAT_W_ext,
[06225] = EMIT_FORMAT_D_ext,
[06226] = EMIT_FORMAT_B_ext,
[06230] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06231] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06222] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06223] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06234] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06225] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06236] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06241] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06242] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06243] = EMIT_FMOVEM_L_ext,
[06244] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06245 ... 06247] = EMIT_FMOVEM_L_ext,
[06251] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06252] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06253] = EMIT_FMOVEM_L_ext,
[06254] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06255 ... 06257] = EMIT_FMOVEM_L_ext,
[06260] = EMIT_FMOVEM_ext,	//static, predecrement
[06262] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06264] = EMIT_FMOVEM_ext,	//static, postincrement
[06266] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06270] = EMIT_FMOVEM_ext,	//static, predecrement
[06272] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06274] = EMIT_FMOVEM_ext,	//static, postincrement
[06276] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A3,Xn)
[06320] = EMIT_FORMAT_L_ext,
[06321] = EMIT_FORMAT_S_ext,
[06322] = EMIT_FORMAT_X_ext,
[06323] = EMIT_FORMAT_P_ext,
[06324] = EMIT_FORMAT_W_ext,
[06325] = EMIT_FORMAT_D_ext,
[06326] = EMIT_FORMAT_B_ext,
[06330] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06331] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06322] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06323] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06334] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06325] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06336] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06341] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06342] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06343] = EMIT_FMOVEM_L_ext,
[06344] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06345 ... 06347] = EMIT_FMOVEM_L_ext,
[06351] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06352] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06353] = EMIT_FMOVEM_L_ext,
[06354] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06355 ... 06357] = EMIT_FMOVEM_L_ext,
[06360] = EMIT_FMOVEM_ext,	//static, predecrement
[06362] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06364] = EMIT_FMOVEM_ext,	//static, postincrement
[06366] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06370] = EMIT_FMOVEM_ext,	//static, predecrement
[06372] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06374] = EMIT_FMOVEM_ext,	//static, postincrement
[06376] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A4,Xn)
[06420] = EMIT_FORMAT_L_ext,
[06421] = EMIT_FORMAT_S_ext,
[06422] = EMIT_FORMAT_X_ext,
[06423] = EMIT_FORMAT_P_ext,
[06424] = EMIT_FORMAT_W_ext,
[06425] = EMIT_FORMAT_D_ext,
[06426] = EMIT_FORMAT_B_ext,
[06430] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06431] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06422] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06423] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06434] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06425] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06436] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06441] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06442] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06443] = EMIT_FMOVEM_L_ext,
[06444] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06445 ... 06447] = EMIT_FMOVEM_L_ext,
[06451] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06452] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06453] = EMIT_FMOVEM_L_ext,
[06454] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06455 ... 06457] = EMIT_FMOVEM_L_ext,
[06460] = EMIT_FMOVEM_ext,	//static, predecrement
[06462] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06464] = EMIT_FMOVEM_ext,	//static, postincrement
[06466] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06470] = EMIT_FMOVEM_ext,	//static, predecrement
[06472] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06474] = EMIT_FMOVEM_ext,	//static, postincrement
[06476] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A5,Xn)
[06520] = EMIT_FORMAT_L_ext,
[06521] = EMIT_FORMAT_S_ext,
[06522] = EMIT_FORMAT_X_ext,
[06523] = EMIT_FORMAT_P_ext,
[06524] = EMIT_FORMAT_W_ext,
[06525] = EMIT_FORMAT_D_ext,
[06526] = EMIT_FORMAT_B_ext,
[06530] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06531] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06522] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06523] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06534] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06525] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06536] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06541] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06542] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06543] = EMIT_FMOVEM_L_ext,
[06544] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06545 ... 06547] = EMIT_FMOVEM_L_ext,
[06551] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06552] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06553] = EMIT_FMOVEM_L_ext,
[06554] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06555 ... 06557] = EMIT_FMOVEM_L_ext,
[06560] = EMIT_FMOVEM_ext,	//static, predecrement
[06562] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06564] = EMIT_FMOVEM_ext,	//static, postincrement
[06566] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06570] = EMIT_FMOVEM_ext,	//static, predecrement
[06572] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06574] = EMIT_FMOVEM_ext,	//static, postincrement
[06576] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A6,Xn)
[06620] = EMIT_FORMAT_L_ext,
[06621] = EMIT_FORMAT_S_ext,
[06622] = EMIT_FORMAT_X_ext,
[06623] = EMIT_FORMAT_P_ext,
[06624] = EMIT_FORMAT_W_ext,
[06625] = EMIT_FORMAT_D_ext,
[06626] = EMIT_FORMAT_B_ext,
[06630] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06631] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06622] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06623] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06634] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06625] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06636] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06641] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06642] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06643] = EMIT_FMOVEM_L_ext,
[06644] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06645 ... 06647] = EMIT_FMOVEM_L_ext,
[06651] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06652] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06653] = EMIT_FMOVEM_L_ext,
[06654] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06655 ... 06657] = EMIT_FMOVEM_L_ext,
[06660] = EMIT_FMOVEM_ext,	//static, predecrement
[06662] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06664] = EMIT_FMOVEM_ext,	//static, postincrement
[06666] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06670] = EMIT_FMOVEM_ext,	//static, predecrement
[06672] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06674] = EMIT_FMOVEM_ext,	//static, postincrement
[06676] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(A7,Xn)
[06720] = EMIT_FORMAT_L_ext,
[06721] = EMIT_FORMAT_S_ext,
[06722] = EMIT_FORMAT_X_ext,
[06723] = EMIT_FORMAT_P_ext,
[06724] = EMIT_FORMAT_W_ext,
[06725] = EMIT_FORMAT_D_ext,
[06726] = EMIT_FORMAT_B_ext,
[06730] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[06731] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[06722] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[06723] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[06734] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[06725] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[06736] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[06741] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[06742] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[06743] = EMIT_FMOVEM_L_ext,
[06744] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[06745 ... 06747] = EMIT_FMOVEM_L_ext,
[06751] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[06752] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[06753] = EMIT_FMOVEM_L_ext,
[06754] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[06755 ... 06757] = EMIT_FMOVEM_L_ext,
[06760] = EMIT_FMOVEM_ext,	//static, predecrement
[06762] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06764] = EMIT_FMOVEM_ext,	//static, postincrement
[06766] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[06770] = EMIT_FMOVEM_ext,	//static, predecrement
[06772] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[06774] = EMIT_FMOVEM_ext,	//static, postincrement
[06776] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(xxx).W
[07020] = EMIT_FORMAT_L_ext,
[07021] = EMIT_FORMAT_S_ext,
[07022] = EMIT_FORMAT_X_ext,
[07023] = EMIT_FORMAT_P_ext,
[07024] = EMIT_FORMAT_W_ext,
[07025] = EMIT_FORMAT_D_ext,
[07026] = EMIT_FORMAT_B_ext,
[07030] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[07031] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[07022] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[07023] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[07034] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[07025] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[07036] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[07041] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[07042] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[07043] = EMIT_FMOVEM_L_ext,
[07044] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[07045 ... 07047] = EMIT_FMOVEM_L_ext,
[07051] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[07052] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[07053] = EMIT_FMOVEM_L_ext,
[07054] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[07055 ... 07057] = EMIT_FMOVEM_L_ext,
[07060] = EMIT_FMOVEM_ext,	//static, predecrement
[07062] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07064] = EMIT_FMOVEM_ext,	//static, postincrement
[07066] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[07070] = EMIT_FMOVEM_ext,	//static, predecrement
[07072] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07074] = EMIT_FMOVEM_ext,	//static, postincrement
[07076] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(xxx).L
[07120] = EMIT_FORMAT_L_ext,
[07121] = EMIT_FORMAT_S_ext,
[07122] = EMIT_FORMAT_X_ext,
[07123] = EMIT_FORMAT_P_ext,
[07124] = EMIT_FORMAT_W_ext,
[07125] = EMIT_FORMAT_D_ext,
[07126] = EMIT_FORMAT_B_ext,
[07130] = EMIT_FMOVE_L_ext,	//FPn,<ea>
[07131] = EMIT_FMOVE_S_ext,	//FPn,<ea>
[07122] = EMIT_FMOVE_X_ext,	//FPn,<ea>
[07123] = EMIT_FMOVE_P_ext,	//FPn,<ea>
[07134] = EMIT_FMOVE_W_ext,	//FPn,<ea>
[07125] = EMIT_FMOVE_D_ext,	//FPn,<ea>
[07136] = EMIT_FMOVE_B_ext,	//FPn,<ea>
[07141] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[07142] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[07143] = EMIT_FMOVEM_L_ext,
[07144] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[07145 ... 07147] = EMIT_FMOVEM_L_ext,
[07151] = EMIT_FMOVE_L_ext,	//FPIAR,<ea>
[07152] = EMIT_FMOVE_L_ext,	//FPSR,<ea>
[07153] = EMIT_FMOVEM_L_ext,
[07154] = EMIT_FMOVE_L_ext,	//FPCR,<ea>
[07155 ... 07157] = EMIT_FMOVEM_L_ext,
[07160] = EMIT_FMOVEM_ext,	//static, predecrement
[07162] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07164] = EMIT_FMOVEM_ext,	//static, postincrement
[07166] = EMIT_FMOVEM_ext,	//dynamic. postincrement
[07170] = EMIT_FMOVEM_ext,	//static, predecrement
[07172] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07174] = EMIT_FMOVEM_ext,	//static, postincrement
[07176] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(d16,PC)
[07220] = EMIT_FORMAT_L_ext,
[07221] = EMIT_FORMAT_S_ext,
[07222] = EMIT_FORMAT_X_ext,
[07223] = EMIT_FORMAT_P_ext,
[07224] = EMIT_FORMAT_W_ext,
[07225] = EMIT_FORMAT_D_ext,
[07226] = EMIT_FORMAT_B_ext,
[07241] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[07242] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[07243] = EMIT_FMOVEM_L_ext,
[07244] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[07245 ... 07247] = EMIT_FMOVEM_L_ext,
[07260] = EMIT_FMOVEM_ext,	//static, predecrement
[07262] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07264] = EMIT_FMOVEM_ext,	//static, postincrement
[07266] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//(PC,Xn) & other relatives (further decoding of the _ext required!)
[07320] = EMIT_FORMAT_L_ext,
[07321] = EMIT_FORMAT_S_ext,
[07322] = EMIT_FORMAT_X_ext,
[07323] = EMIT_FORMAT_P_ext,
[07324] = EMIT_FORMAT_W_ext,
[07325] = EMIT_FORMAT_D_ext,
[07326] = EMIT_FORMAT_B_ext,
[07341] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[07342] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[07343] = EMIT_FMOVEM_L_ext,
[07344] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[07345 ... 07347] = EMIT_FMOVEM_L_ext,
[07360] = EMIT_FMOVEM_ext,	//static, predecrement
[07362] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07364] = EMIT_FMOVEM_ext,	//static, postincrement
[07366] = EMIT_FMOVEM_ext,	//dynamic. postincrement
//#<data>
[07420] = EMIT_FORMAT_L_ext,
[07421] = EMIT_FORMAT_S_ext,
[07422] = EMIT_FORMAT_X_ext,
[07423] = EMIT_FORMAT_P_ext,
[07424] = EMIT_FORMAT_W_ext,
[07425] = EMIT_FORMAT_D_ext,
[07426] = EMIT_FORMAT_B_ext,
[07441] = EMIT_FMOVE_L_ext,	//<ea>,FPIAR
[07442] = EMIT_FMOVE_L_ext,	//<ea>,FPSR
[07443] = EMIT_FMOVEM_L_ext,
[07444] = EMIT_FMOVE_L_ext,	//<ea>,FPCR
[07445 ... 07447] = EMIT_FMOVEM_L_ext,
[07460] = EMIT_FMOVEM_ext,	//static, predecrement
[07462] = EMIT_FMOVEM_ext,	//dynamic, predecrement
[07464] = EMIT_FMOVEM_ext,	//static, postincrement
[07466] = EMIT_FMOVEM_ext,	//dynamic. postincrement
}

/* FPU Instructions, Table 2; bits[6:0]*///bit 6 assumed 0
static EMIT_Function JumpTableOp[64] = {
[0x00] = EMIT_FMOVE,
[0x01] = EMIT_FINT,
[0x02] = EMIT_FSINH,
[0x03] = EMIT_FINTRZ,
[0x04] = EMIT_FSQRT,
[0x06] = EMIT_FLOGNP1,
[0x08] = EMIT_FETOXM1,
[0x09] = EMIT_FTANH,
[0x0A] = EMIT_FATAN,
[0x0B] = EMIT_FINTRN,
[0x0C] = EMIT_FASIN,
[0x0D] = EMIT_FATANH,
[0x0E] = EMIT_FSIN,
[0x0F] = EMIT_FTAN,
[0x10] = EMIT_FETOX,
[0x11] = EMIT_FTWOTOX,
[0x12] = EMIT_FTENTOX,
[0x13] = EMIT_FINTRP,
[0x14] = EMIT_FLOGN,
[0x15] = EMIT_FLOG10,
[0x16] = EMIT_FLOG2,
[0x18] = EMIT_FABS,
[0x19] = EMIT_FCOSH,
[0x1A] = EMIT_FNEG,
[0x1B] = EMIT_FINTRM,
[0x1C] = EMIT_FACOS,
[0x1D] = EMIT_FCOS,
[0x1E] = EMIT_FGETEXP,
[0x1F] = EMIT_FGETMAN,
[0x20] = EMIT_FDIV,
[0x21] = EMIT_FMOD,
[0x22] = EMIT_FADD,
[0x23] = EMIT_FMUL,
[0x24] = EMIT_FSGLDIV,	//single precision
[0x25] = EMIT_FREM,
[0x26] = EMIT_FSCALE,
[0x27] = EMIT_FSGMUL,	//single precision
[0x28] = EMIT_FSUB,
[0x2D] = EMIT_FMOD,
[0x30 ... 0x37] = EMIT_FSINCOS,
[0x38] = EMIT_FCMP,
[0x3A] = EMIT_FTST,
[0x40] = EMIT_FMOVE_S_dst,	//rounded to single
[0x41] = EMIT_FSQRT_S,	//rounded to single
[0x44] = EMIT_FMOVE_D_dst,	//rounded to double
[0x45] = EMIT_FSQRT_D,	//rounded to double
[0x58] = EMIT_FABS_S,	//rounded to single
[0x5A] = EMIT_FNEG_S,	//rounded to single
[0x5C] = EMIT_FABS_D,	//rounded to double
[0x5E] = EMIT_FNEG_D,	//rounded to double
[0x60] = EMIT_FDIV_S,	//rounded to single
[0x62] = EMIT_FADD_S,	//rounded to single
[0x63] = EMIT_FMUL_S,	//rounded to single
[0x64] = EMIT_FDIV_D,	//rounded to double
[0x66] = EMIT_FADD_D,	//rounded to double
[0x67] = EMIT_FMUL_D,	//rounded to double
[0x68] = EMIT_FSUB_S,	//rounded to single
[0x6C] = EMIT_FSUB_D,	//rounded to double
}
/* CC instructions, Table 1; bits[5:0]*/
static EMIT_Function JumpTableCase1[64] = {
[000 ... 007] = EMIT_FSCC_reg,
[010 ... 017] = EMIT_FDBCC,
[020 ... 047] = EMIT_FSCC_mem,
{050 ... 071] = EMIT_FSCC_ext,
[072 ... 074] = EMIT_FTRAPCC,
}
/* Branch, Table 1; bits [4:0]*///bit 5 of the CC is always 0
static EMIT_Function JumpTableCase2[32] = {
//Misc tests
[000] = EMIT_FBF,
[017] = EMIT_FBT,
[020] = EMIT_FBSF,
[021] = EMIT_FBSEQ,
[036] = EMIT_FBSNE,
[037] = EMIT_FBST,
//IEEE Aware tests
[001] = EMIT_FBEQ,
[002] = EMIT_FBOGT,
[003] = EMIT_FBOGE,
[004] = EMIT_FBOLT,
[005] = EMIT_FBOLE,
[006] = EMIT_FBOGL,
[007] = EMIT_FBOR,
[010] = EMIT_FBUN,
[011] = EMIT_FBUEQ,
[012] = EMIT_FBUGT,
[013] = EMIT_FBUGE,
[014] = EMIT_FBULT,
[015] = EMIT_FBULE,
[016] = EMIT_FBNE,
//IEEE Nonaware tests
[022] = EMIT_FBGT,
[023] = EMIT_FBGE,
[024] = EMIT_FBLT,
[025] = EMIT_FBLE,
[026] = EMIT_FBGL,
[027] = EMIT_FBGLE,
[030] = EMIT_FBNGLE,
[031] = EMIT_FBNGL,
[032] = EMIT_FBNLE,
[033] = EMIT_FBNLT,
[034] = EMIT_FBNGE,
[035] = EMIT_FBNGT,
}

uint32_t *EMIT_FMOVECR(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k) /* FMOVECR only pulls extended-precision constants to a FP register */
{
	 uint8_t FPn = (( opcode2 && 0x0380) >> 7);
}
/* Any format function should preload specified registers according to format and jump to FPU Instruction table. */
uint32_t *EMIT_FORMAT_B_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_B_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_B(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_D_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_D_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_L_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_L_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_L(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_P_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_P_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_P(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_S_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_S_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_W_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_W_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_W(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FORMAT_X_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_X_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FORMAT_X(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

/* any and all FMOVE instructions, this can 2 or 3 nested jumps depending on the encoding */
uint32_t *EMIT_FMOVE_B_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_B_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_B(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_D_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_D_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_D(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_L_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_L_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_L(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_P_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_P_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_P(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_S_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_S_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_S(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_W_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_W_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_W(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVE_X_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_X_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVE_X(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVEM_L_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVEM_L_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVEM_L(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOVEM_ext(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t ext, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVEM_mem(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)
uint32_t *EMIT_FMOVEM(uint32_t *ptr, uint16_t opcode, uint16_t opcode2, uint16_t **ptr_m68k)

/* FPU Monadic Operations */
uint32_t *EMIT_FABS(uint32_t *ptr, uint16_t **ptr_m68k)
{
	uint16_t FPCR
}
uint32_t *EMIT_FABS_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FABS_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FACOS(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FASIN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FATAN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FCOS(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FCOSH(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FETOX(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FETOXM1(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FGETEXP(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FGETMAN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FINT(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FINTRZ(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FLOGN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FLOGNP1(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FLOG10(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FLOG2(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FNEG(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FNEG_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FNEG_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSIN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSINH(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSQRT(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSQRT_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FSQRT_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FTAN(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FTANH(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FTENTOX(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FTWOTOX(uint32_t *ptr, uint16_t **ptr_m68k)

/* FPU Dyadic Operations */
uint32_t *EMIT_FADD(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FADD_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FADD_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FCMP(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FDIV(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FDIV_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FDIV_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FMOD(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FMUL(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FMUL_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FMUL_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FREM(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSCALE(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSUB(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSUB_S(uint32_t *ptr, uint16_t **ptr_m68k)
uint32_t *EMIT_FSUB_D(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSGLDIV(uint32_t *ptr, uint16_t **ptr_m68k)

uint32_t *EMIT_FSGMUL(uint32_t *ptr, uint16_t **ptr_m68k)
