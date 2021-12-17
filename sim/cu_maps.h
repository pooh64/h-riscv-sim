#pragma once

#include "sim/common.h"
#include <array>
#include <ostream>

namespace cpu
{

union Instr {
	u32 raw;
	struct {
		u32 op : 7;
		u32 rd : 5;
		u32 funct3 : 3;
		u32 rs1 : 5;
		u32 rs2 : 5;
		u32 funct7 : 7;
	} __attribute__((packed));
};
static_assert(sizeof(Instr) == 4);

enum class CUIType : u8 {
	R,
	I,
	S,
	B,
	J,
	U,
};

enum class CUResSrc : u8 {
	X = 0,
	ALU = 0,
	MEM,
	PC,
};

enum class CUALUSrc1 : u8 {
	X = 0,
	R = 0,
	PC = 1,
};

enum class CUALUSrc2 : u8 {
	X = 0,
	R = 0,
	I = 1,
};

enum class CUALUCtrl : u8 { X = 0, ADD = 0, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND, ARG2 };

enum class CUCMPCtrl : u8 {
	X = 0,
	EQ = 0,
	NE,
	LT,
	GE,
	LTU,
	GEU,
};

enum class CUMemOp : u8 {
	W,
	H,
	B,
	X = W,
};

struct CUFlags_Main {
	CUIType itype : 3 = CUIType::R;
	bool reg_write : 1 = false;
	CUALUSrc1 alu_src1 : 1 = CUALUSrc1::X;
	CUALUSrc2 alu_src2 : 1 = CUALUSrc2::X;
	CUALUCtrl alu_control : 4 = CUALUCtrl::X;
	CUCMPCtrl cmp_control : 4 = CUCMPCtrl::X;
	CUResSrc result_src : 2 = CUResSrc::X;
	bool mem_write : 1 = false;
	CUMemOp mem_op : 2 = CUMemOp::W;
	bool mem_sgne : 1 = false; 
	bool branch : 1 = false;
	bool jump : 1 = false;
	bool jreg : 1 = false;
	bool intpt : 1 = false;
	bool opcode_ok : 1 = false;
} __attribute__((packed));

CUFlags_Main GetCUFlags(Instr inst);
char const *GetOpcodeStr(Instr inst);
std::ostream &operator<<(std::ostream &os, Instr inst);

#if 0
template <typename T, size_t sz>
struct MapTab {
	std::array<T, sz> tab;

	constexpr MapTab(std::initializer_list<std::pair<size_t, T>> list)
	{
		for (auto const &e : list)
			tab[e.first] = e.second;
	}
};

extern const MapTab<CUFlags_Main, 128> cu_flags_map;
static inline CUFlags_Main GetCUFlags(Instr inst)
{
	auto res = cu_flags_map.tab[inst.op];
	// if (!res.opcode_ok)
	//	printf("Unknown opcode: %x\n", inst.op);
	// assert(res.opcode_ok);
	return res;
}

union CUALU_in {
	struct {
		u8 alu_op : 2;
		u8 funct3 : 3;
		u8 funct7_5 : 1;
		u8 op_5 : 1;
	} __attribute__((packed));
	u8 raw;
};

extern const MapTab<CUALUCtrl, 128> cu_alu_map;
static inline CUALUCtrl GetALUControl(Instr inst, CUFlags_Main cu_flags)
{
	CUALU_in cfa_in;
	cfa_in.alu_op = cu_flags.alu_op;
	cfa_in.funct3 = inst.funct3;
	cfa_in.funct7_5 = (u8)(inst.funct7 >> 5); // start from zero?
	cfa_in.op_5 = inst.op;
	return cu_alu_map.tab[cfa_in.raw];
}
#endif

} // namespace cpu