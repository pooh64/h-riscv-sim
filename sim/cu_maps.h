#pragma once

#include "sim/common.h"
#include <array>

namespace cpu
{

template <typename T, size_t sz>
struct MapTab {
	std::array<T, sz> tab;

	constexpr MapTab(std::initializer_list<std::pair<size_t, T>> list)
	{
		for (auto const &e : list)
			tab[e.first] = e.second;
	}
};

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

enum class CUIMMSrc : u8 {
	X = 0,
	I = 0,
	S,
	B,
	J,
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

enum class CUALUCtrl : u8 {
	X = 0,
	ADD = 0,
	SUB,
	SLL,
	SLT,
	SLTU,
	XOR,
	SRL,
	SRA,
	OR,
	AND,
};

enum class CUCMPCtrl : u8 {
	X = 0,
	EQ = 0,
	NE,
	LT,
	GE,
	LTU,
	GEU,
};

struct CUFlags_Main {
	bool reg_write : 1;
	CUALUSrc1 alu_src1 : 1;
	CUALUSrc2 alu_src2 : 1;
	CUALUCtrl alu_control : 4;
	CUCMPCtrl cmp_control : 4;
	CUIMMSrc imm_src : 2;
	CUResSrc result_src : 2;
	bool mem_write : 1;
	bool branch : 1;
	bool jump : 1;
	bool jreg : 1;
	bool intpt : 1;
	bool opcode_ok : 1;
	constexpr CUFlags_Main(bool reg_write_, CUALUSrc1 alu_src1_, CUALUSrc2 alu_src2_,
			       CUALUCtrl alu_control_, CUCMPCtrl cmp_control_, CUIMMSrc imm_src_,
			       CUResSrc result_src_, bool mem_write_, bool branch_, bool jump_, bool jreg_,
			       bool intpt_)
	    : reg_write(reg_write_), alu_src1(alu_src1_), alu_src2(alu_src2_),
	      alu_control(alu_control_), cmp_control(cmp_control_), imm_src(imm_src_),
	      result_src(result_src_), mem_write(mem_write_), branch(branch_), jump(jump_), jreg(jreg_),
	      intpt(intpt_), opcode_ok(1)
	{
	}
	constexpr CUFlags_Main()
	    : reg_write(0), alu_src1(CUALUSrc1::X), alu_src2(CUALUSrc2::X), alu_control(CUALUCtrl::X),
	      cmp_control(CUCMPCtrl::X), imm_src(CUIMMSrc::X), result_src(CUResSrc::X),
	      mem_write(0), branch(0), jump(0), jreg(0), intpt(0), opcode_ok(0){};
} __attribute__((packed));

CUFlags_Main GetCUFlags(Instr inst);

#if 0
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