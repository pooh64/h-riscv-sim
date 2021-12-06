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
	TYPE_I,
	TYPE_S,
	TYPE_B,
	TYPE_J,
};

enum class CUResultSrc : u8 {
	ALU,
	MEM,
	PC,
};

struct CUFlags_Main {
	bool reg_write : 1;
	CUIMMSrc imm_src : 2;
	bool alu_src2_imm : 1;
	bool mem_write : 1;
	CUResultSrc result_src : 2;
	bool branch : 1;
	bool jump : 1;
	u8 alu_op : 2;
	bool intpt : 1;
	bool opcode_ok : 1;
	constexpr CUFlags_Main(bool reg_write_, CUResultSrc result_src_, CUIMMSrc imm_src_,
			       bool alu_src2_imm_, bool mem_write_, bool branch_, bool jump_,
			       u8 alu_op_, bool intpt_)
	    : reg_write(reg_write_), imm_src(imm_src_), alu_src2_imm(alu_src2_imm_),
	      mem_write(mem_write_), result_src(result_src_), branch(branch_), jump(jump_),
	      alu_op(alu_op_), intpt(intpt_), opcode_ok(1)
	{
	}
	constexpr CUFlags_Main()
	    : reg_write(0), imm_src(CUIMMSrc::TYPE_I), alu_src2_imm(0), mem_write(0),
	      result_src(CUResultSrc::ALU), branch(0), jump(0), alu_op(0), intpt(0), opcode_ok(0){};
} __attribute__((packed));

extern const MapTab<CUFlags_Main, 128> cu_flags_map;
static inline CUFlags_Main GetCUFlags(Instr inst)
{
	auto res = cu_flags_map.tab[inst.op];
	// if (!res.opcode_ok)
	//	printf("Unknown opcode: %x\n", inst.op);
	// assert(res.opcode_ok);
	return res;
}

enum class CUALUControl : u8 {
	ADD,
	SUB,
	AND,
	OR,
	XOR,
	SLT,
	SLTU,
	GE,
	GEU,
};

union CUALU_in {
	struct {
		u8 alu_op : 2;
		u8 funct3 : 3;
		u8 funct7_5 : 1;
		u8 op_5 : 1;
	} __attribute__((packed));
	u8 raw;
};

extern const MapTab<CUALUControl, 128> cu_alu_map;
static inline CUALUControl GetALUControl(Instr inst, CUFlags_Main cu_flags)
{
	CUALU_in cfa_in;
	cfa_in.alu_op = cu_flags.alu_op;
	cfa_in.funct3 = inst.funct3;
	cfa_in.funct7_5 = (u8)(inst.funct7 >> 5); // start from zero?
	cfa_in.op_5 = inst.op;
	return cu_alu_map.tab[cfa_in.raw];
}

} // namespace cpu