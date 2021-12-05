#pragma once

#include "sim/cu_maps.h"
#include <cstring>

namespace cpu
{
struct CPU;
struct Module {
	virtual inline void init() = 0;
	virtual inline void tick(CPU &) = 0;
};
#define CPU_MODULE                                                                                 \
	inline void init();                                                                        \
	inline void tick(CPU &);

template <typename T>
struct TickReg final : Module {
	T mutable r, w;
	TickReg()
	{
		init();
	}

	inline void init()
	{
		memset(&r, 0, sizeof(r));
		memset(&w, 0, sizeof(w));
	};
	inline void tick(CPU &)
	{
		r = w;
	}
};

struct MMU {
	u32 *base{nullptr};
	u32 *end{nullptr};

	void setMem(u32 *base_, u32 sz_)
	{
		base = base_;
		end = base_ + sz_ / sizeof(u32);
		assert(sz_ % sizeof(u32) == 0);
	}

	inline void load(u32 a, u32 *d)
	{
		u32 *addr = (u32 *)((u8 *)base + a);
		assert(a % 4 == 0);
		assert(addr < end);
		*d = *addr;
	}

	inline void store(u32 a, u32 d)
	{
		u32 *addr = (u32 *)((u8 *)base + a);
		assert(a % 4 == 0);
		assert(addr < end);
		d = *addr;
	}
};

struct PL_Fetch final : Module {
	CPU_MODULE;
	struct State {
		u32 pc;
	};
	TickReg<State> s;
};

struct PL_Decode final : Module {
	CPU_MODULE;
	struct State {
		Instr inst;
		u32 pc;
		u32 pc_next;
		u8 v;
	};
	TickReg<State> s;
	struct Regfile final : Module {
		CPU_MODULE;
		u32 gpr[32] = {};
		void dump()
		{
			for (int i = 0; i < 32; ++i)
				printf("[%d] = %x\n", i, gpr[i]);
		}
	} regfile;
};

struct PL_Execute final : Module {
	CPU_MODULE;
	struct State {
		u8 reg_write;
		u8 mem_write;
		CUResultSrc result_src;
		u8 jump;
		u8 branch;
		CUALUControl alu_ctrl;
		u8 alu_src2_imm;
		//
		u32 pc;
		u32 pc_next;
		u32 rs1v;
		u32 rs2v;
		u32 imm_ext;
		u8 rs1a;
		u8 rs2a;
		u8 rda;
		u8 v;
	};
	TickReg<State> s;
	bool pc_r{0};
};

struct PL_Memory final : Module {
	CPU_MODULE;
	struct State {
		u8 reg_write;
		u8 mem_write;
		u8 v;
		CUResultSrc result_src;
		//
		u8 reg_addr;
		u32 pc_next;
		u32 mem_wdata;
		u32 alu_res;
	};
	TickReg<State> s;
};

struct PL_Wback final : Module {
	CPU_MODULE;
	struct State {
		u8 reg_write;
		u8 reg_addr;
		u32 reg_wdata;
	};
	TickReg<State> s;
};

struct PL_HU final : Module {
	CPU_MODULE;
	enum class FWD : u8 {
		NO = 0b00,
		MEM = 0b01,
		WB = 0b10,
	};
	inline FWD getRegFWD(CPU &cpu, u8 rsa);
};

struct CPU final : Module {
	CPU_MODULE;
	MMU mmu;
	CPU() {}

	PL_Fetch fe;
	PL_Decode de;
	PL_Execute ex;
	PL_Memory mem;
	PL_Wback wb;
	PL_HU hu;
};

inline void CPU::init() {}
inline void CPU::tick(CPU &cpu)
{
	wb.tick(cpu);
	mem.tick(cpu);
	ex.tick(cpu);
	de.tick(cpu);
	fe.tick(cpu);

	hu.tick(cpu);
}

/**********************************************************/
inline void PL_Fetch::init() {}
inline void PL_Fetch::tick(CPU &cpu)
{
	cpu.mmu.load(cpu.fe.s.r.pc, &cpu.de.s.w.inst.raw);

	u32 pc_next;
	if (!cpu.ex.pc_r)
		pc_next = cpu.fe.s.r.pc + 4;
	else
		pc_next = cpu.ex.s.r.pc + cpu.ex.s.r.imm_ext;

	cpu.fe.s.w.pc = pc_next;

	cpu.de.s.w.v = 0; // hu
	cpu.de.s.w.pc = cpu.fe.s.r.pc;
	cpu.de.s.w.pc_next = pc_next;
}

/**********************************************************/
inline constexpr u32 CPUSignExtend(u32 raw_, CUIMMSrc imm_src)
{
	union imm_layout {
		u32 raw;
		struct {
			u32 _pad0 : 20;
			u32 imm0 : 12;
		} __attribute__((packed)) fi;
		struct {
			u32 imm0 : 12;
			u32 se : 20;
		} __attribute__((packed)) di;
		struct {
			u32 _pad0 : 7;
			u32 imm0 : 5;
			u32 _pad1 : 13;
			u32 imm1 : 7;
		} __attribute__((packed)) fs;
		struct {
			u32 imm0 : 5;
			u32 imm1 : 7;
			u32 se : 20;
		} __attribute__((packed)) ds;
		struct {
			u32 _pad0 : 7;
			u32 imm2 : 1;
			u32 imm0 : 4;
			u32 _pad2 : 13;
			u32 imm1 : 6;
			u32 sgn : 1;
		} __attribute__((packed)) fb;
		struct {
			u32 imm0 : 4;
			u32 imm1 : 6;
			u32 imm2 : 1;
			u32 se : 21;
		} __attribute__((packed)) db;
		struct {
			u32 _pad0 : 12;
			u32 imm2 : 8;
			u32 imm1 : 1;
			u32 imm0 : 10;
			u32 sgn : 1;
		} __attribute__((packed)) fj;
		struct {
			u32 imm0 : 10;
			u32 imm1 : 1;
			u32 imm2 : 8;
			u32 se : 13;
		} __attribute__((packed)) dj;
	} in{.raw = raw_}, out{.raw = 0};
	u8 sgn = (in.raw >> 31) & 1;

	switch (imm_src) {
	case CUIMMSrc::TYPE_I:
		out.di.imm0 = in.fi.imm0;
		if (sgn)
			out.di.se--;
		break;
	case CUIMMSrc::TYPE_S:
		out.ds.imm0 = in.fs.imm0;
		out.ds.imm1 = in.fs.imm1;
		if (sgn)
			out.ds.se--;
		break;
	case CUIMMSrc::TYPE_B:
		out.db.imm0 = in.fb.imm0;
		out.db.imm1 = in.fb.imm1;
		out.db.imm2 = in.fb.imm2;
		if (sgn)
			out.db.se--;
		break;
	case CUIMMSrc::TYPE_J:
		out.dj.imm0 = in.fj.imm0;
		out.dj.imm1 = in.fj.imm1;
		out.dj.imm2 = in.fj.imm2;
		if (sgn)
			out.dj.se--;
		break;
	default:
		assert(0);
	};
	return out.raw;
}

inline void PL_Decode::Regfile::init() {}
inline void PL_Decode::Regfile::tick(CPU &cpu)
{
	u8 a1 = cpu.de.s.r.inst.rs1;
	u8 a2 = cpu.de.s.r.inst.rs2;
	u8 a3 = cpu.wb.s.r.reg_addr;
	u8 d3 = cpu.wb.s.r.reg_wdata;
	u8 we3 = cpu.wb.s.r.reg_write;

	cpu.ex.s.w.rs1v = gpr[a1];
	cpu.ex.s.w.rs2v = gpr[a2];
	if (we3) {
		gpr[a3] = d3;
		gpr[0] = 0;
	}
}

inline void PL_Decode::init() {}
inline void PL_Decode::tick(CPU &cpu)
{
	auto inst = cpu.de.s.r.inst;

	CUFlags_Main cf = GetCUFlags(inst);

	cpu.ex.s.w.reg_write = cf.reg_write;
	cpu.ex.s.w.result_src = cf.result_src;
	cpu.ex.s.w.mem_write = cf.mem_write;
	cpu.ex.s.w.jump = cf.jump;
	cpu.ex.s.w.branch = cf.branch;
	cpu.ex.s.w.alu_ctrl = GetALUControl(inst, cf);
	cpu.ex.s.w.alu_src2_imm = cf.alu_src2_imm;
	//
	cpu.ex.s.w.imm_ext = CPUSignExtend(inst.raw, cf.imm_src);
	cpu.ex.s.w.pc = cpu.de.s.r.pc;
	cpu.ex.s.w.pc_next = cpu.de.s.r.pc_next;
	cpu.ex.s.w.rs1a = inst.rs1;
	cpu.ex.s.w.rs2a = inst.rs2;
	cpu.ex.s.w.rda = inst.rd;
	cpu.ex.s.w.v = cpu.de.s.r.v; // hu

	cpu.de.regfile.tick(cpu);
}

/**********************************************************/
inline constexpr u32 ALUOperator(CUALUControl alu_ctrl, u32 a, u32 b)
{
	switch (alu_ctrl) {
	case CUALUControl::ADD:
		return a + b;
	case CUALUControl::SUB:
		return a - b;
	case CUALUControl::AND:
		return a & b;
	case CUALUControl::OR:
		return a | b;
	case CUALUControl::XOR:
		return a ^ b;
	case CUALUControl::SLT:
		return !!((i32)a < (i32)b);
	case CUALUControl::SLTU:
		return !!(a < b);
	case CUALUControl::GE:
		return !!((i32)a >= (i32)b);
	case CUALUControl::GEU:
		return !!(a >= b);
	default:
		assert(0);
	};
}

inline void PL_Execute::init() {}
inline void PL_Execute::tick(CPU &cpu)
{
	cpu.mem.s.w.reg_write = cpu.ex.s.r.reg_write; // && v
	cpu.mem.s.w.mem_write = cpu.ex.s.r.mem_write; // && v
	cpu.mem.s.w.v = cpu.ex.s.r.v;		      // hu

	cpu.mem.s.w.result_src = cpu.ex.s.r.result_src;
	cpu.mem.s.w.reg_addr = cpu.ex.s.r.rda;

	auto fwd_select = [&cpu](PL_HU::FWD hu_fwd, u32 rsv) {
		switch (hu_fwd) {
		case PL_HU::FWD::NO:
			return rsv;
		case PL_HU::FWD::MEM:
			return cpu.mem.s.r.alu_res;
		case PL_HU::FWD::WB:
			return cpu.wb.s.r.reg_wdata;
		}
		assert(0);
	};
	u32 rs1v = fwd_select(cpu.hu.getRegFWD(cpu, cpu.ex.s.r.rs1a), cpu.ex.s.r.rs1v);
	u32 rs2v = fwd_select(cpu.hu.getRegFWD(cpu, cpu.ex.s.r.rs2a), cpu.ex.s.r.rs2v);

	if (cpu.ex.s.r.alu_src2_imm)
		rs2v = cpu.ex.s.r.imm_ext;

	u32 alu_res = ALUOperator(cpu.ex.s.r.alu_ctrl, rs1v, rs2v);
	cpu.mem.s.w.alu_res = alu_res;

	cpu.ex.pc_r = cpu.ex.s.r.jump || (cpu.ex.s.r.branch && (alu_res == 0));
	cpu.mem.s.w.mem_wdata = rs2v;
	cpu.mem.s.w.pc_next = cpu.ex.s.r.pc_next;
}

/**********************************************************/
inline void PL_Memory::init() {}
inline void PL_Memory::tick(CPU &cpu)
{
	u32 mmu_rd;
	if (cpu.mem.s.r.result_src == CUResultSrc::MEM) {
		cpu.mmu.load(cpu.mem.s.r.alu_res, &mmu_rd);
	}
	if (cpu.mem.s.r.mem_write && !cpu.mem.s.r.v) {
		cpu.mmu.store(cpu.mem.s.r.alu_res, cpu.mem.s.r.mem_wdata);
	}

	switch (cpu.mem.s.r.result_src) {
	case CUResultSrc::ALU:
		cpu.wb.s.w.reg_wdata = cpu.mem.s.r.alu_res;
		break;
	case CUResultSrc::MEM:
		cpu.wb.s.w.reg_wdata = mmu_rd;
		break;
	case CUResultSrc::PC:
		cpu.wb.s.w.reg_wdata = cpu.mem.s.r.pc_next;
		break;
	default:
		assert(0);
	}

	cpu.wb.s.w.reg_write = cpu.mem.s.r.reg_write && !cpu.mem.s.r.v;
	cpu.wb.s.w.reg_addr = cpu.mem.s.r.reg_addr;
}

/**********************************************************/
inline void PL_Wback::init() {}
inline void PL_Wback::tick(CPU &cpu)
{
	/* nop */
}

/**********************************************************/
inline PL_HU::FWD PL_HU::getRegFWD(CPU &cpu, u8 rsa)
{
	if (cpu.mem.s.r.reg_write && (rsa == cpu.mem.s.r.reg_addr))
		return PL_HU::FWD::MEM;

	if (cpu.wb.s.r.reg_write && (rsa == cpu.wb.s.r.reg_addr))
		return PL_HU::FWD::WB;

	return PL_HU::FWD::NO;
}

inline void PL_HU::init() {}
inline void PL_HU::tick(CPU &cpu)
{
	bool load_hazard =
	    (cpu.ex.s.r.result_src == CUResultSrc::MEM) &
	    ((cpu.ex.s.r.rda == cpu.de.s.r.inst.rs1) | (cpu.ex.s.r.rda == cpu.de.s.r.inst.rs2));

	bool pc_flush = cpu.ex.pc_r;

	if (load_hazard || pc_flush) {
		cpu.ex.s.w.v = 1;
		cpu.ex.s.tick(cpu);
	} else if (!false) {
		cpu.ex.s.tick(cpu);
	}

	if (pc_flush) {
		cpu.de.s.w.v = 1;
		cpu.de.s.tick(cpu);
	} else if (!load_hazard) {
		cpu.de.s.tick(cpu);
	}

	if (false) {
	} else if (!load_hazard) {
		cpu.fe.s.tick(cpu);
	}

	cpu.wb.s.tick(cpu);
	cpu.mem.s.tick(cpu);
}

} // namespace cpu