#pragma once

#include "sim/cu_maps.h"
#include <cstring>
#include <string>

namespace cpu
{
struct CPU;
struct Module {
	virtual inline void init() = 0;
	virtual inline void tick(CPU &) = 0;
};
#define CPU_MODULE                                                                                                     \
	inline void init();                                                                                            \
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

struct PL_HU final : Module {
	CPU_MODULE;
	enum class FWD : u8 {
		NO = 0b00,
		MEM = 0b01,
		WB = 0b10,
	};
	inline FWD getRegFWD(CPU &cpu, u8 rsa);
	enum class ExcType : u32 {
		BAD_OPCODE,
		UNALIGNED_ADDR,
		MMU_MISS,
		INT,
	} exc_cause;
	enum class ExcStage : i8 {
		NO = -1,
		FE = 0,
		DE,
		EX,
		MEM,
		WB,
	} exc_stage = ExcStage::NO;
	u32 exc_pc;
	inline void RaiseExcept(ExcStage stage, ExcType type, u32 pc)
	{
		if (stage <= exc_stage)
			return;
		exc_stage = stage;
		exc_cause = type;
		exc_pc = pc;
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

	inline bool load(CPU &cpu, u32 a, u32 *d, PL_HU::ExcType &et);
	inline bool store(CPU &cpu, u32 a, u32 d, PL_HU::ExcType &et);
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
		bool v;
		bool _dbg_trace;
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
		bool reg_write;
		bool mem_write;
		CUMemOp mem_op;
		bool mem_sgne;
		CUResSrc result_src;
		bool jump;
		bool jreg;
		bool branch;
		bool intpt;
		CUALUCtrl alu_ctrl;
		CUCMPCtrl cmp_ctrl;
		CUALUSrc1 alu_src1;
		CUALUSrc2 alu_src2;
		//
		u32 pc;
		u32 pc_next;
		u32 rs1v;
		u32 rs2v;
		u32 imm_ext;
		u8 rs1a;
		u8 rs2a;
		u8 rda;
		bool _dbg_trace;
	};
	TickReg<State> s;
	u32 jbase;
	bool pc_r{0};
};

struct PL_Memory final : Module {
	CPU_MODULE;
	struct State {
		bool reg_write;
		bool mem_write;
		CUResSrc result_src;
		CUMemOp mem_op;
		bool mem_sgne;
		//
		u8 reg_addr;
		u32 pc_next;
		u32 pc;
		u32 mem_wdata;
		u32 alu_res;
		bool _dbg_trace;
	};
	TickReg<State> s;

	struct UOp {
		u32 a;
		u32 d;
		bool active{false};
	} delayed_write;
};

struct PL_Wback final : Module {
	CPU_MODULE;
	struct State {
		bool reg_write;
		u8 reg_addr;
		u32 reg_wdata;
		bool _dbg_trace;
	};
	TickReg<State> s;
};

struct CPU final : Module {
	CPU_MODULE;
	MMU mmu;
	CPU()
	{
		fe.s.r.pc = 0x0;
		de.s.r.inst.raw = 0x00000033; // do nothing
	}
	void execute();
	u32 tvec = 0;
	u32 time = 0;
	bool shutdown = false;
	void trace(std::ostream &os);

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
	time++;
}

inline bool MMU::load(CPU &cpu, u32 a, u32 *d, PL_HU::ExcType &et)
{
	u32 *addr = (u32 *)((u8 *)base + a);
	if (a % 4) {
		et = PL_HU::ExcType::UNALIGNED_ADDR;
		return false;
	}
	if (!(addr < end)) {
		et = PL_HU::ExcType::MMU_MISS;
		return false;
	}
	*d = *addr;
	return true;
}

inline bool MMU::store(CPU &cpu, u32 a, u32 d, PL_HU::ExcType &et)
{
	u32 *addr = (u32 *)((u8 *)base + a);
	if (a % 4) {
		et = PL_HU::ExcType::UNALIGNED_ADDR;
		return false;
	}
	if (a == 0)
		cpu.shutdown = true;
	if (!(addr < end)) {
		et = PL_HU::ExcType::MMU_MISS;
		return false;
	}
	*addr = d;
	return true;
}

/**********************************************************/
inline void PL_Fetch::init() {}
inline void PL_Fetch::tick(CPU &cpu)
{
	PL_HU::ExcType et;
	if (!cpu.mmu.load(cpu, cpu.fe.s.r.pc, &cpu.de.s.w.inst.raw, et)) {
		cpu.hu.RaiseExcept(PL_HU::ExcStage::FE, et, cpu.fe.s.r.pc);
	}

	u32 pc_next;
	if (!cpu.ex.pc_r)
		pc_next = cpu.fe.s.r.pc + 4;
	else
		pc_next = cpu.ex.jbase + cpu.ex.s.r.imm_ext;

	cpu.fe.s.w.pc = pc_next;

	cpu.de.s.w.v = 0; // hu
	cpu.de.s.w.pc = cpu.fe.s.r.pc;
	cpu.de.s.w.pc_next = pc_next;
}

/**********************************************************/
inline constexpr u32 CPUSignExtend(u32 raw_, CUIType imm_src)
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
		struct {
			u32 _pad0 : 12;
			u32 imm0 : 19;
			u32 sgn : 1;
		} __attribute__((packed)) fu;
		struct {
			u32 imm0 : 20;
			u32 se : 12;
		} __attribute__((packed)) du;
	} in{.raw = raw_}, out{.raw = 0};
	u8 sgn = (in.raw >> 31) & 1;

	switch (imm_src) {
	case CUIType::R:
		break;
	case CUIType::I:
		out.di.imm0 = in.fi.imm0;
		if (sgn)
			out.di.se--;
		break;
	case CUIType::S:
		out.ds.imm0 = in.fs.imm0;
		out.ds.imm1 = in.fs.imm1;
		if (sgn)
			out.ds.se--;
		break;
	case CUIType::B:
		out.db.imm0 = in.fb.imm0;
		out.db.imm1 = in.fb.imm1;
		out.db.imm2 = in.fb.imm2;
		if (sgn)
			out.db.se--;
		out.raw <<= 1; // branch
		break;
	case CUIType::J:
		out.dj.imm0 = in.fj.imm0;
		out.dj.imm1 = in.fj.imm1;
		out.dj.imm2 = in.fj.imm2;
		if (sgn)
			out.dj.se--;
		out.raw <<= 1; // jmp
		break;
	case CUIType::U:
		out.du.imm0 = in.fu.imm0;
		if (sgn)
			out.dj.se--;
		out.raw <<= 12; // auipc
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
	u32 d3 = cpu.wb.s.r.reg_wdata;
	u8 we3 = cpu.wb.s.r.reg_write;

	if (we3) {
		gpr[a3] = d3;
		gpr[0] = 0;
	}
	cpu.ex.s.w.rs1v = gpr[a1];
	cpu.ex.s.w.rs2v = gpr[a2];
}

inline void PL_Decode::init() {}
inline void PL_Decode::tick(CPU &cpu)
{
	auto inst = cpu.de.s.r.inst;

	CUFlags_Main cf = GetCUFlags(inst);

	cpu.ex.s.w.reg_write = cf.reg_write && !cpu.de.s.r.v;
	cpu.ex.s.w.result_src = cpu.de.s.r.v ? CUResSrc::ALU : cf.result_src;
	cpu.ex.s.w.mem_write = cf.mem_write && !cpu.de.s.r.v;
	cpu.ex.s.w.mem_op = cf.mem_op;
	cpu.ex.s.w.mem_sgne = cf.mem_sgne;
	cpu.ex.s.w.jump = cf.jump && !cpu.de.s.r.v;
	cpu.ex.s.w.jreg = cf.jreg;
	cpu.ex.s.w.branch = cf.branch && !cpu.de.s.r.v;
	cpu.ex.s.w.intpt = cf.intpt && !cpu.de.s.r.v;
	cpu.ex.s.w.alu_ctrl = cf.alu_control;
	cpu.ex.s.w.cmp_ctrl = cf.cmp_control;
	cpu.ex.s.w.alu_src1 = cf.alu_src1;
	cpu.ex.s.w.alu_src2 = cf.alu_src2;
	if (!cf.opcode_ok && !cpu.de.s.r.v)
		cpu.hu.RaiseExcept(PL_HU::ExcStage::DE, PL_HU::ExcType::BAD_OPCODE, cpu.de.s.r.pc);

	cpu.ex.s.w.imm_ext = CPUSignExtend(inst.raw, cf.itype);
	cpu.ex.s.w.pc = cpu.de.s.r.pc;
	cpu.ex.s.w.pc_next = cpu.de.s.r.pc_next;
	cpu.ex.s.w.rs1a = inst.rs1;
	cpu.ex.s.w.rs2a = inst.rs2;
	cpu.ex.s.w.rda = inst.rd;

	cpu.de.regfile.tick(cpu);
}

/**********************************************************/
inline constexpr u32 ALUOperator(CUALUCtrl alu_ctrl, u32 a, u32 b)
{
	switch (alu_ctrl) {
	case CUALUCtrl::ADD:
		return a + b;
	case CUALUCtrl::SUB:
		return a - b;
	case CUALUCtrl::SLL:
		return a << (b & 31);
	case CUALUCtrl::SLT:
		return !!((i32)a < (i32)b);
	case CUALUCtrl::SLTU:
		return !!(a < b);
	case CUALUCtrl::XOR:
		return a ^ b;
	case CUALUCtrl::SRL:
		return a >> (b & 31);
	case CUALUCtrl::SRA:
		return (i32)a >> (b & 31);
	case CUALUCtrl::OR:
		return a | b;
	case CUALUCtrl::AND:
		return a & b;
	case CUALUCtrl::ARG2:
		return b;
	default:
		assert(0);
	};
}

inline constexpr bool CMPOperator(CUCMPCtrl cmp_ctrl, u32 a, u32 b)
{
	switch (cmp_ctrl) {
	case CUCMPCtrl::EQ:
		return a == b;
	case CUCMPCtrl::NE:
		return a != b;
	case CUCMPCtrl::LT:
		return (i32)a < (i32)b;
	case CUCMPCtrl::GE:
		return (i32)a >= (i32)b;
	case CUCMPCtrl::LTU:
		return a < b;
	case CUCMPCtrl::GEU:
		return a >= b;
	default:
		assert(0);
	};
}

inline void PL_Execute::init() {}
inline void PL_Execute::tick(CPU &cpu)
{
	cpu.mem.s.w.reg_write = cpu.ex.s.r.reg_write;
	cpu.mem.s.w.mem_write = cpu.ex.s.r.mem_write;
	cpu.mem.s.w.mem_op = cpu.ex.s.r.mem_op;
	cpu.mem.s.w.mem_sgne = cpu.ex.s.r.mem_sgne;

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
	u32 sv1 = fwd_select(cpu.hu.getRegFWD(cpu, cpu.ex.s.r.rs1a), cpu.ex.s.r.rs1v);
	u32 sv2 = fwd_select(cpu.hu.getRegFWD(cpu, cpu.ex.s.r.rs2a), cpu.ex.s.r.rs2v);
	jbase = cpu.ex.s.r.jreg ? (sv1 & ~(u32)1) : cpu.ex.s.r.pc;
	cpu.mem.s.w.mem_wdata = sv2;

	if (cpu.ex.s.r.alu_src1 == CUALUSrc1::PC)
		sv1 = cpu.ex.s.r.pc;
	if (cpu.ex.s.r.alu_src2 == CUALUSrc2::I)
		sv2 = cpu.ex.s.r.imm_ext;

	u32 alu_res = ALUOperator(cpu.ex.s.r.alu_ctrl, sv1, sv2);
	cpu.mem.s.w.alu_res = alu_res;

	bool cmp_res = CMPOperator(cpu.ex.s.r.cmp_ctrl, sv1, sv2);

	cpu.ex.pc_r = cpu.ex.s.r.jump || (cpu.ex.s.r.branch && cmp_res);
	cpu.mem.s.w.pc_next = cpu.ex.s.r.pc_next;
	cpu.mem.s.w.pc = cpu.ex.s.r.pc;

	if (cpu.ex.s.r.intpt)
		cpu.hu.RaiseExcept(PL_HU::ExcStage::EX, PL_HU::ExcType::INT, cpu.ex.s.r.pc);
}

/**********************************************************/
inline void PL_Memory::init() {}
inline void PL_Memory::tick(CPU &cpu)
{
	u32 mmu_rd = 0;
	PL_HU::ExcType et;
	if (cpu.mem.s.r.result_src == CUResSrc::MEM) {
		if (!cpu.mmu.load(cpu, cpu.mem.s.r.alu_res & (~(u32)3), &mmu_rd, et))
			cpu.hu.RaiseExcept(PL_HU::ExcStage::MEM, et, cpu.mem.s.r.pc);
		u8 sh = 8 * (cpu.mem.s.r.alu_res & ((u32)3));
		u8 align;
		mmu_rd >>= sh;
		switch (cpu.mem.s.r.mem_op) {
		case CUMemOp::B:
			mmu_rd = cpu.mem.s.r.mem_sgne ? (i32)(i8)mmu_rd : (u8)mmu_rd;
			align = 1;
			break;
		case CUMemOp::H:
			mmu_rd = cpu.mem.s.r.mem_sgne ? (i32)(i16)mmu_rd : (u16)mmu_rd;
			align = 2;
			break;
		case CUMemOp::W:
			align = 4;
			break;
		default:
			assert(0);
		}
		if (sh % align)
			cpu.hu.RaiseExcept(PL_HU::ExcStage::MEM, PL_HU::ExcType::UNALIGNED_ADDR, cpu.mem.s.r.pc);
	}
	if (delayed_write.active) {
		if (!cpu.mmu.store(cpu, delayed_write.a, delayed_write.d, et))
			cpu.hu.RaiseExcept(PL_HU::ExcStage::MEM, et, cpu.mem.s.r.pc);
		delayed_write.active = false;
	} else if (cpu.mem.s.r.mem_write) {
		u32 wmask = 0;
		switch (cpu.mem.s.r.mem_op) {
		case CUMemOp::B:
			wmask = 0xff;
			delayed_write.active = true;
			break;
		case CUMemOp::H:
			wmask = 0xffff;
			delayed_write.active = true;
			break;
		case CUMemOp::W:
			if (!cpu.mmu.store(cpu, cpu.mem.s.r.alu_res, cpu.mem.s.r.mem_wdata, et))
				cpu.hu.RaiseExcept(PL_HU::ExcStage::MEM, et, cpu.mem.s.r.pc);
			break;
		default:
			assert(0);
		}
		if (delayed_write.active) {
			u8 sh = 8 * (cpu.mem.s.r.alu_res & ((u32)3));
			delayed_write.a = cpu.mem.s.r.alu_res & (~(u32)3);
			if (!cpu.mmu.load(cpu, delayed_write.a, &delayed_write.d, et)) {
				cpu.hu.RaiseExcept(PL_HU::ExcStage::MEM, et, cpu.mem.s.r.pc);
				delayed_write.active = false;
			}
			delayed_write.d &= ~(wmask << sh);
			delayed_write.d |= ((wmask & cpu.mem.s.r.mem_wdata) << sh);
		}
		// check sub-store alignment
	}

	switch (cpu.mem.s.r.result_src) {
	case CUResSrc::ALU:
		cpu.wb.s.w.reg_wdata = cpu.mem.s.r.alu_res;
		break;
	case CUResSrc::MEM:
		cpu.wb.s.w.reg_wdata = mmu_rd;
		break;
	case CUResSrc::PC:
		cpu.wb.s.w.reg_wdata = cpu.mem.s.r.pc_next;
		break;
	default:
		assert(0);
	}

	cpu.wb.s.w.reg_write = cpu.mem.s.r.reg_write;
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
	bool load_hazard = (cpu.ex.s.r.result_src == CUResSrc::MEM) &&
			   ((cpu.ex.s.r.rda == cpu.de.s.r.inst.rs1) || (cpu.ex.s.r.rda == cpu.de.s.r.inst.rs2));

	bool pc_flush = cpu.ex.pc_r;
	bool mem_stall = cpu.mem.delayed_write.active;

	if (mem_stall) {
		pc_flush = false;
		load_hazard = false;
		exc_stage = ExcStage::NO;
	}

	if ((pc_flush || load_hazard) && exc_stage <= ExcStage::DE) {
		exc_stage = ExcStage::NO;
	}

	if (exc_stage >= ExcStage::MEM || mem_stall) {
		cpu.wb.s.w.reg_write = 0;
		cpu.wb.s.w._dbg_trace = 0;
		cpu.wb.s.tick(cpu);
	} else if (!false) {
		cpu.wb.s.w._dbg_trace = cpu.mem.s.r._dbg_trace;
		cpu.wb.s.tick(cpu);
	}

	if (exc_stage >= ExcStage::EX) {
		cpu.mem.s.w.reg_write = 0;
		cpu.mem.s.w.mem_write = 0;
		cpu.mem.s.w.result_src = CUResSrc::ALU;
		cpu.mem.s.w._dbg_trace = 0;
		cpu.mem.s.tick(cpu);
	} else if (!mem_stall) {
		cpu.mem.s.w._dbg_trace = cpu.ex.s.r._dbg_trace;
		cpu.mem.s.tick(cpu);
	}

	if (exc_stage >= ExcStage::DE || load_hazard || pc_flush) {
		cpu.ex.s.w.reg_write = 0;
		cpu.ex.s.w.mem_write = 0;
		cpu.ex.s.w.result_src = CUResSrc::ALU;
		cpu.ex.s.w.branch = 0;
		cpu.ex.s.w.jump = 0;
		cpu.ex.s.w.intpt = 0;
		cpu.ex.s.w._dbg_trace = 0;
		cpu.ex.s.tick(cpu);
	} else if (!mem_stall) {
		cpu.ex.s.w._dbg_trace = cpu.de.s.r._dbg_trace;
		cpu.ex.s.tick(cpu);
	}

	if (exc_stage >= ExcStage::FE || pc_flush) {
		cpu.de.s.w.v = 1;
		cpu.de.s.w._dbg_trace = 0;
		cpu.de.s.tick(cpu);
	} else if (!(load_hazard || mem_stall)) {
		cpu.de.s.w._dbg_trace = 1;
		cpu.de.s.tick(cpu);
	}

	if (exc_stage > ExcStage::NO) {
		cpu.fe.s.w.pc = cpu.tvec;
		cpu.fe.s.tick(cpu);
	} else if (!(load_hazard || mem_stall)) {
		cpu.fe.s.tick(cpu);
	}
	exc_stage = ExcStage::NO;
}

} // namespace cpu