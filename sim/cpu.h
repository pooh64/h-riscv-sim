#include <cassert>
#include <cstdint>
#include <utility>

typedef uint32_t u32;
typedef int32_t i32;
typedef uint16_t u16;
typedef uint8_t u8;

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
	T r, w;
	inline void init(){};
	inline void tick(CPU &)
	{
		std::swap(r, w);
	}
};

struct MMU {
	u32 *const base;
	u32 *const end;

	MMU(u32 *base_, u32 sz_) : base(base_), end(base_ + sz_ / sizeof(u32))
	{
		assert(sz_ % sizeof(u32) == 0);
	}

	inline void load(u32 a, u32 &d)
	{
		u32 *addr = base + a;
		assert(addr < end);
		d = *addr;
	}

	inline void store(u32 a, u32 d)
	{
		u32 *addr = base + a;
		assert(addr < end);
		d = *addr;
	}
};

union Instr {
	u32 raw;
	struct {
		u8 op : 7;
		u8 rd : 5;
		u8 funct3 : 3;
		u8 rs1 : 5;
		u8 rs2 : 5;
		u8 funct7 : 7;
	};
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
		u8 pc_r;
	};
	TickReg<State> s;
	struct Regfile final : Module {
		CPU_MODULE;
		u32 gpr[32] = {};
	} regfile;
};

struct PL_Execute final : Module {
	CPU_MODULE;
	struct State {
		u8 reg_write;
		u8 mem_write;
		u8 result_src;
		u8 jump;
		u8 branch;
		u8 alu_ctrl;
		u8 alu_src;
		//
		u32 pc;
		u32 rs1v;
		u32 rs2v;
		u32 imm_ext;
		u8 rs1a;
		u8 rs2a;
		u8 rda;
		u8 v;
	};
	TickReg<State> s;
	u8 pc_r;
};

struct PL_Memory final : Module {
	CPU_MODULE;
	struct State {
		u8 reg_write;
		u8 mem_write;
		u8 result_src;
		//
		u32 pc_4;
		u32 mem_wdata;
		u32 alu_res;
		u32 rda;
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
	inline void getFWD(FWD &rs1, FWD &rs2);
};

struct CPU final : Module {
	CPU_MODULE;
	MMU mmu;
	CPU(u32 *mem_base, u32 mem_sz) : mmu(mem_base, mem_sz) {}

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
}

/**********************************************************/
// Done
inline void PL_Fetch::init() {}
inline void PL_Fetch::tick(CPU &cpu)
{
	cpu.mmu.load(cpu.fe.s.r.pc, cpu.de.s.w.inst.raw);

	if (!cpu.ex.pc_r)
		cpu.fe.s.w.pc = cpu.fe.s.r.pc + 4;
	else
		cpu.fe.s.w.pc = cpu.ex.s.r.pc + cpu.ex.s.r.imm_ext;
}

/**********************************************************/
/* TODO: pass 30:7, cu tables */

enum class ALUControl : u8 {
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

struct CUFlags_Main {
	u8 reg_write : 1;
	u8 imm_src : 2;
	u8 alu_src : 2;
	u8 mem_write : 1;
	u8 result_src : 2;
	u8 branch : 1;
	u8 alu_op : 2;
	u8 jump : 1;
	u8 pc_src : 1;
	u8 opcode_ok : 1;
	constexpr CUFlags_Main(u8 reg_write_, u8 imm_src_, u8 alu_src_, u8 mem_write_,
			       u8 result_src_, u8 branch_, u8 alu_op_, u8 jump_, u8 pc_src_)
	    : reg_write(reg_write_), imm_src(imm_src_), alu_src(alu_src_), mem_write(mem_write_),
	      result_src(result_src_), branch(branch_), alu_op(alu_op_), jump(jump_),
	      pc_src(pc_src_), opcode_ok(1)
	{
	}
	CUFlags_Main() : opcode_ok(0){};
};
static CUFlags_Main cu_main_map[128] = {};

union CUALU_in {
	struct {
		u8 alu_op : 1;
		u8 funct3 : 3;
		u8 funct7_5 : 1;
		u8 op_5 : 1;
	};
	u8 raw;
};
ALUControl cu_alu_map[64] = {};

inline constexpr u32 CPUSignExtend(u32 raw_, u8 imm_src)
{
	union imm_layout {
		u32 raw;
		struct {
			u32 _pad0 : 20;
			u32 imm0 : 12;
		} fi;
		struct {
			u32 imm0 : 12;
			u32 se : 20;
		} di;
		struct {
			u32 _pad0 : 7;
			u32 imm0 : 5;
			u32 _pad1 : 13;
			u32 imm1 : 7;
		} fs;
		struct {
			u32 imm0 : 5;
			u32 imm1 : 7;
			u32 se : 20;
		} ds;
		struct {
			u32 _pad0 : 7;
			u32 imm2 : 1;
			u32 imm0 : 4;
			u32 _pad2 : 13;
			u32 imm1 : 6;
			u32 sgn : 1;
		} fb;
		struct {
			u32 imm0 : 4;
			u32 imm1 : 6;
			u32 imm2 : 1;
			u32 se : 21;
		} db;
		struct {
			u32 _pad0 : 12;
			u32 imm2 : 8;
			u32 imm1 : 1;
			u32 imm0 : 10;
			u32 sgn : 1;
		} fj;
		struct {
			u32 imm0 : 10;
			u32 imm1 : 1;
			u32 imm2 : 8;
			u32 se : 13;
		} dj;
	} in{.raw = raw_}, out{.raw = 0};
	u8 sgn = (in.raw >> 31) | 1;

	switch (imm_src) {
	case 0b00:
		out.di.imm0 = in.fi.imm0;
		if (sgn)
			out.di.se--;
		break;
	case 0b01:
		out.ds.imm0 = in.fs.imm0;
		out.ds.imm1 = in.fs.imm1;
		if (sgn)
			out.ds.se--;
		break;
	case 0b10:
		out.db.imm0 = in.fb.imm0;
		out.db.imm1 = in.fb.imm1;
		out.db.imm2 = in.fb.imm2;
		if (sgn)
			out.db.se--;
		break;
	case 0b11:
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

inline void PL_Decode::Regfile::init()
{
	cu_main_map[0b0000011] = CUFlags_Main(1, 0b00, 1, 0, 0b01, 0, 0b00, 0, 0);
}
inline void PL_Decode::Regfile::tick(CPU &cpu)
{
	u8 a1 = cpu.de.s.r.inst.rs1;
	u8 a2 = cpu.de.s.r.inst.rs2;
	u8 a3 = cpu.wb.s.r.reg_addr;
	u8 d3 = cpu.wb.s.r.reg_wdata;
	u8 we3 = cpu.wb.s.r.reg_write;

	cpu.ex.s.w.rs1v = gpr[a1];
	cpu.ex.s.w.rs2v = gpr[a2];
	if (we3)
		gpr[a3] = d3;
}

inline void PL_Decode::init() {}
inline void PL_Decode::tick(CPU &cpu)
{
	auto inst = cpu.de.s.r.inst;

	CUFlags_Main cfm = cu_main_map[inst.op];
	assert(cfm.opcode_ok);

	CUALU_in cfa_in;
	cfa_in.alu_op = cfm.alu_op;
	cfa_in.funct3 = inst.funct3;
	cfa_in.funct7_5 = (u8)(inst.funct7 >> 5); // start from zero?
	cfa_in.op_5 = inst.op;

	cpu.ex.s.w.reg_write = cfm.reg_write;
	cpu.ex.s.w.result_src = cfm.result_src;
	cpu.ex.s.w.mem_write = cfm.mem_write;
	cpu.ex.s.w.jump = cfm.jump;
	cpu.ex.s.w.branch = cfm.branch;
	cpu.ex.s.w.alu_ctrl = static_cast<u8>(cu_alu_map[cfa_in.raw]);
	cpu.ex.s.w.alu_src = cfm.alu_src;
	//
	cpu.ex.s.w.imm_ext = CPUSignExtend(inst.raw, cfm.imm_src);
	cpu.ex.s.w.pc = cpu.de.s.r.pc;
	cpu.ex.s.w.rs1a = inst.rs1;
	cpu.ex.s.w.rs2a = inst.rs2;
	cpu.ex.s.w.rda = inst.rd;
	cpu.ex.s.w.v = cpu.de.s.r.pc_r && cpu.ex.pc_r;
}

/**********************************************************/
/* TODO: finish ALU, hu fwd */
inline constexpr u32 ALUOperator(u8 alu_ctrl, u32 a, u32 b)
{
	switch (static_cast<ALUControl>(alu_ctrl)) {
	case ALUControl::ADD:
		return a + b;
	case ALUControl::SUB:
		return a - b;
	case ALUControl::AND:
		return a & b;
	case ALUControl::OR:
		return a | b;
	case ALUControl::SLT:
		return !!((i32)a < (i32)b);
	default:
		assert(0);
	};
}

inline void PL_Execute::init() {}
inline void PL_Execute::tick(CPU &cpu)
{
	cpu.mem.s.w.reg_write = cpu.ex.s.r.v && cpu.ex.s.r.reg_write;
	cpu.mem.s.w.mem_write = cpu.ex.s.r.v && cpu.ex.s.r.mem_write;

	cpu.mem.s.w.result_src = cpu.ex.s.r.result_src;
	cpu.mem.s.w.rda = cpu.ex.s.r.rda;

	PL_HU::FWD hu_fwd1, hu_fwd2;
	cpu.hu.getFWD(hu_fwd1, hu_fwd2);

	u32 rs1v, rs2v;
	switch (hu_fwd1) {
	case PL_HU::FWD::NO:
		rs1v = cpu.ex.s.r.rs1v;
		break;
	case PL_HU::FWD::MEM:
		rs1v = cpu.mem.s.r.alu_res;
		break;
	case PL_HU::FWD::WB:
		rs1v = cpu.wb.s.r.reg_addr;
		break;
	}
	switch (hu_fwd2) {
	case PL_HU::FWD::NO:
		rs2v = cpu.ex.s.r.rs2v;
		break;
	case PL_HU::FWD::MEM:
		rs2v = cpu.mem.s.r.alu_res;
		break;
	case PL_HU::FWD::WB:
		rs2v = cpu.wb.s.r.reg_addr;
		break;
	}

	if (cpu.ex.s.r.alu_src)
		rs2v = cpu.ex.s.r.imm_ext;

	u32 alu_res = ALUOperator(cpu.ex.s.r.alu_ctrl, rs1v, rs2v);
	cpu.mem.s.w.alu_res = alu_res;

	cpu.ex.pc_r = cpu.ex.s.r.jump || cpu.ex.s.r.branch && (alu_res == 0);
	cpu.mem.s.w.mem_wdata = rs2v;
	cpu.mem.s.w.pc_4 = cpu.ex.s.r.pc + 4;
}

/**********************************************************/
inline void PL_Memory::init() {}
inline void PL_Memory::tick(CPU &cpu) {}

/**********************************************************/
inline void PL_Wback::init() {}
inline void PL_Wback::tick(CPU &cpu) {}

/**********************************************************/
inline void PL_HU::getFWD(FWD &rs1, FWD &rs2)
{
	// TODO: implement
	rs1 = FWD::NO;
	rs2 = FWD::NO;
}

inline void PL_HU::init() {}
inline void PL_HU::tick(CPU &cpu)
{
	// TODO: implement
	// tick all pipeline regs
}

} // namespace cpu