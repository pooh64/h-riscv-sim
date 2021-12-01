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
		u8 result_src;
		u8 mem_write;
		u8 jump;
		u8 branch;
		u8 alu_ctrl;
		u8 alu_src;
		//
		u32 rs1v;
		u32 rs2v;
		u32 imm_ext;
		u32 pc;
		u8 rs1a;
		u8 rs2a;
		u8 rda;
		u8 v;
	};
	TickReg<State> s;
};

struct PL_Memory final : Module {
	CPU_MODULE;
	struct State {
		u8 wb_we;
		u8 mem_we;
		u16 control;
		u32 mem_wd;
		u32 addr;
		u32 wb_a;
	};
	TickReg<State> s;
};

struct PL_Wback final : Module {
	CPU_MODULE;
	struct State {
		u8 wb_we;
		u8 wb_a;
		u32 wb_d;
	};
	TickReg<State> s;
};

struct PL_HU final : Module {
	CPU_MODULE;
};

struct CPU final : Module {
	CPU_MODULE;
	MMU mmu;
	CPU(u32 *mem_base, u32 mem_sz) : mmu(mem_base, mem_sz) {}

	struct {
		u8 pc_r;
		u8 pc_disp;
	} wires;

	PL_Fetch fe;
	PL_Decode de;
	PL_Execute ex;
	PL_Memory mem;
	PL_Wback wb;
};

inline void CPU::init() {}
inline void CPU::tick(CPU &cpu)
{
	ex.tick(cpu);
	fe.tick(cpu);
	de.tick(cpu);
	mem.tick(cpu);
	wb.tick(cpu);
}

/**********************************************************/
// Done
inline void PL_Fetch::init() {}
inline void PL_Fetch::tick(CPU &cpu)
{
	cpu.mmu.load(cpu.fe.s.r.pc, cpu.de.s.w.inst.raw);

	if (!cpu.wires.pc_r)
		cpu.fe.s.w.pc = cpu.fe.s.r.pc + 4;
	else
		cpu.fe.s.w.pc = cpu.ex.s.r.pc + cpu.wires.pc_disp;
}

/**********************************************************/
/* TODO: pass 30:7, cu tables */
struct CUFlags_Main {
	u8 reg_write : 1;
	u8 imm_src : 2;
	u8 alu_src : 2;
	u8 mem_write : 1;
	u8 result_src : 2;
	u8 branch : 1;
	u8 alu_op : 2;
	u8 jump : 1;
	u8 opcode_ok : 1;
	constexpr CUFlags_Main(u8 reg_write_, u8 imm_src_, u8 alu_src_, u8 mem_write_,
			       u8 result_src_, u8 branch_, u8 alu_op_, u8 jump_)
	    : reg_write(reg_write_), imm_src(imm_src_), alu_src(alu_src_), mem_write(mem_write_),
	      result_src(result_src_), branch(branch_), alu_op(alu_op_), jump(jump_), opcode_ok(1)
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
static u8 cu_alu_map[64] = {};

inline u32 CPUSignExtend(u32 raw_, u8 imm_src)
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
	} in, out;
	in.raw = raw_;
	out.raw = 0;
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
	cu_main_map[0b0000011] = CUFlags_Main(1, 0b00, 1, 0, 0b01, 0, 0b00, 0);
}
inline void PL_Decode::Regfile::tick(CPU &cpu)
{
	u8 a1 = cpu.de.s.r.inst.rs1;
	u8 a2 = cpu.de.s.r.inst.rs2;
	u8 a3 = cpu.wb.s.r.wb_a;
	u8 d3 = cpu.wb.s.r.wb_d;
	u8 we3 = cpu.wb.s.r.wb_we;

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
	cpu.ex.s.w.alu_ctrl = cu_alu_map[cfa_in.raw];
	cpu.ex.s.w.alu_src = cfm.alu_src;
	//
	cpu.ex.s.w.imm_ext = CPUSignExtend(inst.raw, cfm.imm_src);
	cpu.ex.s.w.pc = cpu.de.s.r.pc;
	cpu.ex.s.w.rs1a = inst.rs1;
	cpu.ex.s.w.rs2a = inst.rs2;
	cpu.ex.s.w.rda = inst.rd;
	cpu.ex.s.w.v = cpu.de.s.r.pc_r && cpu.wires.pc_r;
}

/**********************************************************/
inline void PL_Execute::init() {}
inline void PL_Execute::tick(CPU &cpu) {}

/**********************************************************/
inline void PL_Memory::init() {}
inline void PL_Memory::tick(CPU &cpu) {}

/**********************************************************/
inline void PL_Wback::init() {}
inline void PL_Wback::tick(CPU &cpu) {}

/**********************************************************/
inline void PL_HU::init() {}
inline void PL_HU::tick(CPU &cpu) {}

} // namespace cpu