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

union Instr {
	u32 raw;
	struct {
		u32 control : 7;
		u32 rd : 5;
		u32 rs1 : 5;
		u32 rs2 : 5;
	};
};

struct PL_FetchReg final : Module {
	CPU_MODULE;
	TickReg<u32> reg;
};
struct PL_DecodeReg final : Module {
	CPU_MODULE;
	struct DecodeState {
		Instr inst;
		u32 pc;
		u8 pc_r;
	};
	TickReg<DecodeState> reg;
};
struct PL_ExecuteReg final : Module {
	CPU_MODULE;
	struct ExecuteState {
		u32 rs1;
		u32 rs2;
		u32 imm;
		u32 pc;
		u16 control;
		u8 v;
		u8 se; // 32?
	};
	TickReg<ExecuteState> reg;
};
struct PL_MemoryReg final : Module {
	CPU_MODULE;
	struct MemoryState {
		u8 wb_we;
		u8 mem_we;
		u16 control;
		u32 mem_wd;
		u32 addr;
		u32 wb_a;
	};
	TickReg<MemoryState> reg;
};
struct PL_WBackReg final : Module {
	CPU_MODULE;
	struct WBackState {
		u8 wb_we;
		u8 wb_a;
		u32 wb_d;
	};
	TickReg<WBackState> reg;
};

struct IMEM final : Module {
	CPU_MODULE;
};

struct PL_PCUpdate final : Module {
	CPU_MODULE;
};

struct MMU {
	u32 *const base;
	u32 *const end;

	MMU(u32 *base_, u32 sz_) : base(base_), end(base_ + sz_ / sizeof(u32))
	{
		assert(sz_ % sizeof(u32) == 0);
	}

	inline void read(u32 a, u32 &d)
	{
		u32 *addr = base + a;
		assert(addr < end);
		d = *addr;
	}

	inline void write(u32 a, u32 d)
	{
		u32 *addr = base + a;
		assert(addr < end);
		d = *addr;
	}
};

struct CPU final : Module {
	CPU_MODULE;
	MMU mmu;
	CPU(u32 *mem_base, u32 mem_sz) : mmu(mem_base, mem_sz) {}

	PL_FetchReg r_fetch;
	PL_PCUpdate m_fetch_pcupd;
	IMEM imem;
	struct {
		u8 pc_r;
		u8 pc_disp;
	} branch_wires;

	PL_DecodeReg r_decode;
	PL_ExecuteReg r_execute;
	PL_MemoryReg r_memory;
	PL_WBackReg r_wback;
};

inline void IMEM::init() {}
inline void IMEM::tick(CPU &cpu)
{
	cpu.mmu.read(cpu.r_fetch.reg.r, cpu.r_decode.reg.w.inst.raw);
}

inline void PL_PCUpdate::init() {}
inline void PL_PCUpdate::tick(CPU &cpu)
{
	if (!cpu.branch_wires.pc_r)
		cpu.r_fetch.reg.w = cpu.r_fetch.reg.r + 4;
	else
		cpu.r_fetch.reg.w = cpu.r_execute.reg.r.pc + cpu.branch_wires.pc_disp;
}

inline void PL_FetchReg::init() {}
inline void PL_FetchReg::tick(CPU &cpu)
{
	reg.tick(cpu);
}
inline void PL_DecodeReg::init() {}
inline void PL_DecodeReg::tick(CPU &cpu)
{
	reg.tick(cpu);
}
inline void PL_ExecuteReg::init() {}
inline void PL_ExecuteReg::tick(CPU &cpu)
{
	reg.tick(cpu);
}
inline void PL_MemoryReg::init() {}
inline void PL_MemoryReg::tick(CPU &cpu)
{
	reg.tick(cpu);
}
inline void PL_WBackReg::init() {}
inline void PL_WBackReg::tick(CPU &cpu)
{
	reg.tick(cpu);
}

inline void CPU::init() {}
inline void CPU::tick(CPU &cpu)
{
	r_fetch.tick(cpu);
	r_decode.tick(cpu);
	r_execute.tick(cpu);
	r_memory.tick(cpu);
	r_wback.tick(cpu);
}

} // namespace cpu