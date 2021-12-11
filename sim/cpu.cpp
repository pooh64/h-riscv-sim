#include "sim/cpu.h"
namespace cpu
{
void CPU::execute()
{
	while (!shutdown) {
		tick(*this);
	}
}

void CPU::trace(std::ostream &os)
{
	PL_HU::ExcType et;
	Instr inst;
	os << fe.s.r.pc << " ";

	if (de.s.r._dbg_trace) {
		inst.raw = 0;
		mmu.load(*this, de.s.r.pc, &inst.raw, et);
		os << inst << " ";
	} else {
		os << "flush ";
	}
	if (ex.s.r._dbg_trace) {
		inst.raw = 0;
		mmu.load(*this, ex.s.r.pc, &inst.raw, et);
		os << inst << " ";
	} else {
		os << "flush ";
	}
	if (mem.s.r._dbg_trace) {
		inst.raw = 0;
		mmu.load(*this, mem.s.r.pc, &inst.raw, et);
		os << inst << " ";
	} else {
		os << "flush ";
	}
	if (wb.s.r._dbg_trace) {
		os << "wback ";
	} else {
		os << "flushed ";
	}
}

} // namespace cpu