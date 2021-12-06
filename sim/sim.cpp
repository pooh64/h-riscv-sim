#include "sim/cpu.h"

#include <iostream>

void CpuLoop(cpu::CPU &cpu)
{
	while (1) {
		cpu.tick(cpu);
		if (!cpu.hu.flush_cnt) {
			cpu.de.regfile.dump();
			printf("Exception: %x at %x\n", static_cast<int>(cpu.hu.exc_type),
			       cpu.hu.exc_pc);
			return;
		}
	}
}

int main()
{
	u32 mem[1024] = {};
	cpu::CPU cpu_sim;
	cpu_sim.mmu.setMem(mem, sizeof(mem) / sizeof(*mem));
	cpu_sim.fe.s.r.pc = 0x0;
	cpu_sim.de.s.r.inst.raw = 0x00000033; // do nothing

	mem[0] = 0x6002103;
	mem[1] = 0b1110011; // ebreak

	mem[96 / 4] = 123;

	CpuLoop(cpu_sim);
	return 0;
}