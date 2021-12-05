#include "sim/cpu.h"

#include <iostream>

void CpuLoop(cpu::CPU &cpu)
{
	while (1) {
		cpu.tick(cpu);
		cpu.de.regfile.dump();
		int c;
		//scanf("%d", &c);
	}
}

int main()
{
	u32 mem[1024] = {};
	cpu::CPU cpu_sim;
	cpu_sim.mmu.setMem(mem, sizeof(mem) / sizeof(*mem));
	cpu_sim.fe.s.r.pc = 0x0;

	mem[0] = 0x6002103;
	mem[1] = 0x6002103;
	mem[2] = 0x6002103;
	mem[3] = 0x6002103;
	mem[4] = 0x6002103;
	mem[5] = 0x6002103;
	mem[6] = 0x6002103;
	mem[7] = 0x6002103;
	mem[8] = 0x6002103;
	mem[9] = 0x6002103;

	mem[96 / 4] = 123;

	CpuLoop(cpu_sim);
	return 0;
}