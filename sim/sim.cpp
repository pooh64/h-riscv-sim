#include "sim/cpu.h"

#include <iostream>

void CpuLoop(cpu::CPU &cpu)
{
	cpu.tick(cpu);
}

int main()
{
	u32 mem[1024];

	cpu::CPU cpu_sim(mem, sizeof(mem) / sizeof(*mem));
	CpuLoop(cpu_sim);
	return 0;
}