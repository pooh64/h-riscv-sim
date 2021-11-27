#include "sim/cpu.h"

#include <iostream>

int main()
{
	u32 mem[1024];

	cpu::CPU cpu_sim(mem, sizeof(mem) / sizeof(*mem));
	return 0;
}