#include "sim/cpu.h"

#include <iostream>

void DumpExc(cpu::CPU &cpu)
{
	cpu.de.regfile.dump();
	printf("Exception: %d at %d\n", static_cast<int>(cpu.hu.exc_type), cpu.hu.exc_pc);
}

void test_1()
{
	cpu::CPU cpu_sim{};
	u32 mem[64] = {};
	cpu_sim.mmu.setMem(mem, sizeof(mem) / sizeof(*mem));
	mem[0] = 0x02002503; // lw a0, 32(zero)
	mem[1] = 0x02402583; // lw a1, 36(zero)
	mem[2] = 0x40a58633; // sub a2, a1, a0
	mem[3] = 0x02c02423; // sw a2, 40(zero)
	mem[4] = 0x00100073; // ebreak
	mem[32 / 4] = 0x21323424;
	mem[36 / 4] = 0xdeadbabe;

	cpu_sim.execute();
	assert(cpu_sim.hu.exc_pc == 16);
	assert(cpu_sim.de.regfile.gpr[12] == 0xdeadbabe - 0x21323424);
	assert(mem[40 / 4] == 0xdeadbabe - 0x21323424);
}

void test_2()
{
	cpu::CPU cpu_sim{};
	u32 mem[64] = {};
	cpu_sim.mmu.setMem(mem, sizeof(mem) / sizeof(*mem));
	mem[0] = 0x0080056f; // jal a0, lab1
	mem[1] = 0x07b00593; // li a1, 123
	mem[2] = 0x14100613; // lab1: li a2, 321
	mem[3] = 0x00100073; // ebreak

	cpu_sim.execute();
	assert(cpu_sim.hu.exc_pc == 12);
	assert(cpu_sim.de.regfile.gpr[10] == 4);
	assert(cpu_sim.de.regfile.gpr[11] == 0);
	assert(cpu_sim.de.regfile.gpr[12] == 321);
}

void test()
{
	test_1();
	test_2();
}

int main()
{
	test();
	return 0;
}