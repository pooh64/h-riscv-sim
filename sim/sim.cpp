#include "sim/cpu.h"

#include <iostream>
#include <memory>

void DumpExc(cpu::CPU &cpu)
{
	cpu.de.regfile.dump();
	printf("Exception: %d at %d\n", static_cast<int>(cpu.hu.exc_cause), cpu.hu.exc_pc);
}

struct CPUEnv {
	cpu::CPU cpusim{};
	static constexpr u32 MEM_SZ = 4096;
	std::unique_ptr<u8[]> mem{new u8[MEM_SZ]};

	CPUEnv()
	{
		cpusim.mmu.setMem((u32 *)mem.get(), MEM_SZ);
		u32 mov00 = 0x00002023;
		cpusim.tvec = 4080;
		((u32 *)(mem.get() + cpusim.tvec))[0] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[1] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[2] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[3] = mov00;
	}

	void execute(u32 pc)
	{
		cpusim.fe.s.r.pc = pc;
		cpusim.execute();
	}
};

void test_0()
{
	/*
		ebreak
	*/
	u32 const code[] = {0x00100073U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024);
}

void test_1()
{
	/*
		lw a0, 32(zero)
		lw a1, 36(zero)
		sub a2, a1, a0
		sw a2, 40(zero)
		ebreak
	*/
	u32 const code[] = {0x02002503U, 0x02402583U, 0x40a58633U, 0x02c02423U, 0x00100073U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	*(u32 *)(&env.mem[32]) = 0x21323424;
	*(u32 *)(&env.mem[36]) = 0xdeadbabe;

	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 4);

	assert(env.cpusim.de.regfile.gpr[12] == 0xdeadbabe - 0x21323424);
	assert(*(u32 *)(&env.mem[40]) == 0xdeadbabe - 0x21323424);
}

void test_2()
{
	/*
		jal a0, lab1
		li a1, 123
	lab1:
		li a2, 321
		ebreak
	*/
	u32 const code[] = {0x0080056fU, 0x07b00593U, 0x14100613U, 0x00100073U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 3);

	assert(env.cpusim.de.regfile.gpr[10] == 1024 + 4);
	assert(env.cpusim.de.regfile.gpr[11] == 0);
	assert(env.cpusim.de.regfile.gpr[12] == 321);
}

void test_3()
{
	/*
	start:
		li sp, 1024
		jal ra, main
		ebreak
		nop
	main:
		addi    sp,sp,-16
		sw      s0,12(sp)
		addi    s0,sp,16
		li      a5,0
		mv      a0,a5
		lw      s0,12(sp)
		addi    sp,sp,16
		jr      ra
	*/
	u32 const code[] = {0x40000113U, 0x00c000efU, 0x00100073U, 0x00000013U, 0xff010113U, 0x00812623U,
			    0x01010413U, 0x00000793U, 0x00078513U, 0x00c12403U, 0x01010113U, 0x00008067U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 2);

	assert(env.cpusim.de.regfile.gpr[1] == 1024 + 4 * 2);
	assert(env.cpusim.de.regfile.gpr[2] == 1024);
	assert(env.cpusim.de.regfile.gpr[10] == 0);
}

void test()
{
	test_0();
	test_1();
	test_2();
	test_3();
}

int main()
{
	test();
	return 0;
}