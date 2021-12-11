#include "sim/cpu.h"
#include "sim/elfload.h"

#include <bit>
#include <iostream>
#include <memory>

void DumpExc(cpu::CPU &cpu)
{
	cpu.de.regfile.dump();
	printf("Exception: %d at %d\n", static_cast<int>(cpu.hu.exc_cause), cpu.hu.exc_pc);
}

struct CPUEnv {
	cpu::CPU cpusim{};
	std::unique_ptr<u8[]> mem;
	static constexpr u32 tvec_hanlder_sz = 16;

	CPUEnv(u32 mem_sz = 4096, u32 tvec = 4096 - tvec_hanlder_sz) : mem{new u8[mem_sz]}
	{
		cpusim.mmu.setMem((u32 *)mem.get(), mem_sz);
		u32 mov00 = 0x00002023;
		cpusim.tvec = tvec;
		((u32 *)(mem.get() + cpusim.tvec))[0] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[1] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[2] = mov00;
		((u32 *)(mem.get() + cpusim.tvec))[3] = mov00;
	}
	void execute(u32 pc);
};

void CPUEnv::execute(u32 pc)
{
	cpusim.fe.s.r.pc = pc;
	// cpusim.execute();
	while (!cpusim.shutdown) {
		if constexpr (false) {
			cpusim.trace(std::cout);
			std::cout << "\n";
		}
		cpusim.tick(cpusim);
	}
	std::cout << "cycles: " << cpusim.time << "\n";
}

void test_0()
{
	/*
		ebreak
	*/
	u32 const code[] = {0x00100073U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	std::cout << "**************** test_0\n";
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

	std::cout << "**************** test_1\n";
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

	std::cout << "**************** test_2\n";
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

	std::cout << "**************** test_3\n";
	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 2);

	assert(env.cpusim.de.regfile.gpr[1] == 1024 + 4 * 2);
	assert(env.cpusim.de.regfile.gpr[2] == 1024);
	assert(env.cpusim.de.regfile.gpr[10] == 0);
}

void test_4()
{
	/*
	start:
		li sp, 1024
		jal ra, main
		ebreak
		nop
	main:
		addi    sp,sp,-32
		sw      s0,28(sp)
		addi    s0,sp,32
		sw      zero,-20(s0)
		sw      zero,-24(s0)
		j       .L3
	.L4:
		lw      a5,-20(s0)
		addi    a5,a5,2
		sw      a5,-20(s0)
		lw      a5,-24(s0)
		addi    a5,a5,1
		sw      a5,-24(s0)
	.L3:
		lw      a4,-24(s0)
		li      a5,2
		ble     a4,a5,.L4
		lw      a5,-20(s0)
		mv      a0,a5
		lw      s0,28(sp)
		addi    sp,sp,32
		jr      ra
	*/
	u32 const code[] = {0x40000113U, 0x00c000efU, 0x00100073U, 0x00000013U, 0xfe010113U, 0x00812e23U,
			    0x02010413U, 0xfe042623U, 0xfe042423U, 0x01c0006fU, 0xfec42783U, 0x00278793U,
			    0xfef42623U, 0xfe842783U, 0x00178793U, 0xfef42423U, 0xfe842703U, 0x00200793U,
			    0xfee7d0e3U, 0xfec42783U, 0x00078513U, 0x01c12403U, 0x02010113U, 0x00008067U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	std::cout << "**************** test_4\n";
	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 2);

	assert(env.cpusim.de.regfile.gpr[1] == 1024 + 4 * 2);
	assert(env.cpusim.de.regfile.gpr[2] == 1024);
	assert(env.cpusim.de.regfile.gpr[10] == 6);
}

void test_5()
{
	/* multiplication via loop, recursive fact */
	u32 const code[] = {0x40000113U, 0x0c4000efU, 0x00100073U, 0x00000013U, 0xfd010113U, 0x02812623U, 0x03010413U,
			    0xfca42e23U, 0xfcb42c23U, 0xfe042623U, 0xfe042423U, 0x0200006fU, 0xfec42703U, 0xfd842783U,
			    0x00f707b3U, 0xfef42623U, 0xfe842783U, 0x00178793U, 0xfef42423U, 0xfe842703U, 0xfdc42783U,
			    0xfcf74ee3U, 0xfec42783U, 0x00078513U, 0x02c12403U, 0x03010113U, 0x00008067U, 0xfe010113U,
			    0x00112e23U, 0x00812c23U, 0x02010413U, 0xfea42623U, 0xfec42783U, 0x02078663U, 0xfec42783U,
			    0xfff78793U, 0x00078513U, 0xfd9ff0efU, 0x00050793U, 0x00078593U, 0xfec42503U, 0xf6dff0efU,
			    0x00050793U, 0x0080006fU, 0x00100793U, 0x00078513U, 0x01c12083U, 0x01812403U, 0x02010113U,
			    0x00008067U, 0xff010113U, 0x00112623U, 0x00812423U, 0x01010413U, 0x00500513U, 0xf91ff0efU,
			    0x00050793U, 0x00078513U, 0x00c12083U, 0x00812403U, 0x01010113U, 0x00008067U};
	CPUEnv env{};
	memcpy(env.mem.get() + 1024, code, sizeof(code));

	std::cout << "**************** test_5\n";
	env.execute(1024);
	assert(env.cpusim.hu.exc_pc == 1024 + 4 * 2);

	assert(env.cpusim.de.regfile.gpr[1] == 1024 + 4 * 2);
	assert(env.cpusim.de.regfile.gpr[2] == 1024);
	assert(env.cpusim.de.regfile.gpr[10] == 120);
}

void test()
{
	test_0();
	test_1();
	test_2();
	test_3();
	test_4();
	test_5();
}

void execute_elf(char const *path)
{
	std::cout << "**************** execute elf\n";
	elf_entry elf;
	if (elf_mmap(path, &elf) < 0)
		return;

	constexpr u32 load_offs = 4096;

	u32 mem_sz = std::__bit_ceil(load_offs + elf.sz + CPUEnv::tvec_hanlder_sz);
	CPUEnv env(mem_sz, mem_sz - CPUEnv::tvec_hanlder_sz);
	memcpy(env.mem.get() + load_offs, elf.ptr, elf.sz);

	std::cout << "**************** start execution\n";
	u32 entry_va = load_offs + elf.entry;
	env.execute(entry_va);
	std::cout << "epc: " << env.cpusim.hu.exc_pc << "\n";
	std::cout << "ecause: " << (u32) env.cpusim.hu.exc_cause << "\n";
	assert(env.cpusim.hu.exc_pc == entry_va + 4 * 2);
	assert(env.cpusim.hu.exc_cause == cpu::PL_HU::ExcType::INT);
	std::cout << "Process returned: " << env.cpusim.de.regfile.gpr[10] << "\n";
}

int main(int argc, char **argv)
{
	if (argc == 1) {
		test();
	} else if (argc == 2) {
		execute_elf(argv[1]);
	}
	return 0;
}