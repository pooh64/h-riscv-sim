#include "sim/elfload.h"
#include "sim/common.h"
#include <cstring>

extern "C" {
#include <elf.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
};

int elf_mmap(const char *path, struct elf_entry *elf)
{
	int rc;
	int fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;

	struct stat statbuf;
	if ((rc = fstat(fd, &statbuf)) < 0)
		return -1;

	void *map = mmap(NULL, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
	if (map == MAP_FAILED)
		return -1;

	auto ehdr = (Elf32_Ehdr *)map;

	if (memcmp(&ehdr->e_ident,
		   "\x7f"
		   "ELF",
		   4))
		return -1;

	auto entryva = ehdr->e_entry;
	auto phoff = ehdr->e_phoff;
	auto phnum = ehdr->e_phnum;
	auto phentsize = ehdr->e_phentsize;

	Elf32_Phdr *to_load = nullptr;

	for (i32 i = 0; i < phnum; ++i) {
		auto ph = (Elf32_Phdr *)((u8 *)map + phoff + phentsize * i);
		if (ph->p_type != PT_LOAD)
			continue;
		if (ph->p_vaddr <= entryva && ph->p_vaddr + ph->p_memsz > entryva) {
			to_load = ph;
			break;
		}
	}
	if (!to_load)
		return -1;

	elf->ptr = (u8 *)map + to_load->p_offset;
	elf->sz = to_load->p_filesz;
	elf->entry = entryva - to_load->p_vaddr;
	return 0;
}
