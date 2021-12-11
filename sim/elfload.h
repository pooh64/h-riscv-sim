#pragma once

#include <cstddef>

struct elf_entry {
	void *ptr;
	size_t sz;
	size_t entry;
};

int elf_mmap(const char *path, struct elf_entry *elf);