#include "sim/cpu.h"

namespace cpu
{
void CPU::execute()
{
	while (hu.flush_cnt) {
		tick(*this);
	}
}
} // namespace cpu