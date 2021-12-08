#include "sim/cpu.h"
namespace cpu
{
void CPU::execute()
{
	while (!shutdown) {
		tick(*this);
	}
}

} // namespace cpu