#include "sim/cu_maps.h"
#include <array>
#include <cstddef>
#include <tuple>

namespace cpu
{

#define D constexpr CUFlags_Main
D cf_ill{};
D cf_lw(1, 1, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::TYPE_I, CUResSrc::MEM, 0, 0, 0, 0);
D cf_int(0, 0, CUALUCtrl::X, CUCMPCtrl::X, CUIMMSrc::TYPE_I, CUResSrc::X, 0, 0, 0, 1);
D cf_add(1, 0, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::X, CUResSrc::ALU, 0, 0, 0, 0);
D cf_sub(1, 0, CUALUCtrl::SUB, CUCMPCtrl::X, CUIMMSrc::X, CUResSrc::ALU, 0, 0, 0, 0);
#undef D

CUFlags_Main GetCUFlags(Instr inst)
{
	CUFlags_Main res;
	switch (inst.op) {
	case 0b0000011:
		return cf_lw;
	case 0b1110011:
		return cf_int;
	case 0b0110011: { // r-type
		switch (inst.funct3) {
		case 0b000:
			if (inst.funct7 >> 5)
				return cf_sub;
			return cf_add;
		}
	}
	}
	return cf_ill;
}

#if 0
const MapTab<CUFlags_Main, 128> cu_flags_map{
    {0b0000011, CUFlags_Main(1, CUResSrc::MEM, CUIMMSrc::TYPE_I, 1, 0, 0, 0, 0b00, 0)}, // lw
    {0b1110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::TYPE_I, 0, 0, 0, 0, 0b00, 1)}, // irq
    {0b0110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::TYPE_I, 0, 0, 0, 0, 0b10, 0)}, // r-type
};
const MapTab<CUALUControl, 128> cu_alu_map{};
#endif

} // namespace cpu