#include "sim/cu_maps.h"
#include <array>
#include <cstddef>
#include <tuple>

namespace cpu
{

#define D constexpr CUFlags_Main
D cf_ill{};
D cf_jal(1, CUALUSrc1::X, CUALUSrc2::X, CUALUCtrl::X, CUCMPCtrl::X, CUIMMSrc::J, CUResSrc::PC, 0, 0, 1, 0, 0);
D cf_jalr(1, CUALUSrc1::X, CUALUSrc2::X, CUALUCtrl::X, CUCMPCtrl::X, CUIMMSrc::I, CUResSrc::PC, 0, 0, 1, 1, 0);
D cf_lw(1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::I, CUResSrc::MEM, 0, 0, 0, 0, 0);
D cf_sw(0, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::S, CUResSrc::X, 1, 0, 0, 0, 0);
D cf_addi(1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::I, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_add(1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::ADD, CUCMPCtrl::X, CUIMMSrc::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_sub(1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::SUB, CUCMPCtrl::X, CUIMMSrc::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_int(0, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::X, CUCMPCtrl::X, CUIMMSrc::I, CUResSrc::X, 0, 0, 0, 0, 1);
#undef D

CUFlags_Main GetCUFlags(Instr inst)
{
	CUFlags_Main res;
	switch (inst.op) {
	case 0b1101111:
		return cf_jal;
	case 0b1100111:
		switch (inst.funct3) {
		case 0b000:
			return cf_jalr;
		default:
			return cf_ill;
		}
	case 0b0000011: // lX
		switch (inst.funct3) {
		case 0b010:
			return cf_lw;
		default:
			return cf_ill;
		}
	case 0b0100011: // sX
		switch (inst.funct3) {
		case 0b010:
			return cf_sw;
		default:
			return cf_ill;
		}
	case 0b1110011:
		return cf_int;
	case 0b0110011: // r-type arithm
		switch (inst.funct3) {
		case 0b000:
			if (inst.funct7 >> 5)
				return cf_sub;
			return cf_add;
		default:
			return cf_ill;
		}
	case 0b0010011: // i-type arithm
		switch (inst.funct3) {
		case 0b000:
			return cf_addi;
		default:
			return cf_ill;
		}
	default:
		return cf_ill;
	}
	return cf_ill;
}

#if 0
const MapTab<CUFlags_Main, 128> cu_flags_map{
    {0b0000011, CUFlags_Main(1, CUResSrc::MEM, CUIMMSrc::I, 1, 0, 0, 0, 0b00, 0)}, // lw
    {0b1110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::I, 0, 0, 0, 0, 0b00, 1)}, // irq
    {0b0110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::I, 0, 0, 0, 0, 0b10, 0)}, // r-type
};
const MapTab<CUALUControl, 128> cu_alu_map{};
#endif

} // namespace cpu