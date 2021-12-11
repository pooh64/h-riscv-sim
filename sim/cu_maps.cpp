#include "sim/cu_maps.h"
#include <array>
#include <cstddef>
#include <tuple>

namespace cpu
{

#define D constexpr CUFlags_Main
D cf_ill{};
D cf_auipc(CUIType::U, 1, CUALUSrc1::PC, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_jal(CUIType::J, 1, CUALUSrc1::X, CUALUSrc2::X, CUALUCtrl::X, CUCMPCtrl::X, CUResSrc::PC, 0, 0, 1, 0, 0);
D cf_jalr(CUIType::I, 1, CUALUSrc1::X, CUALUSrc2::X, CUALUCtrl::X, CUCMPCtrl::X, CUResSrc::PC, 0, 0, 1, 1, 0);
#define B(name, cond)                                                                                                  \
	constexpr CUFlags_Main name(CUIType::B, 0, CUALUSrc1::X, CUALUSrc2::X, CUALUCtrl::X, CUCMPCtrl::cond,          \
				    CUResSrc::X, 0, 1, 0, 0, 0)
B(cf_beq, EQ);
B(cf_bne, NE);
B(cf_blt, LT);
B(cf_bge, GE);
B(cf_bltu, LTU);
B(cf_bgeu, GEU);
#undef B
D cf_lw(CUIType::I, 1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::MEM, 0, 0, 0, 0, 0);
D cf_sw(CUIType::S, 0, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::X, 1, 0, 0, 0, 0);
D cf_addi(CUIType::I, 1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_add(CUIType::R, 1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_sub(CUIType::R, 1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::SUB, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_ecall(CUIType::I, 0, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::X, CUCMPCtrl::X, CUResSrc::X, 0, 0, 0, 0, 1);
#undef D

#define CPU_DECODE_SWITCH                                                                                              \
	CUFlags_Main res;                                                                                              \
	switch (inst.op) {                                                                                             \
	case 0b0010111:                                                                                                \
		return OP(auipc);                                                                                      \
	case 0b1101111:                                                                                                \
		return OP(jal);                                                                                        \
	case 0b1100111:                                                                                                \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(jalr);                                                                               \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b1100011:                                                                                                \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(beq);                                                                                \
		case 0b001:                                                                                            \
			return OP(bne);                                                                                \
		case 0b100:                                                                                            \
			return OP(blt);                                                                                \
		case 0b101:                                                                                            \
			return OP(bge);                                                                                \
		case 0b110:                                                                                            \
			return OP(bltu);                                                                               \
		case 0b111:                                                                                            \
			return OP(bgeu);                                                                               \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0000011: /* lX */                                                                                       \
		switch (inst.funct3) {                                                                                 \
		case 0b010:                                                                                            \
			return OP(lw);                                                                                 \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0100011: /* sX */                                                                                       \
		switch (inst.funct3) {                                                                                 \
		case 0b010:                                                                                            \
			return OP(sw);                                                                                 \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b1110011:                                                                                                \
		return OP(ecall);                                                                                      \
	case 0b0110011: /* r-type arithm */                                                                            \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				return OP(sub);                                                                        \
			return OP(add);                                                                                \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0010011: /* i-type arithm */                                                                            \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(addi);                                                                               \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	default:                                                                                                       \
		return OP(ill);                                                                                        \
	}                                                                                                              \
	return OP(ill);

CUFlags_Main GetCUFlags(Instr inst)
{
#define OP(instn) cf_##instn
	CPU_DECODE_SWITCH
#undef OP
}

char const *GetOpcodeStr(Instr inst){
#define OP(instn) #instn
    CPU_DECODE_SWITCH
#undef OP
}

std::ostream &
operator<<(std::ostream &os, Instr inst)
{
	return os << GetOpcodeStr(inst);
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