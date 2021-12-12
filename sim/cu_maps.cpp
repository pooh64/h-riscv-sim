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
D cf_andi(CUIType::I, 1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::AND, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_slli(CUIType::I, 1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::SLL, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_srli(CUIType::I, 1, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::SRL, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_add(CUIType::R, 1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::ADD, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_sub(CUIType::R, 1, CUALUSrc1::R, CUALUSrc2::R, CUALUCtrl::SUB, CUCMPCtrl::X, CUResSrc::ALU, 0, 0, 0, 0, 0);
D cf_ebreak(CUIType::I, 0, CUALUSrc1::R, CUALUSrc2::I, CUALUCtrl::X, CUCMPCtrl::X, CUResSrc::X, 0, 0, 0, 0, 1);
#undef D

#define CPU_DECODE_SWITCH                                                                                              \
	CUFlags_Main res;                                                                                              \
	switch (inst.op) {                                                                                             \
	case 0b0110111:                                                                                                \
		assert(!"lui");                                                                                        \
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
		case 0b000:                                                                                            \
			assert(!"lb");                                                                                 \
		case 0b001:                                                                                            \
			assert(!"lh");                                                                                 \
		case 0b010:                                                                                            \
			return OP(lw);                                                                                 \
		case 0b100:                                                                                            \
			assert(!"lbu");                                                                                \
		case 0b101:                                                                                            \
			assert(!"lhu");                                                                                \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0100011: /* sX */                                                                                       \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			assert(!"sb");                                                                                 \
		case 0b001:                                                                                            \
			assert(!"sh");                                                                                 \
		case 0b010:                                                                                            \
			return OP(sw);                                                                                 \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0010011: /* i-type arithm */                                                                            \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(addi);                                                                               \
		case 0b010:                                                                                            \
			assert(!"slti");                                                                               \
		case 0b011:                                                                                            \
			assert(!"sltiu");                                                                              \
		case 0b100:                                                                                            \
			assert(!"xori");                                                                               \
		case 0b110:                                                                                            \
			assert(!"ori");                                                                                \
		case 0b111:                                                                                            \
			return OP(andi);                                                                               \
		case 0b001:                                                                                            \
			return OP(slli);                                                                               \
		case 0b101:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				assert(!"srai");                                                                       \
			return OP(srli);                                                                               \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0110011: /* r-type arithm */                                                                            \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				return OP(sub);                                                                        \
			return OP(add);                                                                                \
		case 0b001:                                                                                            \
			assert(!"sll");                                                                                \
		case 0b010:                                                                                            \
			assert(!"slt");                                                                                \
		case 0b011:                                                                                            \
			assert(!"sltu");                                                                               \
		case 0b100:                                                                                            \
			assert(!"xor");                                                                                \
		case 0b101:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				assert("!sra");                                                                        \
			assert(!"srl");                                                                                \
		case 0b110:                                                                                            \
			assert(!"or");                                                                                 \
		case 0b111:                                                                                            \
			assert(!"and");                                                                                \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0001111:                                                                                                \
		assert(!"fence");                                                                                      \
	case 0b1110011:                                                                                                \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(ebreak);                                                                             \
		default:                                                                                               \
			assert(!"csr*");                                                                               \
		}                                                                                                      \
	default:                                                                                                       \
		return OP(ill);                                                                                        \
	}

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