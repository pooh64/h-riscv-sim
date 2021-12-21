#include "sim/cu_maps.h"
#include <array>
#include <cstddef>
#include <tuple>

namespace cpu
{

template <CUIType itype, CUALUCtrl alu_ctrl, CUALUSrc1 src1, CUALUSrc2 src2>
static constexpr CUFlags_Main BuildALUInst()
{
	CUFlags_Main cf;
	cf.itype = itype;
	cf.reg_write = true;
	cf.alu_src1 = src1;
	cf.alu_src2 = src2;
	cf.alu_control = alu_ctrl;
	cf.result_src = CUResSrc::ALU;
	cf.opcode_ok = true;
	return cf;
}

template <CUIType itype, CUALUCtrl alu_ctrl>
static constexpr CUFlags_Main BuildArithm()
{
	static_assert(itype == CUIType::R || itype == CUIType::I);
	if constexpr (itype == CUIType::R)
		return BuildALUInst<itype, alu_ctrl, CUALUSrc1::R, CUALUSrc2::R>();
	return BuildALUInst<itype, alu_ctrl, CUALUSrc1::R, CUALUSrc2::I>();
}

template <CUIType itype, bool jreg>
static constexpr CUFlags_Main BuildJump()
{
	CUFlags_Main cf;
	cf.reg_write = true;
	cf.itype = itype;
	cf.result_src = CUResSrc::PC;
	cf.jump = true;
	cf.jreg = jreg;
	cf.opcode_ok = true;
	return cf;
}

template <CUCMPCtrl cmp_ctrl>
static constexpr CUFlags_Main BuildBranch()
{
	CUFlags_Main cf;
	cf.itype = CUIType::B;
	cf.cmp_control = cmp_ctrl;
	cf.branch = true;
	cf.opcode_ok = true;
	return cf;
}

template <CUMemOp mem_op, bool sgne>
static constexpr CUFlags_Main BuildLoad()
{
	CUFlags_Main cf;
	cf.itype = CUIType::I;
	cf.reg_write = true;
	cf.alu_src1 = CUALUSrc1::R;
	cf.alu_src2 = CUALUSrc2::I;
	cf.alu_control = CUALUCtrl::ADD;
	cf.result_src = CUResSrc::MEM;
	cf.mem_op = mem_op;
	cf.mem_sgne = sgne;
	cf.opcode_ok = true;
	return cf;
}

template <CUMemOp mem_op>
static constexpr CUFlags_Main BuildStore()
{
	CUFlags_Main cf;
	cf.itype = CUIType::S;
	cf.alu_src1 = CUALUSrc1::R;
	cf.alu_src2 = CUALUSrc2::I;
	cf.alu_control = CUALUCtrl::ADD;
	cf.mem_write = true;
	cf.mem_op = mem_op;
	cf.mem_sgne = 0;
	cf.opcode_ok = true;
	return cf;
}

static constexpr CUFlags_Main BuildEcall()
{
	CUFlags_Main cf;
	cf.itype = CUIType::I;
	cf.intpt = true;
	cf.opcode_ok = true;
	return cf;
}

auto constexpr cf_ill = CUFlags_Main();

auto constexpr cf_lui = BuildALUInst<CUIType::U, CUALUCtrl::ARG2, CUALUSrc1::X, CUALUSrc2::I>();
auto constexpr cf_auipc = BuildALUInst<CUIType::U, CUALUCtrl::ADD, CUALUSrc1::PC, CUALUSrc2::I>();
auto constexpr cf_jal = BuildJump<CUIType::J, false>();
auto constexpr cf_jalr = BuildJump<CUIType::I, true>();

auto constexpr cf_beq = BuildBranch<CUCMPCtrl::EQ>();
auto constexpr cf_bne = BuildBranch<CUCMPCtrl::NE>();
auto constexpr cf_blt = BuildBranch<CUCMPCtrl::LT>();
auto constexpr cf_bge = BuildBranch<CUCMPCtrl::GE>();
auto constexpr cf_bltu = BuildBranch<CUCMPCtrl::LTU>();
auto constexpr cf_bgeu = BuildBranch<CUCMPCtrl::GEU>();

auto constexpr cf_lb = BuildLoad<CUMemOp::B, true>();
auto constexpr cf_lh = BuildLoad<CUMemOp::H, true>();
auto constexpr cf_lw = BuildLoad<CUMemOp::W, false>();
auto constexpr cf_lbu = BuildLoad<CUMemOp::B, false>();
auto constexpr cf_lhu = BuildLoad<CUMemOp::H, false>();
auto constexpr cf_sb = BuildStore<CUMemOp::B>();
auto constexpr cf_sh = BuildStore<CUMemOp::H>();
auto constexpr cf_sw = BuildStore<CUMemOp::W>();

auto constexpr cf_addi = BuildArithm<CUIType::I, CUALUCtrl::ADD>();
auto constexpr cf_slti = BuildArithm<CUIType::I, CUALUCtrl::SLT>();
auto constexpr cf_sltiu = BuildArithm<CUIType::I, CUALUCtrl::SLTU>();
auto constexpr cf_xori = BuildArithm<CUIType::I, CUALUCtrl::XOR>();
auto constexpr cf_ori = BuildArithm<CUIType::I, CUALUCtrl::OR>();
auto constexpr cf_andi = BuildArithm<CUIType::I, CUALUCtrl::AND>();
auto constexpr cf_slli = BuildArithm<CUIType::I, CUALUCtrl::SLL>();
auto constexpr cf_srai = BuildArithm<CUIType::I, CUALUCtrl::SRA>();
auto constexpr cf_srli = BuildArithm<CUIType::I, CUALUCtrl::SRL>();

auto constexpr cf_add = BuildArithm<CUIType::R, CUALUCtrl::ADD>();
auto constexpr cf_sub = BuildArithm<CUIType::R, CUALUCtrl::SUB>();
auto constexpr cf_sll = BuildArithm<CUIType::R, CUALUCtrl::SLL>();
auto constexpr cf_slt = BuildArithm<CUIType::R, CUALUCtrl::SLT>();
auto constexpr cf_sltu = BuildArithm<CUIType::R, CUALUCtrl::SLTU>();
auto constexpr cf_xor = BuildArithm<CUIType::R, CUALUCtrl::XOR>();
auto constexpr cf_sra = BuildArithm<CUIType::R, CUALUCtrl::SRA>();
auto constexpr cf_srl = BuildArithm<CUIType::R, CUALUCtrl::SRL>();
auto constexpr cf_or = BuildArithm<CUIType::R, CUALUCtrl::OR>();
auto constexpr cf_and = BuildArithm<CUIType::R, CUALUCtrl::AND>();

auto constexpr cf_ebreak = BuildEcall();

#define CPU_DECODE_SWITCH                                                                                              \
	switch (inst.op) {                                                                                             \
	case 0b0110111:                                                                                                \
		return OP(lui);                                                                                        \
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
			return OP(lb);                                                                                 \
		case 0b001:                                                                                            \
			return OP(lh);                                                                                 \
		case 0b010:                                                                                            \
			return OP(lw);                                                                                 \
		case 0b100:                                                                                            \
			return OP(lbu);                                                                                \
		case 0b101:                                                                                            \
			return OP(lhu);                                                                                \
		default:                                                                                               \
			return OP(ill);                                                                                \
		}                                                                                                      \
	case 0b0100011: /* sX */                                                                                       \
		switch (inst.funct3) {                                                                                 \
		case 0b000:                                                                                            \
			return OP(sb);                                                                                 \
		case 0b001:                                                                                            \
			return OP(sh);                                                                                 \
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
			return OP(slti);                                                                               \
		case 0b011:                                                                                            \
			return OP(sltiu);                                                                              \
		case 0b100:                                                                                            \
			return OP(xori);                                                                               \
		case 0b110:                                                                                            \
			return OP(ori);                                                                                \
		case 0b111:                                                                                            \
			return OP(andi);                                                                               \
		case 0b001:                                                                                            \
			return OP(slli);                                                                               \
		case 0b101:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				return OP(srai);                                                                       \
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
			return OP(sll);                                                                                \
		case 0b010:                                                                                            \
			return OP(slt);                                                                                \
		case 0b011:                                                                                            \
			return OP(sltu);                                                                               \
		case 0b100:                                                                                            \
			return OP(xor);                                                                                \
		case 0b101:                                                                                            \
			if (inst.funct7 >> 5)                                                                          \
				return OP(sra);                                                                        \
			return OP(srl);                                                                                \
		case 0b110:                                                                                            \
			return OP(or);                                                                                 \
		case 0b111:                                                                                            \
			return OP(and);                                                                                \
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
    {0b1110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::I, 0, 0, 0, 0, 0b00, 1)}, // int
    {0b0110011, CUFlags_Main(1, CUResSrc::ALU, CUIMMSrc::I, 0, 0, 0, 0, 0b10, 0)}, // r-type
};
const MapTab<CUALUControl, 128> cu_alu_map{};
#endif

} // namespace cpu