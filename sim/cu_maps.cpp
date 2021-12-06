#include "sim/cu_maps.h"
#include <array>
#include <cstddef>
#include <tuple>

namespace cpu
{

const MapTab<CUFlags_Main, 128> cu_flags_map{
    {0b0000011, CUFlags_Main(1, CUResultSrc::MEM, CUIMMSrc::TYPE_I, 1, 0, 0, 0, 0b00, 0)}, // lw
    {0b1110011, CUFlags_Main(1, CUResultSrc::ALU, CUIMMSrc::TYPE_I, 0, 0, 0, 0, 0b00, 1)}, // irq
    {0b0110011, CUFlags_Main(1, CUResultSrc::ALU, CUIMMSrc::TYPE_I, 0, 0, 0, 0, 0b10, 0)}, // r-type
};

const MapTab<CUALUControl, 128> cu_alu_map{};

} // namespace cpu