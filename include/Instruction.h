// MIT License
//
// Copyright (c) 2021 Daniel Robertson
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BH1750_INSTRUCTION_H_345C5904_C6EC_4C38_BA76_BAE2D327037E
#define BH1750_INSTRUCTION_H_345C5904_C6EC_4C38_BA76_BAE2D327037E

#include <cstdint>

namespace BH1750 {

/**
 * Values taken from datasheet pg. 5
 */

enum class Instruction : uint8_t {
    RESET =	0b00000111
};

enum class MeasurementMode : uint8_t {
    CONTINUOUS_HIGH_RES_MODE = 0b00010000,
    CONTINUOUS_HIGH_RES_MODE_2 = 0b00010001,
    CONTINUOUS_LOW_RES_MODE = 0b00010011,
    ONE_TIME_HIGH_RES_MODE = 0b00100000,
    ONE_TIME_HIGH_RES_MODE_2 = 0b00100001,
    ONE_TIME_LOW_RES_MODE = 0b00100011
};

enum class PowerMode : uint8_t {
    POWER_UP = 0b00000001,
    POWER_DOWN = 0b00000000
};

};
#endif
