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

#ifndef BH1750_BH1750_H_2D8F8E34_8C06_4D9C_904D_C4361954AA72
#define BH1750_BH1750_H_2D8F8E34_8C06_4D9C_904D_C4361954AA72

#include <chrono>
#include <cstdint>
#include "Instruction.h"

namespace BH1750 {
class BH1750 {

/**
 * Datasheet
 * https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
 */

public:

    static const int DEFAULT_I2C_DEVICE = 1;

    /**
     * When BH1750 address pin is <= 0.3VCC, I2C address is 0x23
     * Datasheet pg. 10
     */
    static const int I2C_LOW_ADDR = 0x23;

    /**
     * When BH1750 address pin is >= 0.7VCC, I2C address is 0x5C
     * Datasheet pg. 10
     */
    static const int I2C_HIGH_ADDR = 0x5C;
    
    /**
     * Values taken from datasheet pg. 2
     */
    static constexpr float MIN_MEASUREMENT_ACCURACY = 0.96f;
    static constexpr float TYP_MEASUREMENT_ACCURACY = 1.2f;
    static constexpr float MAX_MEASUREMENT_ACCURACY = 1.44f;
    
    /**
     * Values taken from datasheet pg. 11
     */
    static const std::uint8_t MIN_MTREG = 31;
    static const std::uint8_t TYP_MTREG = 69;
    static const std::uint8_t MAX_MTREG = 254;
    
    /**
     * Values taken from datasheet pg. 2
     */
    static constexpr auto TYP_HIGH_RES_TIME = std::chrono::milliseconds(120);
    static constexpr auto MAX_HIGH_RES_TIME = std::chrono::milliseconds(180);
    static constexpr auto TYP_LOW_RES_TIME = std::chrono::milliseconds(16);
    static constexpr auto MAX_LOW_RES_TIME = std::chrono::milliseconds(24);

    static bool isHighRes(const MeasurementMode mode) noexcept;
    static bool isLowRes(const MeasurementMode mode) noexcept;
    static bool isMode2(const MeasurementMode mode) noexcept;
    static bool isContinuous(const MeasurementMode mode) noexcept;
    static bool isOneTime(const MeasurementMode mode) noexcept;

    /**
     * Calculates the wait time needed between measurement and data ready
     * @param  {uint8_t} mt           : time
     * @param  {MeasurementMode} mode : mode
     * @param  {bool} maxWait         : whether to return the max wait time (as opposed to typical)
     */
    static std::chrono::milliseconds calculateWaitTime(
        const std::uint8_t mt,
        const MeasurementMode mode,
        const bool maxWait = true) noexcept;

    /**
     * Convert a light level from the sensor into lux level
     * @param  {uint16_t} level       : raw light level from sensor
     * @param  {MeasurementMode} mode : mode
     * @param  {uint8_t} mt           : time
     * @param  {float} acc            : accuracy
     * @return {double}               : lux
     */
    static double convertLevel(
        const std::uint16_t level,
        const MeasurementMode mode,
        const std::uint8_t mt,
        const float acc) noexcept;

    /**
     * BH1750 
     * 
     * @param  {int8_t} addr  : I2C address
     * @param  {char*} device : I2C device
     */
    BH1750(
        const int device = DEFAULT_I2C_DEVICE,
        const int addr = I2C_LOW_ADDR) noexcept;

    virtual ~BH1750();

    MeasurementMode getMeasurementMode() const noexcept;
    MeasurementMode getLastMeasurementMode() const noexcept;
    std::uint8_t getMeasurementTime() const noexcept;
    float getMeasurementAccuracy() const noexcept;
    PowerMode getPowerMode() const noexcept;

    /**
     * @param  {MeasurementMode} mm : mode
     * @param  {uint8_t} mt         : time
     * @param  {float} acc          : accuracy
     */
    void connect(
        const MeasurementMode mm = MeasurementMode::CONTINUOUS_HIGH_RES_MODE,
        const std::uint8_t mt = TYP_MTREG,
        const float acc = TYP_MEASUREMENT_ACCURACY);

    void disconnect();

    /**
     * Instruct the sensor to measure light level
     */
    void measure();

    /**
     * Clear the sensor's measurement register
     */
    void reset() const;

    /**
     * Change sensor settings
     * @param  {MeasurementMode} mode : mode
     * @param  {uint8_t} mt           : time
     * @param  {float} acc            : accuracy
     */
    void configure(
        const MeasurementMode mode,
        const std::uint8_t mt = TYP_MTREG,
        const float acc = TYP_MEASUREMENT_ACCURACY);

    /**
     * Read the currently stored light level from the sensor
     * @return {uint16_t}  : 
     */
    std::uint16_t readLevel() const;

    /**
     * Measure lux from the sensor
     * @return {double}  : 
     */
    double lux();

    void powerDown();
    void powerUp();


protected:

    int _handle;
    const int _dev;
    const int _addr;

    PowerMode _powerMode;
    MeasurementMode _measurementMode;
    MeasurementMode _lastMeasurementMode;
    float _accuracy = TYP_MEASUREMENT_ACCURACY;
    std::uint8_t _mtReg;

    void _setMeasurementMode(const MeasurementMode mode);
    void _setMeasurementTimeRegister(const std::uint8_t mt);
    void _setMeasurementAccuracy(const float acc);

};
};
#endif