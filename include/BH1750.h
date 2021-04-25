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

#include <cstdint>
#include <chrono>
#include "Instruction.h"

namespace BH1750 {
class BH1750 {

public:

	static constexpr const char* const DEFAULT_I2C_DEVICE = "/dev/i2c-1";
	
	//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
	//pg. 10
	//When ADDR pin is high (>= 0.7VCC), I2C address is 0x5C
	//When ADDR pin is low (<= 0.3VCC), I2C address is 0x23
	static const int8_t I2C_LOW_ADDR = 0x23;
	static const int8_t I2C_HIGH_ADDR = 0x5C;
	
	static constexpr float MIN_MEASUREMENT_ACCURACY = 0.96f;
	static constexpr float TYP_MEASUREMENT_ACCURACY = 1.2f;
	static constexpr float MAX_MEASUREMENT_ACCURACY = 1.44f;
	static const uint8_t MIN_MTREG = 31;
	static const uint8_t TYP_MTREG = 69;
	static const uint8_t MAX_MTREG = 254;
	static constexpr std::chrono::milliseconds TYP_HIGH_RES_TIME = std::chrono::milliseconds(120);
	static constexpr std::chrono::milliseconds MAX_HIGH_RES_TIME = std::chrono::milliseconds(180);
	static constexpr std::chrono::milliseconds TYP_LOW_RES_TIME = std::chrono::milliseconds(16);
	static constexpr std::chrono::milliseconds MAX_LOW_RES_TIME = std::chrono::milliseconds(24);

	static bool isHighRes(const MeasurementMode mode) noexcept;
	static bool isLowRes(const MeasurementMode mode) noexcept;
	static bool isMode2(const MeasurementMode mode) noexcept;
	static bool isContinuous(const MeasurementMode mode) noexcept;
	static bool isOneTime(const MeasurementMode mode) noexcept;

	static std::chrono::milliseconds calculateWaitTime(
		const uint8_t mt,
		const MeasurementMode mode,
		const bool maxWait = true) noexcept;

	/**
	 * Converts a sensor light level into lux level depending on current mode
	 * @param  {uint16_t} level : 
	 * @return {double}         : 
	 */
	static double convertLevel(
		const uint16_t level,
		const MeasurementMode mode,
		const uint8_t mt,
		const float acc) noexcept;


protected:
	const char* _dev;
	const int8_t _addr;
	int _fd = -1;
	PowerMode _powerMode = PowerMode::POWER_DOWN;
	MeasurementMode _measurementMode = MeasurementMode::CONTINUOUS_HIGH_RES_MODE;
	MeasurementMode _lastMeasurementMode = MeasurementMode::CONTINUOUS_HIGH_RES_MODE;
	float _accuracy = TYP_MEASUREMENT_ACCURACY;
	uint8_t _mtReg = TYP_MTREG;

	void _setMeasurementMode(const MeasurementMode mode);
	void _setMeasurementTimeRegister(const uint8_t mt);
	void _setMeasurementAccuracy(const float acc);


public:

	BH1750(
		const int8_t addr = I2C_LOW_ADDR,
		const char* device = DEFAULT_I2C_DEVICE) noexcept;

	~BH1750();
	
	MeasurementMode getMeasurementMode() const noexcept;
	MeasurementMode getLastMeasurementMode() const noexcept;
	uint8_t getMeasurementTime() const noexcept;
	float getMeasurementAccuracy() const noexcept;

	void connect(
		const MeasurementMode mm = MeasurementMode::CONTINUOUS_HIGH_RES_MODE,
		const uint8_t mt = TYP_MTREG);

	/**
	 * Instruct the sensor to measure light level
	 */
	void measure();

	/**
	 * Clear the sensor's measurement register
	 */
	void reset() const;

	/**
	 * This function exists because of the chance setting mt reg will be incorrect
	 * if mode is changed
	 * @param  {MeasurementMode} mode : 
	 * @param  {uint8_t} mt           : 
	 */
	void configure(
		const MeasurementMode mode,
		const uint8_t mt = TYP_MTREG,
		const float acc = TYP_MEASUREMENT_ACCURACY);

	/**
	 * Read the currently stored light level from the sensor
	 * @return {uint16_t}  : 
	 */
	uint16_t readLevel() const;

	/**
	 * Measure lux from the sensor
	 * @return {double}  : 
	 */
	double lux();

	void power_down();
	void power_up();

};
};
#endif