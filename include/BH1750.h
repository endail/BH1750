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
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <wiringPiI2C.h>
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
	static constexpr float MAX_MEASUREMENT_ACCURAC = 1.44f;
	static const uint8_t MIN_MTREG = 31;
	static const uint8_t TYP_MTREG = 69;
	static const uint8_t MAX_MTREG = 254;
	static constexpr std::chrono::milliseconds TYP_HIGH_RES_TIME = std::chrono::milliseconds(120);
	static constexpr std::chrono::milliseconds MAX_HIGH_RES_TIME = std::chrono::milliseconds(180);
	static constexpr std::chrono::milliseconds TYP_LOW_RES_TIME = std::chrono::milliseconds(16);
	static constexpr std::chrono::milliseconds MAX_LOW_RES_TIME = std::chrono::milliseconds(24);


	static bool isHighRes(const MeasurementMode mode) noexcept {
		
		switch(mode) {
			case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
			case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
			case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
			case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
				return true;
		}

		return false;

	}

	static bool isLowRes(const MeasurementMode mode) noexcept {

		switch(mode) {
			case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
			case MeasurementMode::ONE_TIME_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	static bool isMode2(const MeasurementMode mode) noexcept {

		switch(mode) {
			case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
			case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
				return true;
		}

		return false;

	}

	static bool isContinuous(const MeasurementMode mode) noexcept {

		switch(mode) {
			case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
			case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
			case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	static bool isOneTime(const MeasurementMode mode) noexcept {

		switch(mode) {
			case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
			case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
			case MeasurementMode::ONE_TIME_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	static std::chrono::duration calculateWaitTime(
		const uint8_t mt,
		const MeasurementMode mode,
		const bool maxWait = true) noexcept {

		using namespace std::chrono;

			const double ms = static_cast<double>(mt) / static_cast<double>(TYP_MTREG);
			
			if(isHighRes(mode)) {
				return milliseconds(ms * maxWait ? MAX_HIGH_RES_TIME : TYP_HIGH_RES_TIME);
			}

			return milliseconds(ms * maxWait ? MAX_LOW_RES_TIME : TYP_LOW_RES_TIME);

	}

	/**
	 * Converts a sensor light level into lux level depending on current mode
	 * @param  {uint16_t} level : 
	 * @return {double}         : 
	 */
	static double convertLevel(
		const uint16_t level,
		const MeasurementMode mode,
		const uint8_t mt) noexcept {

			double temp = static_cast<double>(level);

			if(isHighRes(mode)) {

				//all high res modes
				temp = temp * static_cast<double>(TYP_MTREG) / static_cast<double>(mt);

				//only for high res and mode 2
				if(isMode2(mode)) {
					temp = temp / 2.0;
				}

			}

			//all modes
			return temp * TYP_MEASUREMENT_ACCURACY;

	}


protected:
	const char* _dev;
	const int8_t _addr;
	int _fd = -1;
	PowerMode _powerMode = PowerMode::POWER_OFF;
	MeasurementMode _measurementMode;
	MeasurementMode _lastMeasurementMode;
	uint8_t _mtReg = TYP_MTREG;

	void _setMeasurementMode(const MeasurementMode mode) {

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(mode)) < 0) {
			throw std::runtime_error("failed to set measurement mode");
		}

		this->_measurementMode = mode;

	}

	void _setMeasurementTimeRegister(const uint8_t mt) {

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 11
		//range is between 31 (inc) and 254 (inc)
		if(!(mt >= MIN_MTREG && mt <= MAX_MTREG)) {
			throw std::range_error("reg must be between 31 and 254");
		}

		//there's an issue here with measurement mode
		//if setMeasurementMode is called AFTER this function, the
		//config will be incorrect
		//
		//is the third I2C write call needed?
		if(	::wiringPiI2CWrite(this->_fd, 0b01000000 | (mt & 0b11100000)) < 0 ||
			::wiringPiI2CWrite(this->_fd, 0b01100000 | (mt & 0b00011111)) < 0 ||
			::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(this->_measurementMode)) < 0 ) {
				throw std::runtime_error("failed to set measurement register");
		}

		this->_mtReg = mt;

	}


public:

	BH1750(
		const int8_t addr = I2C_LOW_ADDR,
		const char* device = DEFAULT_I2C_DEVICE) noexcept 
			: _dev(device), _addr(addr) {
	}

	~BH1750() {
		::close(this->_fd);
	}

	MeasurementMode getMeasurementMode() const noexcept {
		return this->_measurementMode;
	}

	MeasurementMode getLastMeasurementMode() const noexcept {
		return this->_lastMeasurementMode;
	}

	uint8_t getMeasurementTime() const noexcept {
		return this->_mtReg;
	}

	void connect(
		const MeasurementMode mm = MeasurementMode::CONTINUOUS_HIGH_RES_MODE,
		const uint8_t mt = TYP_MTREG) {

			//only permit this function to execute if no file descriptor set
			if(this->_fd >= 0) {
				throw std::runtime_error("sensor is already connected");
			}

			if((this->_fd = ::wiringPiI2CSetupInterface(this->_dev, this->_addr)) < 0) {
				throw std::runtime_error("failed to connect to device");
			}

			this->_powerMode = PowerMode::POWER_ON;
			this->_setMeasurementMode(mm);
			this->_setMeasurementTimeRegister(mt);

	}

	/**
	 * Instruct the sensor to measure light level
	 */
	void measure() {

		if(this->_powerMode == PowerMode::POWER_DOWN) {
			throw std::runtime_error("sensor is powered down");
		}
		
		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(this->_measurementMode)) < 0) {
			throw std::runtime_error("failed to measure light level");
		}

		this->_lastMeasurementMode = this->_measurementMode;

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 5
		//One Time modes return the device to power down mode after
		//measurement
		if(isOneTime(this->_lastMeasurementMode)) {
			this->_powerMode = PowerMode::POWER_DOWN;
		}

	}

	/**
	 * Clear the sensor's measurement register
	 */
	void reset() const {

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 5
		//"Reset Data register value. Reset command is not acceptable in
		//Power Down mode."
		if(this->_mode == PowerMode::POWER_DOWN) {
			throw std::runtime_error("cannot reset while powered down");
		}

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(Instruction::RESET)) < 0) {
			throw std::runtime_error("failed to reset device registers");
		}

	}

	/**
	 * This function exists because of the chance setting mt reg will be incorrect
	 * if mode is changed
	 * @param  {MeasurementMode} mode : 
	 * @param  {uint8_t} mt           : 
	 */
	void configure(const MeasurementMode mode, const uint8_t mt = TYP_MTREG) {
		this->_setMeasurementMode(mode);
		this->_setMeasurementTimeRegister(mt);
	}

	/**
	 * Read the currently stored light level from the sensor
	 * @return {uint16_t}  : 
	 */
	uint16_t readLevel() const {

		//may need to replace this with two 8-bit reads
		//or lower-level 16bit read() call
		const int count = ::wiringPiI2CReadReg16(this->_fd, 0);

		if(count < 0) {
			throw std::runtime_error("failed to read light level");
		}

		return static_cast<uint16_t>(count);

	}

	/**
	 * Measure lux from the sensor
	 * @return {double}  : 
	 */
	double lux() {

		this->measure();

		const std::chrono::duration waitTime = calculateWaitTime(
			this->_mtReg,
			this->_lastMeasurementMode,
			true);

		std::this_thread::sleep_for(waitTime);

		return convertLevel(
			this->readLevel(),
			this->_lastMeasurementMode,
			this->_mtReg);

	}

	void power_down() {

		if(this->_powerMode == PowerMode::POWER_OFF) {
			return;
		}

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(PowerMode::POWER_DOWN)) < 0) {
			throw std::runtime_error("device failed to power down");
		}

		this->_powerMode = PowerMode::POWER_DOWN;

	}

	void power_up() {

		if(this->_powerMode == PowerMode::POWER_ON) {
			return;
		}

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(PowerMode::POWER_ON)) < 0) {
			throw std::runtime_error("device failed to power on");
		}

		this->_powerMode = PowerMode::POWER_ON;

	}

};

constexpr std::chrono::milliseconds BH1750::TYP_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::TYP_LOW_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_LOW_RES_TIME;

};
#endif