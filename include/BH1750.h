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
#include <wiringPiI2C.h>

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

	enum class Instruction : uint8_t {
		POWER_DOWN = 0b00000000,
		POWER_ON = 0b00000001,
		RESET = 0b00000111,
		CONTINUOUS_HIGH_RES_MODE = 0b00010000,
		CONTINUOUS_HIGH_RES_MODE_2 = 0b00010001,
		CONTINUOUS_LOW_RES_MODE = 0b00010011,
		ONE_TIME_HIGH_RES_MODE = 0b00100000,
		ONE_TIME_HIGH_RES_MODE_2 = 0b00100001,
		ONE_TIME_LOW_RES_MODE = 0b00100011
	};

protected:
	const char* _dev;
	const int8_t _addr;
	int _fd = -1;
	Instruction _mode = Instruction::POWER_DOWN;
	uint8_t _mtReg = TYP_MTREG;

public:

	BH1750(
		const int8_t addr = I2C_LOW_ADDR,
		const char* device = DEFAULT_I2C_DEVICE) noexcept 
			: _dev(device), _addr(addr) {
	}

	~BH1750() = default;

	const char* getDevice() const noexcept {
		return this->_dev;
	}

	int8_t getAddress() const noexcept {
		return this->_addr;
	}

	int getFileDescriptor() const noexcept {
		return this->_fd;
	}

	Instruction getMode() const noexcept {
		return this->_mode;
	}

	uint8_t getMeasurementTimeRegister() const noexcept {
		return this->_mtReg;
	}

	bool isHighRes() const noexcept {
		
		switch(this->_mode) {
			case Instruction::CONTINUOUS_HIGH_RES_MODE:
			case Instruction::CONTINUOUS_HIGH_RES_MODE_2:
			case Instruction::ONE_TIME_HIGH_RES_MODE:
			case Instruction::ONE_TIME_HIGH_RES_MODE_2:
				return true;
		}

		return false;

	}

	bool isLowRes() const noexcept {

		switch(this->_mode) {
			case Instruction::CONTINUOUS_LOW_RES_MODE:
			case Instruction::ONE_TIME_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	bool isMode2() const noexcept {

		switch(this->_mode) {
			case Instruction::CONTINUOUS_HIGH_RES_MODE_2:
			case Instruction::ONE_TIME_HIGH_RES_MODE_2:
				return true;
		}

		return false;

	}

	bool isContinuous() const noexcept {

		switch(this->_mode) {
			case Instruction::CONTINUOUS_HIGH_RES_MODE:
			case Instruction::CONTINUOUS_HIGH_RES_MODE_2:
			case Instruction::CONTINUOUS_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	bool isOneTime() const noexcept {

		switch(this->_mode) {
			case Instruction::ONE_TIME_HIGH_RES_MODE:
			case Instruction::ONE_TIME_HIGH_RES_MODE_2:
			case Instruction::ONE_TIME_LOW_RES_MODE:
				return true;
		}

		return false;

	}

	void begin(const Instruction mode = Instruction::CONTINUOUS_HIGH_RES_MODE) {

		if((this->_fd = ::wiringPiI2CSetupInterface(this->_dev, this->_addr)) < 0) {
			throw std::runtime_error("failed to connect to device");
		}

		this->configure(mode);
		this->setMeasurementRegister(TYP_MTREG);

	}

	void configure(const Instruction mode = Instruction::CONTINUOUS_HIGH_RES_MODE) {

		switch(mode) {
			case Instruction::CONTINUOUS_HIGH_RES_MODE:
			case Instruction::CONTINUOUS_HIGH_RES_MODE_2:
			case Instruction::CONTINUOUS_LOW_RES_MODE:
			case Instruction::ONE_TIME_HIGH_RES_MODE:
			case Instruction::ONE_TIME_HIGH_RES_MODE_2:
			case Instruction::ONE_TIME_LOW_RES_MODE:
				break;
			default:
				throw std::runtime_error("mode is not permitted");
		}

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(mode)) < 0) {
			throw std::runtime_error("failed to configure device");
		}

		this->_mode = mode;

	}

	void measure() {
		
		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(this->_mode)) < 0) {
			throw std::runtime_error("failed to measure light level");
		}

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 5
		//One Time modes return the device to power down mode after
		//measurement
		if(this->isOneTime()) {
			this->_mode = Instruction::POWER_DOWN;
		}

	}

	std::chrono::milliseconds waitTime(const bool maxWait = true) const noexcept {

		using namespace std::chrono;

		const double ms = static_cast<double>(this->_mtReg) / static_cast<double>(TYP_MTREG);
		
		if(this->isHighRes()) {
			return milliseconds(ms * maxWait ? MAX_HIGH_RES_TIME : TYP_HIGH_RES_TIME);
		}

		return milliseconds(ms * maxWait ? MAX_LOW_RES_TIME : TYP_LOW_RES_TIME);

	}

	void wait() const noexcept {
		std::this_thread::sleep_for(this->waitTime());
	}

	void reset() {

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 5
		//"Reset Data register value. Reset command is not acceptable in
		//Power Down mode."
		if(this->_mode == Instruction::POWER_DOWN) {
			throw std::runtime_error("cannot reset while powered down");
		}

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(Instruction::RESET)) < 0) {
			throw std::runtime_error("failed to reset device registers");
		}

	}

	void setMeasurementRegister(const uint8_t reg) noexcept {

		//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
		//pg. 11
		//range is between 31 (inc) and 254 (inc)
		if(!(reg >= MIN_MTREG && reg <= MAX_MTREG)) {
			throw std::range_error("reg must be between 31 and 254");
		}

		if(	::wiringPiI2CWrite(this->_fd, 0b01000000 | (reg & 0b11100000)) < 0 ||
			::wiringPiI2CWrite(this->_fd, 0b01100000 | (reg & 0b00011111)) < 0 ||
			::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(this->_mode)) < 0 ) {
				throw std::runtime_error("failed to set measurement register");
		}

		this->_mtReg = reg;

	}

	uint16_t readLevel() {

		//may need to replace this with two 8-bit reads
		//or lower-level 16bit read() call
		const int count = ::wiringPiI2CReadReg16(this->_fd, 0);

		if(count < 0) {
			throw std::runtime_error("failed to read light level");
		}

		return static_cast<uint16_t>(count);

	}

	double convertLevel(const uint16_t level) const noexcept {

		double temp = static_cast<double>(level);

		if(this->isHighRes()) {

			//all high res modes
			temp = temp * static_cast<double>(TYP_MTREG) /
				static_cast<double>(this->_mtReg);

			//only for high res and mode 2
			if(this->isMode2()) {
				temp = temp / 2;
			}

		}

		//all modes
		return temp * TYP_MEASUREMENT_ACCURACY;

	}

	double lux() {

		if(this->_mode == Instruction::POWER_DOWN) {
			throw std::runtime_error("device is not powered");
		}

		this->reset(); //is reset necessary?
		this->measure();
		this->wait();

		return this->convertLevel(this->readLevel());

	}

	void power_down() {

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(Instruction::POWER_DOWN)) < 0) {
			throw std::runtime_error("device failed to power down");
		}

		this->_mode = Instruction::POWER_DOWN;

	}

	void power_up() {

		if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(Instruction::POWER_ON)) < 0) {
			throw std::runtime_error("device failed to power on");
		}

		this->_mode = Instruction::POWER_ON;

	}

};

constexpr std::chrono::milliseconds BH1750::TYP_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::TYP_LOW_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_LOW_RES_TIME;

};
#endif