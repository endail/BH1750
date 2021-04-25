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

#include "../include/BH1750.h"
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <wiringPiI2C.h>

namespace BH1750 {

bool BH1750::isHighRes(const MeasurementMode mode) noexcept {
	
	switch(mode) {
		case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
		case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
		case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
		case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
			return true;
	}

	return false;

}

bool BH1750::isLowRes(const MeasurementMode mode) noexcept {

	switch(mode) {
		case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
		case MeasurementMode::ONE_TIME_LOW_RES_MODE:
			return true;
	}

	return false;

}

bool BH1750::isMode2(const MeasurementMode mode) noexcept {

	switch(mode) {
		case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
		case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
			return true;
	}

	return false;

}

bool BH1750::isContinuous(const MeasurementMode mode) noexcept {

	switch(mode) {
		case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
		case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
		case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
			return true;
	}

	return false;

}

bool BH1750::isOneTime(const MeasurementMode mode) noexcept {

	switch(mode) {
		case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
		case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
		case MeasurementMode::ONE_TIME_LOW_RES_MODE:
			return true;
	}

	return false;

}

std::chrono::duration BH1750::calculateWaitTime(
	const uint8_t mt,
	const MeasurementMode mode,
	const bool maxWait = true) noexcept {

	using namespace std::chrono;

		const double ms = static_cast<double>(mt) / static_cast<double>(TYP_MTREG);
		
		if(isHighRes(mode)) {
			return milliseconds(ms * (maxWait ? MAX_HIGH_RES_TIME : TYP_HIGH_RES_TIME));
		}

		return milliseconds(ms * (maxWait ? MAX_LOW_RES_TIME : TYP_LOW_RES_TIME));

}

double BH1750::convertLevel(
	const uint16_t level,
	const MeasurementMode mode,
	const uint8_t mt,
	const float acc) noexcept {

		double temp = static_cast<double>(level);

		if(isHighRes(mode) && isMode2(mode)) {
			temp = (temp / acc) *
				(static_cast<double>(TYP_MTREG) / static_cast<double>(mt)) /
				2.0;
		}
		else {
			temp = temp / acc * 
				(static_cast<double>(TYP_MTREG) / static_cast<double>(mt));
		}

		return temp;

}

void BH1750::_setMeasurementMode(const MeasurementMode mode) {

	if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(mode)) < 0) {
		throw std::runtime_error("failed to set measurement mode");
	}

	this->_measurementMode = mode;

}

void BH1750::_setMeasurementTimeRegister(const uint8_t mt) {

	//https://www.mouser.com/datasheet/2/348/bh1750fvi-e-186247.pdf
	//pg. 11
	//range is between 31 (inc) and 254 (inc)
	if(!(mt >= MIN_MTREG && mt <= MAX_MTREG)) {
		throw std::range_error("reg must be between MIN_MTREG and MAX_MTREG");
	}

	//is the third I2C write call needed?
	if(	::wiringPiI2CWrite(this->_fd, 0b01000000 | (mt & 0b11100000)) < 0 ||
		::wiringPiI2CWrite(this->_fd, 0b01100000 | (mt & 0b00011111)) < 0 ||
		::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(this->_measurementMode)) < 0 ) {
			throw std::runtime_error("failed to set measurement register");
	}

	this->_mtReg = mt;

}

BH1750::_setMeasurementAccuracy(const float acc) {

	if(acc < MIN_MEASUREMENT_ACCURACY || acc > MAX_MEASUREMENT_ACCURACY) {
		throw range_error("accuracy must be within MIN_MEASUREMENT_ACCURACY and MAX_MEASUREMENT_ACCURACY");
	}

	this->_accuracy = acc;

}

BH1750::BH1750(
	const int8_t addr,
	const char* device) noexcept 
		: _dev(device), _addr(addr) {
}

BH1750::~BH1750() {
	::close(this->_fd);
}

MeasurementMode BH1750::getMeasurementMode() const noexcept {
	return this->_measurementMode;
}

MeasurementMode BH1750::getLastMeasurementMode() const noexcept {
	return this->_lastMeasurementMode;
}

uint8_t BH1750::getMeasurementTime() const noexcept {
	return this->_mtReg;
}

float BH1750::getMeasurementAccuracy() const noexcept {
	return this->_accuracy;
}

void BH1750::connect(
	const MeasurementMode mm,
	const uint8_t mt) {

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

void BH1750::measure() {

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

void BH1750::reset() const {

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

void BH1750::configure(
	const MeasurementMode mode,
	const uint8_t mt,
	const float acc) {
		this->_setMeasurementMode(mode);
		this->_setMeasurementTimeRegister(mt);
		this->_setMeasurementAccuracy(acc);
}

uint16_t BH1750::readLevel() const {

	//may need to replace this with two 8-bit reads
	//or lower-level 16bit read() call
	const int count = ::wiringPiI2CReadReg16(this->_fd, 0);

	if(count < 0) {
		throw std::runtime_error("failed to read light level");
	}

	return static_cast<uint16_t>(count);

}

double BH1750::lux() {

	this->measure();

	const std::chrono::duration waitTime = calculateWaitTime(
		this->_mtReg,
		this->_lastMeasurementMode,
		true);

	std::this_thread::sleep_for(waitTime);

	return convertLevel(
		this->readLevel(),
		this->_lastMeasurementMode,
		this->_mtReg,
		this->_accuracy);

}

void BH1750::power_down() {

	if(this->_powerMode == PowerMode::POWER_OFF) {
		return;
	}

	if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(PowerMode::POWER_DOWN)) < 0) {
		throw std::runtime_error("device failed to power down");
	}

	this->_powerMode = PowerMode::POWER_DOWN;

}

void BH1750::power_up() {

	if(this->_powerMode == PowerMode::POWER_ON) {
		return;
	}

	if(::wiringPiI2CWrite(this->_fd, static_cast<uint8_t>(PowerMode::POWER_ON)) < 0) {
		throw std::runtime_error("device failed to power on");
	}

	this->_powerMode = PowerMode::POWER_ON;

}

constexpr std::chrono::milliseconds BH1750::TYP_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::TYP_LOW_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_LOW_RES_TIME;

};