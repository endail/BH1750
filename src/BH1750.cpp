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
#include "../include/Instruction.h"
#include <cmath>
#include <cstdint>
#include <lgpio.h>
#include <stdexcept>
#include <thread>

namespace BH1750 {

constexpr std::chrono::milliseconds BH1750::TYP_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_HIGH_RES_TIME;
constexpr std::chrono::milliseconds BH1750::TYP_LOW_RES_TIME;
constexpr std::chrono::milliseconds BH1750::MAX_LOW_RES_TIME;

bool BH1750::isHighRes(const MeasurementMode mode) noexcept {
    
    switch(mode) {
        case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
        case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
        case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
        case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
            return true;
        default:
            return false;
     }
    

}

bool BH1750::isLowRes(const MeasurementMode mode) noexcept {

    switch(mode) {
        case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
        case MeasurementMode::ONE_TIME_LOW_RES_MODE:
            return true;
        default:
            return false;
    }

}

bool BH1750::isMode2(const MeasurementMode mode) noexcept {

    switch(mode) {
        case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
        case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
            return true;
        default:
            return false;
    }

}

bool BH1750::isContinuous(const MeasurementMode mode) noexcept {

    switch(mode) {
        case MeasurementMode::CONTINUOUS_HIGH_RES_MODE:
        case MeasurementMode::CONTINUOUS_HIGH_RES_MODE_2:
        case MeasurementMode::CONTINUOUS_LOW_RES_MODE:
            return true;
        default:
            return false;
    }

}

bool BH1750::isOneTime(const MeasurementMode mode) noexcept {

    switch(mode) {
        case MeasurementMode::ONE_TIME_HIGH_RES_MODE:
        case MeasurementMode::ONE_TIME_HIGH_RES_MODE_2:
        case MeasurementMode::ONE_TIME_LOW_RES_MODE:
            return true;
        default:
            return false;
    }

}

std::chrono::milliseconds BH1750::calculateWaitTime(
    const std::uint8_t mt,
    const MeasurementMode mode,
    const bool maxWait) noexcept {

        std::size_t ms = std::round(static_cast<double>(mt) / static_cast<double>(TYP_MTREG));

        if(isHighRes(mode)) {
            ms = ms * (maxWait ? MAX_HIGH_RES_TIME.count() : TYP_HIGH_RES_TIME.count());
        }
        else {
            ms = ms * (maxWait ? MAX_LOW_RES_TIME.count() : TYP_LOW_RES_TIME.count());
        }

        return std::chrono::milliseconds(ms);

}

double BH1750::convertLevel(
    const std::uint16_t level,
    const MeasurementMode mode,
    const std::uint8_t mt,
    const float acc) noexcept {

        //algorithm on datasheet pg. 11

        double temp = 
            static_cast<double>(level)
            / acc
            * (static_cast<double>(TYP_MTREG) / static_cast<double>(mt));

        if(isHighRes(mode) && isMode2(mode)) {
            temp = temp / 2.0;
        }

        return temp;

}

void BH1750::_setMeasurementMode(const MeasurementMode mode) {

    if((::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(mode))) < 0) {
        throw std::runtime_error("failed to set measurement mode");
    }

    this->_measurementMode = mode;

}

void BH1750::_setMeasurementTimeRegister(const std::uint8_t mt) {

    if(!(mt >= MIN_MTREG && mt <= MAX_MTREG)) {
        throw std::range_error("reg must be between MIN_MTREG and MAX_MTREG");
    }

    //procedure for setting mt register on datasheet pg. 11
    //unsure if third I2C call is needed

    if(!(
        ::lgI2cWriteByte(this->_handle, 0b01000000 | (mt & 0b11100000)) == 0 &&
        ::lgI2cWriteByte(this->_handle, 0b01100000 | (mt & 0b00011111)) == 0 &&
        ::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(this->_measurementMode)) == 0
    )) {
        throw std::runtime_error("failed to set measurement register");
    }

    this->_mtReg = mt;

}

void BH1750::_setMeasurementAccuracy(const float acc) {

    if(acc < MIN_MEASUREMENT_ACCURACY || acc > MAX_MEASUREMENT_ACCURACY) {
        throw std::range_error("accuracy must be within MIN_MEASUREMENT_ACCURACY and MAX_MEASUREMENT_ACCURACY");
    }

    this->_accuracy = acc;

}

BH1750::BH1750(const int device, const int addr) noexcept :
    _dev(device),
    _addr(addr),
    _handle(-1),
    _powerMode(PowerMode::POWER_DOWN),
    _measurementMode(MeasurementMode::CONTINUOUS_HIGH_RES_MODE),
    _lastMeasurementMode(MeasurementMode::CONTINUOUS_HIGH_RES_MODE),
    _accuracy(TYP_MEASUREMENT_ACCURACY),
    _mtReg(TYP_MTREG) {
}

BH1750::~BH1750() {
    ::lgI2cClose(this->_handle);
}

MeasurementMode BH1750::getMeasurementMode() const noexcept {
    return this->_measurementMode;
}

MeasurementMode BH1750::getLastMeasurementMode() const noexcept {
    return this->_lastMeasurementMode;
}

std::uint8_t BH1750::getMeasurementTime() const noexcept {
    return this->_mtReg;
}

float BH1750::getMeasurementAccuracy() const noexcept {
    return this->_accuracy;
}

PowerMode BH1750::getPowerMode() const noexcept {
    return this->_powerMode;
}

void BH1750::connect(
    const MeasurementMode mm,
    const std::uint8_t mt,
    const float acc) {

        if(this->_handle >= 0) {
            throw std::runtime_error("sensor is already connected");
        }

        if((this->_handle = ::lgI2cOpen(this->_dev, this->_addr, 0)) < 0) {
            throw std::runtime_error("failed to connect to device");
        }

        this->_powerMode = PowerMode::POWER_UP;
        this->configure(mm, mt, acc);

}

void BH1750::measure() {

    if(::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(this->_measurementMode)) < 0) {
        throw std::runtime_error("failed to measure light level");
    }

    this->_lastMeasurementMode = this->_measurementMode;

    //One Time modes return the device to power down
    //after measurement. see: datasheet pg. 5
    if(isOneTime(this->_lastMeasurementMode)) {
        this->_powerMode = PowerMode::POWER_DOWN;
    }

}

void BH1750::reset() const {

    //"Reset Data register value. Reset command is not acceptable in
    //Power Down mode." See: datasheet pg. 5
    if(this->_powerMode == PowerMode::POWER_DOWN) {
        throw std::runtime_error("cannot reset while powered down");
    }

    if(::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(Instruction::RESET)) < 0) {
        throw std::runtime_error("failed to reset device registers");
    }

}

void BH1750::configure(
    const MeasurementMode mode,
    const std::uint8_t mt,
    const float acc) {
        this->_setMeasurementMode(mode);
        this->_setMeasurementTimeRegister(mt);
        this->_setMeasurementAccuracy(acc);
}

std::uint16_t BH1750::readLevel() const {

    //read from the 0th register
    const int word = ::lgI2cReadWordData(this->_handle, 0);

    if(word < 0) {
        throw std::runtime_error("failed to read light level");
    }

    return static_cast<std::uint16_t>(word);

}

double BH1750::lux() {

    this->measure();

    const auto waitTime = calculateWaitTime(
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

void BH1750::powerDown() {

    //do nothing if device is already powered down
    if(this->_powerMode == PowerMode::POWER_DOWN) {
        return;
    }

    if(::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(PowerMode::POWER_DOWN)) < 0) {
        throw std::runtime_error("device failed to power down");
    }

    this->_powerMode = PowerMode::POWER_DOWN;

}

void BH1750::powerUp() {

    //do nothing if device is already powered up
    if(this->_powerMode == PowerMode::POWER_UP) {
        return;
    }

    if(::lgI2cWriteByte(this->_handle, static_cast<std::uint8_t>(PowerMode::POWER_UP)) < 0) {
        throw std::runtime_error("device failed to power on");
    }

    this->_powerMode = PowerMode::POWER_UP;

}

};
