#include "../include/BH1750.h"
#include <thread>
#include <iostream>
#include <chrono>
#include <iomanip>

int main() {

	BH1750::BH1750 sensor;
	sensor.connect();

	while(true) {
		std::cout << std::fixed << std::setprecision(2) << sensor.lux() << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	return 0;

}