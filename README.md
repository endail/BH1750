# BH1750

[![Build on Raspberry Pi](https://github.com/endail/BH1750/actions/workflows/buildcheck.yml/badge.svg)](https://github.com/endail/BH1750/actions/workflows/buildcheck.yml) [![cppcheck](https://github.com/endail/BH1750/actions/workflows/cppcheck.yml/badge.svg)](https://github.com/endail/BH1750/actions/workflows/cppcheck.yml)

- Use with Raspberry Pi
- Requires [lgpio](http://abyz.me.uk/lg/index.html)
- Code tested inside [virtual Raspberry Pi Zero/3/4 environments](.github/workflows/buildcheck.yml) on GitHub

## Example

```cpp
#include "BH1750.h"
#include <iostream>

int main() {
  
  BH1750::BH1750 sensor;
  sensor.connect();
  
  std::cout << sensor.lux() << std::endl;
  
  return 0;

}
```

## Use

After writing your own code (eg. main.cpp), compile and link with the lgpio library as follows:

```console
pi@raspberrypi:~ $ g++ -Wall -o prog main.cpp -llgpio
```
