# BH1750

Use with Raspberry Pi.

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

Inspired by <https://github.com/claws/BH1750>
