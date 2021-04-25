# BH1750

Use with Raspberry Pi.

```cpp
BH1750::BH1750 sensor;
sensor.connect();
std::cout << sensor.lux();
```

Inspired by <https://github.com/claws/BH1750>
