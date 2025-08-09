# Arduino Library for BMP3XX sensor series

This library is for Bosch's BMP3XX series of pressure sensors. It is based on the Bosch's BMP390 datasheet and application notes. It is designed to be slightly more lightweight than the Adafruit BMP3XX library, and with minimal dependencies so it can be more easily integrated into other projects.

## Installation

Just download the repo as a zip and go to `Sketch / Include Library / Add .ZIP Library...` in the Arduino IDE. 

You can also clone the repo into your `libraries` folder.

Or add it to your `platformio.ini` file under `lib_deps`:

```ini
lib_deps =
    https://github.com/artichoked1/BMP3XX.git#1.0.0
```

## Usage
```cpp
// Initialise sensor
BMP3XX bmp;
bmp.configure(BMP3XX_OVERSAMPLING_4X, BMP3XX_OVERSAMPLING_4X, BMP3XX_IIR_FILTER_OFF, BMP3XX_ODR_50HZ);
bmp.begin();

// Read temp
float temp = bmp.readTemperature();

// Read pressure
float pressure = bmp.readPressure();
```

## Further notes

This library is designed to be as simple as possible, so it lacks more rigorous error checking and handling. It also does not currently include SPI support, but this could be added in the future.

There is also no altitude calculation, as I wrote this library for use in a weather monitoring project where I only need temperature and pressure data.

I have only tested this library with the BMP390 sensor, but it should theoretically work with the BMP388 and BMP390L sensors as well. If you have any issues, please let me know.