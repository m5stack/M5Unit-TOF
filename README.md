# M5Unit - TOF

## Overview

Library for ToF using [M5UnitUnified](https://github.com/m5stack/M5UnitUnified).  
M5UnitUnified is a library for unified handling of various M5 units products.

### SKU:U010

ToF that employs time-of-flight techniques to resolve distance between the emit point and the reach point of a subject, measuring the round trip time of an artificial light signal provided by a laser.

This unit integrated a distance measuring sensor VL53L0x providing accurate distance measurement whatever the target reflectance, unlike conventional technologies. It can measure absolute distances up to 2m in less than 30ms.

### SKU:U172

ToF4M Unit is a high-precision distance sensor module that utilizes the VL53L1CXV0FY/1 sensor and employs Time-of-Flight (ToF) technology. It is capable of measuring object height, detecting object presence, and tracking object movement within a range of 4mm to 4000mm. It finds applications in robotics, autonomous vehicles, drones, security systems, and more.

ToF4M Unit offers exceptional accuracy, with measurements as precise as 0.1mm. This level of accuracy enables it to detect even the smallest distance variations and capture surface details of objects. With its fast response time, ToF4M Unit can perform measurements within milliseconds. This real-time feedback of distance information makes it suitable for scenarios that require quick responsiveness.

ToF4M Unit communicates using the I2C protocol, providing advantages such as simplified connectivity, support for multi-master and multi-slave topologies, and high reliability. It can be easily integrated with existing sensor buses, effectively saving IO resources. The module features a standard Grove interface, ensuring reliable and stable signal transmission. When used with the M5Stack hardware ecosystem, it supports plug-and-play functionality, enabling rapid project development.

### SKU:U072

ToF HAT is a high precision laser-ranging sensor specifically designed for M5StickC. Integrated with VL53L0X and 940nm VCSEL emitter. It can provide high precision and low latency performance on object distance detection. 

### SKU:U196

The Unit Mini ToF-90° is an ultra-compact Time of Flight (ToF) ranging unit integrating the VL53L0X laser ranging module. 
By rotating the laser emission module by 90°, it achieves horizontal forward detection with a 25° field of view. 


## Related Link

See also examples using conventional methods here.

- [Unit ToF & Datasheet](https://docs.m5stack.com/en/unit/TOF)
- [Unit ToF4M & Datasheet](https://docs.m5stack.com/en/unit/Unit-ToF4M)
- [Hat ToF & Datasheet](https://docs.m5stack.com/en/hat/hat-tof)
- [Unit Mini ToF-90 & Datasheet](https://docs.m5stack.com/en/unit/Unit%20Mini%20ToF-90)


## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-TOF - MIT](LICENSE)

## Support via [PaHub](https://docs.m5stack.com/en/unit/Unit-PaHub%20v2.1)

|Unit|Support|Note|
|---|---|---|
|UnitToF|OK||
|UnitToF4M|OK||
|UnitToF90|OK||

UnitToF / UnitToF4M / UnitToF90 all use the same address (0x29), so connect them to different PaHub channels to use them together.
See also [examples/UnitUnified/ViaPaHub](examples/UnitUnified/ViaPaHub) and [M5Unit-HUB](https://github.com/m5stack/M5Unit-HUB)

## Examples
See also [examples/UnitUnified](examples/UnitUnified)

### For ArduinoIDE settings
You must choose a define symbol for the unit you will use.
(Rewrite source or specify with compile options)

- PlotToSerial
```cpp
// *************************************************************
// Choose one define symbol to match the unit you are using
// *************************************************************
#if !defined(USING_UNIT_TOF) && !defined(USING_UNIT_TOF4M) && !defined(USING_HAT_TOF) && !defined(USING_UNIT_TOF90)
// For UnitToF
// #define USING_UNIT_TOF
// For UnitToF4M
// #define USING_UNIT_TOF4M
// For HatToF
// #define USING_HAT_TOF
// For UnitToF90
// #define USING_UNIT_TOF90
#endif
```

### For ESP-IDF settings

> **NOTE:** The ESP-IDF native build (`idf.py`) targets ESP-IDF **5.1 or later** (5.x and 6.x).

On ESP-IDF native builds (`idf.py`), the unit for PlotToSerial is selected via Kconfig instead of editing the source `#define`. `PlotToSerial/main/Kconfig.projbuild` sources `examples/UnitUnified/common/Kconfig.variant` (UnitToF / UnitToF4M / UnitToF90 / HatToF), and `examples/UnitUnified/common/variant.cmake` maps the chosen `CONFIG_EXAMPLE_USING_*` to the source-level macro shared with the Arduino build, so the example source itself does not need to be edited. The **HatToF** option appears only when the target is esp32 / esp32s3 / esp32c6 (the chips of the boards with a Hat header).

Pick the unit with `menuconfig`:

```sh
cd examples/UnitUnified/PlotToSerial
idf.py set-target esp32s3                          # or esp32 / esp32c6 / esp32p4 / ...
idf.py menuconfig
# -> M5Unit-TOF PlotToSerial example -> Target unit -> choose ONE
idf.py build flash monitor
```

The other examples (DualSensor, ViaPaHub) have no variant; run `idf.py set-target <chip>` and `idf.py build flash monitor` directly.

## Doxygen document
[GitHub Pages](https://m5stack.github.io/M5Unit-TOF/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxygen](https://www.doxygen.nl/)
- [Git](https://git-scm.com/) (Output commit hash to html)
