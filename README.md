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


## Related Link

See also examples using conventional methods here.

- [Unit ToF & Datasheet](https://docs.m5stack.com/en/unit/TOF)
- [Unit ToF4M & Datasheet](https://docs.m5stack.com/en/unit/Unit-ToF4M)


## Required Libraries:

- [M5UnitUnified](https://github.com/m5stack/M5UnitUnified)
- [M5Utility](https://github.com/m5stack/M5Utility)
- [M5HAL](https://github.com/m5stack/M5HAL)

## License

- [M5Unit-TOF- MIT](LICENSE)


## Examples
See also [examples/UnitUnified](examples/UnitUnified)

## Doxygen document
[GitHub Pages](https://m5stack.github.io/M5Unit-TOF/)

If you want to generate documents on your local machine, execute the following command

```
bash docs/doxy.sh
```

It will output it under docs/html  
If you want to output Git commit hashes to html, do it for the git cloned folder.

### Required
- [Doxyegn](https://www.doxygen.nl/)
- [pcregrep](https://formulae.brew.sh/formula/pcre2)
- [Git](https://git-scm.com/) (Output commit hash to html)

