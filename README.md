SparkFun BMA400 Arduino Library
========================================
<table class="table table-striped table-bordered"?
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/21208"><img src="https://cdn.sparkfun.com/assets/parts/2/0/9/7/1/21208_SEN-_01.jpg" alt="SparkFun Triple Axis Accelerometer Breakout - BMA400 (Qwiic)"></a></td>
    <td><a href="https://www.sparkfun.com/products/21207"><img src="https://cdn.sparkfun.com/assets/parts/2/0/9/7/0/21207_SEN-_01.jpg" alt="SparkFun Micro Triple Axis Accelerometer Breakout - BMA400 (Qwiic)"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/21208"><i>SparkFun Triple Axis Accelerometer Breakout - BMA400 (Qwiic)</i></a></td>
    <td><a href="https://www.sparkfun.com/products/21207"><i>SparkFun Micro Triple Axis Accelerometer Breakout - BMA400 (Qwiic)</i></a></td>
  </tr>
</table>

This library provides an easy way to control the BMA400 3-axis accelerometer. Each axis has 12-bit resolution, and supports ranges of 2g up to 16g. It also includes a number of interrupt features, including motion detection, orientation detection, tap detection, and step counting. The sensor also includes filtering, and an on-board FIFO buffer can be used to store measurements for more efficient burst reading of data.

This library implements [Bosch's BMA400 API](https://github.com/BoschSensortec/BMA400-API) in an Arduino-friendly way. All functions return error codes to indicate the result of each operation, where BMA400_OK (`0`) indicates success. Most examples ignore the error codes to reduce clutter, but an example is included to demonstrate error handling techniques.

## Repository Contents
* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** - General library properties for the Arduino package manager.

## Documentation
* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[Hardware Repo](https://github.com/sparkfun/SparkFun_Qwiic_Accelerometer_BMA400)** - Repository for the BMA400 board.
* **[Library](https://github.com/sparkfun/SparkFun_BMA400_Arduino_Library)** - This library, providing functions to write applications for the BMA400 with Arduino IDE.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/sparkfun-triple-axis-accelerometer-breakout---bma400-qwiic-hookup-guide)** - Basic hookup guide for the BMA400.
* **[LICENSE.md](./LICENSE.md)** - License Information

## Product Versions
* [SEN-21208](https://www.sparkfun.com/products/21208) - Standard Size Initial Release.
* [SEN-21207](https://www.sparkfun.com/products/21207) - Micro Size Initial Release.

## Version History

* [v1.0.0](https://github.com/sparkfun/SparkFun_BMA400_Arduino_Library/releases/tag/v1.0.0) - Initial public release.

## License Information

This product is _**open source**_! 

Please review the LICENSE.md file for license information. 

If you have any questions or concerns on licensing, please contact technical support on our [SparkFun forums](https://forum.sparkfun.com/viewforum.php?f=152).

Distributed as-is; no warranty is given.

- Your friends at SparkFun.

_<COLLABORATION CREDIT>_
