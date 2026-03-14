# Description

The **Home Monitoring Device** is a remote sensor dedicated to indoor applications. It offers the following features:

* **Temperature** and **humidity** measurements.
* **Air quality** monitoring.
* **Movement or orientation detection** with accelerometer.
* **Basic HMI** (RGB LED and button) to check battery voltage.
* **Sub-GHz radio** to send data over long range IoT networks such as Sigfox.

# Hardware

The board was designed on **Circuit Maker V2.0**. Below is the list of hardware revisions:

| Hardware revision | Description | `cmake_hw_version` | Status |
|:---:|:---:|:---:|:---:|
| [HMD HW1.0](https://365.altium.com/files/21353EBB-5412-4C6F-A8E0-7281AC274F13) | Initial version. | `HW1_0` | :x: |
| [HMD HW2.0](https://365.altium.com/files/B495104F-B4DE-4F3F-935E-B6F2A86241CF) | Improved radio front-end, additional sensors power configuration and lower consumption with ENS161.  | `HW2_0` | :white_check_mark: |

# Embedded software

## Environment

The embedded software is developed under **Eclipse IDE** version 2024-09 (4.33.0) and **GNU MCU** plugin. The `script` folder contains Eclipse run/debug configuration files and **JLink** scripts to flash the MCU.

## Target

The board is based on the **STM32L051K8U6** microcontroller of the STMicroelectronics L0 family. Each hardware revision has a corresponding **build configuration** in the Eclipse project, which sets up the code for the selected board version.

## Structure

The project is organized as follow:

* `drivers` :
    * `device` : MCU **startup** code and **linker** script.
    * `registers` : MCU **registers** address definition.
    * `peripherals` : internal MCU **peripherals** drivers.
    * `components` : external **components** drivers.
    * `utils` : **utility** functions.
* `middleware` :
    * `analog` : High level **analog measurements** driver.
    * `cli` : **AT commands** implementation.
    * `power` : Board **power tree** manager.
    * `sigfox` : **Sigfox EP_LIB** and **ADDON_RFP** submodules and low level implementation.
* `application` : Main **application**.

## Build

The project can be compiled by command line with `cmake`.

```bash
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="script/cmake-arm-none-eabi/toolchain.cmake" \
      -DTOOLCHAIN_PATH="<arm_none_eabi_gcc_path>" \
      -DHMD_HW_VERSION="<cmake_hw_version>" \
      -DHMD_MODE_CLI=OFF \
      -DHMD_BUTTON_ENABLE=ON \
      -DHMD_TEMPERATURE_HUMIDITY_SHT3X_ENABLE=ON \
      -DHMD_TEMPERATURE_HUMIDITY_ENS21X_ENABLE=ON \
      -DHMD_AIR_QUALITY_ENABLE=ON \
      -DHMD_ACCELEROMETER_ENABLE=ON \
      -G "Unix Makefiles" ..
make all
```
