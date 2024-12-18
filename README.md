# ataraxis-transport-layer-mc

A C++ library for Arduino and Teensy microcontrollers that provides methods for establishing and maintaining 
bidirectional communication with PC clients over USB or UART serial interfaces.

[![PlatformIO Registry](https://tinyurl.com/485rn6st)](https://tinyurl.com/mptfb9hb)
![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![license](https://img.shields.io/badge/license-GPLv3-blue)

___

## Detailed Description
This is a C++ implementation of the ataraxis-transport-layer (AXTL) library, designed to run on Arduino or Teensy 
microcontrollers. It provides methods for bidirectionally communicating with a host-computer running the 
[ataraxis-transport-layer-pc](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc) companion library written in 
Python. The library abstracts most steps necessary for data transmission, such as serializing data into payloads, 
packing the payloads into packets, and transmitting packets as byte-streams to the receiver. It also abstracts the 
reverse sequence of steps necessary to verify and decode the payload from the packet received as a stream of bytes. The 
library is specifically designed to support time-critical applications, such as scientific experiments, and can achieve 
microsecond communication speeds for newer microcontroller-PC configurations.
___

## Features

- Supports all recent Arduino and Teensy architectures and platforms.
- Uses Consistent Overhead Byte Stuffing (COBS) to encode payloads.
- Supports Circular Redundancy Check (CRC) 8-, 16- and 32-bit polynomials to ensure data integrity during transmission.
- Fully configurable through Constructor and Template parameters.
- Contains many sanity checks performed at compile time to reduce the potential for data corruption or loss in transit.
- Has a [companion](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc) libray written in Python to simplify 
  PC-MicroController communication.
- GPL 3 License.
___

## Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Usage](#usage)
- [API Documentation](#api-documentation)
- [Developers](#developers)
- [Versioning](#versioning)
- [Authors](#authors)
- [License](#license)
- [Acknowledgements](#Acknowledgments)
___

## Dependencies

### Main Dependency
- An IDE or Framework capable of uploading microcontroller software. This library is designed to be used with
  [Platformio,](https://platformio.org/install) and we strongly encourage using this IDE for Arduino / Teensy 
  development. Alternatively, [Arduino IDE](https://www.arduino.cc/en/software) also satisfies this dependency, but 
  is not officially supported or recommended for most users.

### Additional Dependencies
These dependencies will be automatically resolved whenever the library is installed via Platformio. ***They are 
mandatory for all other IDEs / Frameworks!***

- [elapsedMillis](https://github.com/pfeerick/elapsedMillis/blob/master/elapsedMillis.h)

For developers, see the [Developers](#developers) section for information on installing additional development 
dependencies.
___

## Installation

### Source

Note, installation from source is ***highly discouraged*** for everyone who is not an active project developer.
Developers should see the [Developers](#Developers) section for more details on installing from source. The instructions
below assume you are ***not*** a developer.

1. Download this repository to your local machine using your preferred method, such as Git-cloning. Use one
   of the stable releases from [GitHub](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc/releases).
2. Unpack the downloaded tarball and move all 'src' contents into the appropriate destination 
   ('include,' 'src' or 'libs') directory of your project.
3. Add ```include <transport_layer.h>``` to the top of the file(s) that need to access the library API.

### Platformio

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/ataraxis-transport-layer-mc@^1.0.1```. If you already have lib_deps specification, add the 
   library specification to the existing list of used libraries.
2. Add ```include <transport_layer.h>``` to the top of the file(s) that need to access the library API.
___

## Usage

### TransportLayer
The TransportLayer class provides an intermediate-level API for bidirectional communication over USB or UART serial 
interfaces. It ensures proper encoding and decoding of data packets using the Consistent Overhead Byte Stuffing (COBS) 
protocol and ensures transmitted packet integrity via Cyclic Redundancy Check (CRC).

#### Packet Anatomy:
This class sends and receives data in the form of packets. Each packet adheres to the following general 
layout:

`[START] [PAYLOAD SIZE] [COBS OVERHEAD] [PAYLOAD (1 to 254 bytes)] [DELIMITER] [CRC CHECKSUM (1 to 4 bytes)]`

To optimize runtime efficiency, the class generates two buffers at compile time that store the incoming and outgoing 
data packets. TransportLayer’s WriteData() and ReadData() methods work with data ***exclusively*** from the region of 
the buffer allocated to store the PAYLOAD bytes. The rest of the packet data is intentionally not accessible via the 
public API. Therefore, users can safely ignore all packet-related information and focus on working with transmitted and
received serialized payloads.

#### Quickstart
This is a minimal example of how to use this library. See the [rx_tx_loop.cpp](./examples/rx_tx_loop.cpp) for 
.cpp implementation:

```
// Includes the core dependency for all Teensyduino projects.
#include <Arduino.h>

// Includes the TransportLayer header to access class API.
#include <transport_layer.h>

// Instantiates a new TransportLayer object. Most template and constructor arguments should automatically scale with
// your microcontroller. Check the API documentation website if you want to fine-tune class parameters to better match
// your use case.
TransportLayer<> tl_class(Serial);  // NOLINT(*-interfaces-global-init)

void setup()
{
    Serial.begin(115200);  // Opens the Serial port, initiating PC communication
}

// Pre-creates the objects used for the demonstration below.
uint32_t test_scalar  = 0;
uint8_t test_array[4] = {0, 0, 0, 0};

struct TestStruct
{
        bool test_flag   = true;
        float test_float = 6.66;
} __attribute__((packed)) test_struct;  // Critically, the structure HAS to be packed!

void loop()
{
    // Checks if data is available for reception.
    if (tl_class.Available())
    {
        // If the data is available, carries out the reception procedure (reads the received byte-stream, parses the
        // payload, and makes it available for reading).
        const bool data_received = tl_class.ReceiveData();

        // If the reception was successful, reads the data, assumed to be the test array object. Note, this example
        // is intended to be used together with the example script from the ataraxis-transport-layer-pc library.
        if (data_received)
        {
            // Overwrites the memory of placeholder objects with the received data.
            uint16_t next_index = 0;  // Starts reading from the beginning of the payload region.
            next_index          = tl_class.ReadData(test_scalar, next_index);
            next_index          = tl_class.ReadData(test_array, next_index);
            // Since test_struct is the last object in the payload, we do not need to save the new next_index.
            tl_class.ReadData(test_struct, next_index);

            // Now the placeholder objects are updated with the values transmitted from the PC. The section below
            // showcases sending the data to the PC. It re-transmits the same data in the same order except
            // for the test_scalar which is changed to a new value.
            test_scalar = 987654321;

            // Writes objects to the TransportLayer's transmission buffer, staging them to be sent with the next
            // SendData() command. Note, the objects are written in the order they will be read by the PC.
            next_index = 0;  // Resets the index to 0.
            next_index = tl_class.WriteData(test_scalar, next_index);
            next_index = tl_class.WriteData(test_array, next_index);
            tl_class.WriteData(test_struct, next_index);  // Once again, the index after last object is not saved.

            // Packages and sends the contents of the transmission buffer that were written above to the PC.
            tl_class.SendData();  // This also returns a boolean status that we discard for this example.
        }
    }
}
```
#### Key Methods

##### Sending Data
There are two key methods associated with sending data to the PC:
- The `WriteData()` method serializes the input object into bytes and writes the resultant byte sequence into 
  the `_transmission_buffer` payload region starting at the specified `start_index`.
- The `SendData()` method encodes the payload into a packet using COBS, calculates the CRC checksum for the encoded 
  packet, and transmits the packet and the CRC checksum to PC. The method requires that at least one byte of data is 
  written to the staging buffer via the WriteData() method before it can be sent to the PC.

The example below showcases the sequence of steps necessary to send the data to the PC and assumes TransportLayer 
'tl_class' was initialized following the steps in the [Quickstart](#quickstart) example:
```
// Generates the test array to simulate the payload.
uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

// Writes the data into the _transmission_buffer.
tl_class.WriteData(test_array, 0);

// Sends the payload to the Stream buffer. If all steps of this process succeed, the method returns 'true' and the data
// is handed off to the 
bool sent_status = tl_class.SendData();
```

#### Receiving Data
There are three key methods associated with receiving data from the PC:
- The `Available()` method checks if the serial interface has received enough bytes to justify parsing the data. If this
  method returns False, calling ReceiveData() will likely fail.
- The `ReceiveData()` method reads the encoded packet from the byte-stream stored in Serial interface buffer, verifies 
  its integrity with CRC, and decodes the payload from the packet using COBS. If the packet was successfully received 
  and unpacked, this method returns True.
- The `ReadData()` method overwrites the memory (data) of the input object with the data extracted from the received 
  payload. To do so, the method reads the number of bytes necessary to 'fill' the object with data from the payload, 
  starting at the `start_index`. Following this procedure, the object will have new value(s) that match the read 
  data.

The example below showcases the sequence of steps necessary to receive data from the PC and assumes TransportLayer
'tl_class' was initialized following the steps in the [Quickstart](#quickstart) example: 
```
// Packages and sends the contents of the transmission buffer that were written above to the PC.
tl_class.SendData();  //

if (tl_class.Available())
{
    tl_class.ReceiveData();
}
uint16_t value    = 44321;
uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

struct MyStruct
{
        uint8_t a  = 60;
        uint16_t b = 12345;
        uint32_t c = 1234567890;
} test_structure;

// Overwrites the test objects with the data stored inside the buffer
uint16_t next_index = tl_class.ReadData(value);  // ReadData defaults to start_index 0 if it is not provided
uint16_t next_index = tl_class.ReadData(array, next_index);
uint16_t next_index = tl_class.ReadData(test_structure, next_index);
```
___

## API Documentation

See the [API documentation](https://ataraxis-transport-layer-mc-api-docs.netlify.app/) for the detailed description of 
the methods and classes exposed by components of this library.
___

## Developers

This section provides installation, dependency, and build-system instructions for the developers that want to
modify the source code of this library.

### Installing the library

1. If you do not already have it installed, install [Platformio](https://platformio.org/install/integration) either as
   a standalone IDE or as a plugin for your main C++ IDE. As part of this process, you may need to install a standalone
   version of [Python](https://www.python.org/downloads/).
2. Download this repository to your local machine using your preferred method, such as git-cloning.
3. ```cd``` to the root directory of the project using your command line interface of choice. Make sure the root 
   contains the `platformio.ini` file.
4. Run ```pio project init ``` to initialize the project on your local machine. Provide additional flags to this command
   as needed to properly configure the project for your specific needs. See 
   [Platformio API documentation](https://docs.platformio.org/en/latest/core/userguide/project/cmd_init.html) for 
   supported flags.
5. Optionally, use ```pio project metadata``` to dump the metadata that integrate the newly initialized project with
   your C++ IDE. Note, not all IDEs are supported, and not all IDEs need this step.

***Warning!*** If you are developing for a platform or architecture that the project is not explicitly configured for, 
you will first need to edit the platformio.ini file to support your target microcontroller by configuring a new 
environment. This project comes preconfigured with `teensy 4.1`, `arduino due` and `arduino mega (R3)` support.

### Additional Dependencies

In addition to installing platformio and main project dependencies, install the following dependencies:

- [Tox](https://tox.wiki/en/4.15.0/user_guide.html), if you intend to use preconfigured tox-based project automation.
  Currently, this is used only to build API documentation from source code docstrings.
- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation.

### Development Automation

Unlike other Ataraxis libraries, the automation for this library is primarily provided via 
[Platformio’s command line interface (cli)](https://docs.platformio.org/en/latest/core/userguide/index.html) core. 
Additionally, we also use [tox](https://tox.wiki/en/latest/user_guide.html) for certain automation tasks not directly 
covered by platformio, such as API documentation generation. Check [tox.ini file](tox.ini) for details about
available pipelines and their implementation. Alternatively, call ```tox list``` from the root directory of the project
to see the list of available tasks.

**Note!** All pull requests for this project have to successfully complete the `tox`, `pio check` and `pio test` tasks 
before being submitted.
---

## Versioning

We use [semantic versioning](https://semver.org/) for this project. For the versions available, see the
[tags on this repository](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc/tags).

---

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))
- Jasmine Si
---

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- All [Sun Lab](https://neuroai.github.io/sunlab/) members for providing the inspiration and comments during the 
  development of this library.
- [PowerBroker2](https://github.com/PowerBroker2) and his 
[SerialTransfer](https://github.com/PowerBroker2/SerialTransfer) for inspiring this library and serving as an example 
and benchmark. Check SerialTransfer as a good alternative with non-overlapping functionality that may be better for your
project.
---
