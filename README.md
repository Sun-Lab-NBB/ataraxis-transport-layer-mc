# ataraxis-transport-layer-mc

A C++ library for Arduino and Teensy microcontrollers that provides methods for establishing and maintaining 
bidirectional communication with PC clients over USB or UART serial interfaces.

![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![platformio](https://shields.io/badge/PlatformIO-E37B0D?style=flat&logo=platformio&logoColor=white)
![license](https://img.shields.io/badge/license-GPLv3-blue)
___

## Detailed Description
This is a C++ implementation of the ataraxis-transport-layer (AXTL) library, designed to run on Arduino or Teensy 
microcontrollers. It provides methods for bidirectionally communicating with a host-computer running the 
[ataraxis-transport-layer-pc](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer) companion library written in 
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

1. Download this repository to your local machine using your preferred method, such as Git-cloning. Optionally, use one
   of the stable releases from [GitHub](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller).
2. Remove the '[main.cpp](./src/main.cpp)' file from the 'src' directory of the project.
3. Move all remaining '[src](./src)' contents into the appropriate destination ('include,' 'src' or 'libs') directory 
   of your project.
4. Add ```include <transport_layer.h>``` to the top of the file(s) that need to access classes from this library.

### Platformio

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/ataraxis-transport-layer-mc@^1.0.0```. If you already have lib_deps specification, add the 
   library specification to the existing list of used libraries.
2. Add ```include <transport_layer.h>``` to the top of the file(s) that need to access classes from this library.
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
This is a minimal example of how to use this library. See the [main.cpp](./src/main.cpp) for .cpp implementation:

```
// Note, this example should run on both Arduino and Teensy boards.

// First, include the main STP class to access its' API.
#include <transport_layer.h>

// Maximum outgoing payload size, in bytes. Cannot exceed 254 bytes due to COBS encoding.
uint8_t maximum_tx_payload_size = 254;

// Maximum incoming payload size, in bytes. Cannot exceed 254 bytes.
uint8_t maximum_rx_payload_size = 200;

// The minimal incoming payload size.
uint8_t minimum_payload_size = 1;

// These parameters jointly specify the CRC algorithm to be used fro the CRC calcualtion. The class automatically scales
// to work for 8-, 16- and 32-bit algorithms.
uint16_t polynomial = 0x1021;
uint16_t initial_value = 0xFFFF;
uint16_t final_xor_value = 0x0000;

// The value used to mark the beginning of transmitted packets in the byte-stream. 
uint8_t start_byte = 129;

// The value used to mark the end of the main packet section.
uint8_t delimiter_byte = 0;

// The number of microseconds that can separate the reception of any two consecutive bytes of the same packet.
uint32_t timeout = 20000; // In microseconds

// The reference to the Serial interface class. 
serial = Serial;

// Instantiates a new TransportLayer object.
TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size, minimum_payload_size> tl_class(
    serial,
    polynomial,
    initial_value,
    final_xor_value,
    start_byte,
    delimiter_byte,
    timeout
);

void setup()
{
    Serial.begin(115200);  // Opens the Serial port, initiating PC communication
}


void loop()
{
    // Checks if data is available for reception.
    if (tl_class.Available())
    {
        // If the data is available, carries out the reception procedure (acually receives the byte-stream, parses the 
        // payload and makes it available for reading).
        bool data_received = tl_class.ReceiveData();
        
        // Provided that the reception was successful, reads the data, assumed to be the test array object
        if (data_received)
        {
            // Instantiates a simple test object
            uint8_t test_data[4] = {0, 0, 0, 0};
            
            // Reads the received data into the array object. 
            uint16_t next_index = tl_class.ReadData(test_data);
            
            // Instantiates a new object to send back to PC.
            uint8_t send_data[4] = {5, 6, 7, 8};
            
            // Writes the object to the transmission buffer, staging it to be sent witht he next SendData() command. 
            uint16_t add_index = tl_class.WriteData(send_data, 0);
            
            // This showcases a chained addition, where test_data is staged right after send_data.
            add_index = tl_class.WriteData(test_data, add_index);
            
            // Packages and sends the contents of the class transmission buffer that were written above to the PC.
            bool data_sent= tl_class.SendData();  // This also returns a boolean status.
        }
    }
}
```
#### Key Methods

##### Sending Data
When sending data, there are two key methods: `WriteData()` and `SendData()`. 
- The `WriteData()` method writes the input object as bytes into the `_transmission_buffer` payload region starting
  at the specified `start_index`.
- The `SendData()` method encodes the payload using COBS, calculates a CRC checksum, and transmits the contents of
  the transmission buffer as a serialized packet. To ensure correct data is sent, you must first populate the
  `_transmission_buffer` using `WriteData()`. Otherwise, `SendData()` will transmit whatever data is currently stored
  in the buffer.

```
// Generates the test array to be packaged and 'sent'
uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

// Writes the package into the _transmission_buffer
protocol.WriteData(test_array, 0);

// Sends the payload to the Stream buffer. If all steps of this process succeed, the method returns 'true'.
bool sent_status = protocol.SendData();
```

#### Receiving Data
When receiving data, there are three key methods:
- `Available()` checks if the reception buffer has enough bytes to justify reading. Returns True if there are bytes
  to be read from the transmission interface reception buffer, and False otherwise.
- `ReceiveData()` reads the incoming packet verifies it with CRC, and decodes it using COBS. If the packet was 
successfully received and unpacked, this method returns True, and returns False otherwise.
- `ReadData()` extracts the payload from the _reception_buffer. Reads the `requested_bytes` number of bytes from the 
`_reception_buffer` payload region starting at the `start_index` into the provided object.

```
if (tl_class.Available()) {
  tl_class.ReceiveData();
}
uint16_t value = 44321;
uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
struct MyStruct
{
  uint8_t a = 60;
  uint16_t b = 12345;
  uint32_t c = 1234567890;
} test_structure;

// Overwrites the test objects with the data stored inside the buffer
uint16_t next_index = tl_class.ReadData(value);
uint16_t next_index = tl_class.ReadData(array, next_index);
uint16_t next_index = tl_class.ReadData(test_structure, next_index);
```
___

## API Documentation

See the [API documentation](https://ataraxis-transport-layer-api-docs.netlify.app/) for the detailed description of 
the methods and classes exposed by components of this library.
___

## Developers

This section provides additional installation, dependency, and build-system instructions for the developers that want to
modify the source code of this library.

### Installing the library

1. If you do not already have it installed, install [Platformio](https://platformio.org/install/integration) either as
   a standalone IDE or as a plugin for your main C++ IDE. As part of this process, you may need to install a standalone
   version of [Python](https://www.python.org/downloads/). Note, for the rest of this guide, installing platformio CLI
   is enough.
2. Download this repository to your local machine using your preferred method, such as git-cloning.
3. ```cd``` to the root directory of the project using your CLI of choice.
4. Run ```pio --target upload -e ENVNAME``` command to compile and upload the library to your microcontroller. Replace
   the ```ENVNAME``` with the environment name for your board. Currently, the project is preconfigured to work for
   ```mega```, ```teensy41``` and ```due``` environments.
5. Optionally, run ```pio test -e ENVNAME``` command using the appropriate environment to test the library on your
   target platform

Note, if you are developing for a board that the project is not explicitly configured for, you will first need to edit
the platformio.ini file to support your target microcontroller by configuring a new environment.

### Additional Dependencies

In addition to installing platformio and main project dependencies, additionally install the following dependencies:

- [Tox](https://tox.wiki/en/4.15.0/user_guide.html), if you intend to use preconfigured tox-based project automation.
- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation. ***Note***, if 
  you chose not to install Tox, you will also need [Breathe](https://breathe.readthedocs.io/en/latest/) and
  [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html).

### Development Automation

To help developers, this project comes with a set of fully configured 'tox'-based pipelines for verifying and building
the project. Each of the tox commands builds the necessary project dependencies in the isolated environment before
carrying out its tasks.

Below is a list of all available commands and their purpose:

- ```tox -e test-ENVNAME``` Builds the project and executes the tests stored in the /test directory using 'Unity' test
  framework. Note, replace the ```ENVNAME``` with the name of the tested environment. By default, the tox is configured
  to
  run tests for 'teensy41,' 'mega' and 'due' platforms. To add different environments, edit the tox.ini file.
- ```tox -e docs``` Uses Doxygen, Breathe, and Sphinx to build the source code documentation from Doxygen-formatted
  docstrings, rendering a static API .html file.
- ```tox -e build-ENVNAME``` Builds the project for the specified environment (platform). Does not upload the built hex
  file to the board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox -e upload-ENVNAME``` Builds the project for the specified environment (platform) and uploads it to the
  connected board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox --parallel``` Carries out all commands listed above in-parallel (where possible). Remove the '--parallel'
  argument to run the commands sequentially. Note, this command will test, build and upload the library for all
  development platforms, which currently includes: 'teensy41,' 'mega' and 'due.'
---

## Versioning

We use [semantic versioning](https://semver.org/) for this project. For the versions available, see the
[tags on this repository](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc/tags).

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
