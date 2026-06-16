# ataraxis-transport-layer-mc

Provides methods for establishing and maintaining bidirectional communication with PC clients over USB and UART serial
interfaces.

[![PlatformIO Registry](https://tinyurl.com/485rn6st)](https://tinyurl.com/mptfb9hb)
![C++](https://img.shields.io/badge/C%2B%2B-blue?logo=cplusplus&logoColor=white&labelColor=grey)
![Arduino](https://img.shields.io/badge/Arduino-blue?logo=Arduino&logoColor=white&labelColor=grey)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

___

## Detailed Description

This is the C++ implementation of the ataraxis-transport-layer (AXTL) library, designed to run on Arduino and Teensy
microcontrollers. It provides methods for bidirectionally communicating with a host-computer running the
[ataraxis-transport-layer-pc](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc) companion library written in
Python. The library abstracts all steps necessary to safely send and receive data over the USB and UART communication
interfaces. It is specifically designed to support time-critical applications, such as scientific experiments, and can
achieve microsecond communication speeds for modern microcontroller-PC hardware combinations. This library is part of
the [Ataraxis](https://github.com/Sun-Lab-NBB/ataraxis) framework for AI-assisted scientific hardware control.

___

## Features

- Supports all recent Arduino and Teensy architectures and platforms.
- Uses Consistent Overhead Byte Stuffing (COBS) to encode payloads during transmission.
- Supports Cyclic Redundancy Check (CRC) 8-, 16- and 32-bit polynomials to ensure data integrity during transmission.
- Allows fine-tuning all library components to support a wide range of application contexts.
- Has a [companion](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc) PC library written in Python.
- Apache 2.0 License.

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
- [Acknowledgments](#acknowledgments)

___

## Dependencies

- An IDE or Framework capable of uploading microcontroller software that supports 
  [Platformio](https://platformio.org/install). This library is explicitly designed to be uploaded via Platformio and 
  will likely not work with any other IDE or Framework.

***Note,*** developers should see the [Developers](#developers) section for information on installing additional
development dependencies.

___

## Installation

### Source

***Note,*** installation from source is ***highly discouraged*** for anyone who is not an active project developer.

1. Download this repository to the local machine using the preferred method, such as git-cloning. Use one of the
   [stable releases](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc/tags).
2. Unpack the downloaded tarball and move the 'src' header files (do not copy 'src/main.cpp', which is a
   development-only entry point) into the appropriate destination ('include,' 'src,' or 'libs') directory of the
   project that needs to use this library.
3. Add ```#include <transport_layer.h>``` to the top of the file(s) that need to access the library API.

### Platformio

1. Navigate to the project’s platformio.ini file and add the following line to the target environment specification:
   ```lib_deps = inkaros/ataraxis-transport-layer-mc@^3.0.0```.
2. Add ```#include <transport_layer.h>``` to the top of the file(s) that need to access the library API.

___

## Usage

### TransportLayer
The TransportLayer class provides the API for bidirectional communication over USB or UART serial interfaces. It 
ensures proper encoding and decoding of data packets using the Consistent Overhead Byte Stuffing (COBS) 
scheme and ensures transmitted packet integrity through the use of the Cyclic Redundancy Check (CRC) checksums.

#### Packet Anatomy
The TransportLayer class sends and receives data in the form of packets. Each packet adheres to the following general 
layout:

`[START] [PAYLOAD SIZE] [COBS OVERHEAD] [PAYLOAD (1 to 254 bytes)] [DELIMITER] [CRC CHECKSUM (1 to 4 bytes)]`

To optimize runtime efficiency, the class generates two buffers at compile time that store the incoming and outgoing 
data packets. The size of the buffers depends on the maximum expected incoming and outgoing payload sizes, defined 
at class instantiation. The buffers are allocated to at most accommodate the maximum expected payload sizes and the 
additional packet-related metadata. **The maximum possible memory footprint of the buffers is 524 bytes.**

Additionally, the class **reserves either 256, 512, or 1024 bytes** depending on the size of the CRC polynomial 
selected at class instantiation (8-bit, 16-bit, or 32-bit).

***Note,*** TransportLayer’s WriteData() and ReadData() methods ***exclusively*** work with the **PAYLOAD** region of 
each data buffer. End users can safely ignore all packet-related information and focus on working with transmitted and
received serialized payloads, as it is impossible to access and manipulate packet metadata via the public API.

#### Quickstart
This minimal example demonstrates how to use this library to send and receive data. It is designed to be used together
with the quickstart example of the [companion](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc#quickstart) 
library. See the [rx_tx_loop.cpp](./examples/rx_tx_loop.cpp) for the .cpp implementation of this example:

```
// Includes the core dependency for all Teensyduino projects.
#include <Arduino.h>

// Includes the TransportLayer header to access class API.
#include <transport_layer.h>

// Instantiates a new TransportLayer object. Most template and constructor arguments are set to use optimal default
// values for most host microcontrollers. Consult the ReadMe and the API documentation to learn about fine-tuning the
// TransportLayer's parameters to better match the intended use-case.
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
        // If the data is available, carries out the reception procedure: reads the received byte-stream, parses the
        // payload, and makes it available for reading.
        const bool data_received = tl_class.ReceiveData();

        // If the reception was successful, reads the data, assumed to contain serialized test objects. Note, this
        // example is intended to be used together with the example script from the ataraxis-transport-layer-pc library.
        if (data_received)
        {
            // Overwrites the memory of the placeholder objects with the received data. This gradually 'consumes' the
            // received payload, so the data must be read in the same order as it was written to the payload.
            tl_class.ReadData(test_scalar);
            tl_class.ReadData(test_array);
            tl_class.ReadData(test_struct);

            // The section below showcases sending the data to the PC. It re-transmits the same data in the same order
            // except for the test_scalar which is changed to a new value.
            test_scalar = 987654321;

            // Writes objects to the TransportLayer's transmission buffer, staging them to be sent with the next
            // SendData() command. Note, the objects are written in the order they will be read by the PC, as the method
            // automatically concatenates their data into a continuous payload byte-stream.
            tl_class.WriteData(test_scalar);
            tl_class.WriteData(test_array);
            tl_class.WriteData(test_struct);

            // Packages and sends the contents of the transmission buffer to the PC.
            tl_class.SendData();
        }
    }
}
```

#### Key Methods

##### Sending Data
There are two key methods associated with sending data to the PC:
- The `WriteData()` method serializes the input object and writes the resultant byte sequence to the 
  transmission buffer’s payload region. Each call appends the data to the end of the payload already stored in the 
  transmission buffer.
- The `SendData()` method encodes the payload stored in the transmission buffer into a packet using COBS, calculates
  and adds the CRC checksum to the encoded packet, and transmits the packet to the PC. At least one byte of data
  should be written to the transmission buffer via the WriteData() method before SendData() is called; transmitting an
  empty payload produces a packet that the receiver rejects.

The example below showcases the sequence of steps necessary to send the data to the PC and assumes TransportLayer 
'tl_class' was initialized following the steps in the [Quickstart](#quickstart) example:
```
// Generates the test array to simulate the payload.
const uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

// Writes the data into the instance's transmission buffer. The method returns 'true' if it is able to write the data 
// and 'false' otherwise.
bool write_status = tl_class.WriteData(test_array);

// Constructs and hands the packet to the communication interface to be transmitted to the PC.
tl_class.SendData();  // This method does not have expected failure states to evaluate, so it does not return anything.
```

***Note,*** the transmission buffer is reset when the data is transmitted or via the call to the 
`ResetTransmissionBuffer()` method. Resetting the transmission buffer discards all data stored in the buffer.

#### Receiving Data
There are three key methods associated with receiving data from the PC:
- The `Available()` method checks if the serial interface has received enough bytes to justify parsing the data.
- The `ReceiveData()` method reads the encoded packet from the byte-stream stored in Serial interface buffer, verifies 
  its integrity with the CRC checksum, and decodes the payload from the packet using COBS. If the packet was 
  successfully received and unpacked, this method returns True.
- The `ReadData()` method overwrites the memory (data) of the input object with the data extracted from the received 
  payload. To do so, the method reads and consumes the number of bytes necessary to 'fill' the object with data from 
  the payload. Following this procedure, the object stores the new value(s) that match the read data and the consumed
  bytes are discarded, meaning it is only possible to read the same data **once**.

The example below showcases the sequence of steps necessary to receive data from the PC and assumes TransportLayer
'tl_class' was initialized following the steps in the [Quickstart](#quickstart) example: 
```
// Generates the test array to which the received data will be written.
uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

// Blocks until the data is received from the PC
while (!tl_class.Available())
{
}

// Parses the received data. Note, this method internally calls 'Available', so it is safe to call ReceiveData()
// instead of Available() in the 'while' loop above without changing how this example behaves.
bool receive_status = tl_class.ReceiveData();  // Returns 'true' if the data was received and passed verification.

// Overwrites the test_array with the data received from the PC. The method returns 'true' if it is able to read the 
// data and 'false' otherwise.
bool read_status = tl_class.ReadData(test_array);
```

***Note,*** each call to the ReceiveData() method resets the instance’s reception buffer, discarding any potentially
unprocessed data.

___

## API Documentation

See the [API documentation](https://ataraxis-transport-layer-mc-api-docs.netlify.app/) for the detailed description of 
the methods and classes exposed by components of this library.

___

## Developers

This section provides installation, dependency, and build-system instructions for project developers.

### Installing the Project

1. Install [Platformio](https://platformio.org/install/integration) either as a standalone IDE or as an IDE plugin.
2. Download this repository to the local machine using the preferred method, such as git-cloning.
3. If the downloaded distribution is stored as a compressed archive, unpack it using the appropriate decompression tool.
4. ```cd``` to the root directory of the prepared project distribution.
5. Run ```pio project init ``` to initialize the project on the local machine. See 
   [Platformio API documentation](https://docs.platformio.org/en/latest/core/userguide/project/cmd_init.html) for 
   more details on initializing and configuring projects with platformio.
6. If using an IDE that does not natively support platformio integration, call the ```pio project metadata``` command 
   to generate the metadata to integrate the project with the IDE. Note; most mainstream IDEs do not require or benefit
   from this step.

***Warning!*** To build this library for a platform or architecture that is not explicitly supported, edit the 
platformio.ini file to include the desired configuration as a separate environment. This project comes preconfigured 
with support for `teensy 4.1`, `arduino due`, and `arduino mega (R3)` platforms.

### Additional Dependencies

In addition to installing Platformio and main project dependencies, install the following dependencies:

- [Tox](https://tox.wiki/en/4.15.0/user_guide.html) and [Doxygen](https://www.doxygen.nl/manual/install.html) to build 
  the API documentation for the project. Note; both dependencies have to be available on the local system’s path.

### Development Automation

Unlike other Ataraxis libraries, the automation for this library is primarily provided via the
[PlatformIO's command line interface](https://docs.platformio.org/en/latest/core/userguide/index.html). 
Additionally, this project uses [tox](https://tox.wiki/en/latest/user_guide.html) for certain automation tasks not 
directly covered by platformio, such as API documentation generation. Check the [tox.ini file](tox.ini) for details 
about the available pipelines and their implementation. Alternatively, call ```tox list``` from the root directory of 
the project to see the list of available tasks.

***Note,*** all pull requests for this project have to successfully complete the `tox`, `pio check`, and `pio test`
tasks before being submitted.

___

## Versioning

This project uses [semantic versioning](https://semver.org/). See the
[tags on this repository](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc/tags) for the available project
releases.

___

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))
- Jasmine Si

___

## License

This project is licensed under the Apache 2.0 License: see the [LICENSE](LICENSE) file for details.

___

## Acknowledgments

- All Sun lab [members](https://neuroai.github.io/sunlab/people) for providing the inspiration and comments during the
  development of this library.
- [PowerBroker2](https://github.com/PowerBroker2) and his 
  [SerialTransfer](https://github.com/PowerBroker2/SerialTransfer) for inspiring this library and serving as an example 
  and benchmark. Check the SerialTransfer project as a good alternative to this library with a non-overlapping 
  set of features.
- The creators of all other dependencies and projects listed in the [platformio.ini](platformio.ini) file.
