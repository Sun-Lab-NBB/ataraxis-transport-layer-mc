# ataraxis-micro-controller

A C++ library for Arduino and Teensy microcontrollers that integrates user-defined hardware modules with Python clients
running Ataraxis software.

![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![arduino](https://img.shields.io/badge/Arduino-00878F?logo=arduino&logoColor=fff&style=plastic)
![platformio](https://shields.io/badge/PlatformIO-E37B0D?style=flat&logo=platformio&logoColor=white)
![license](https://img.shields.io/badge/license-GPLv3-blue)
___

## Detailed Description

This library supports the concurrent operation of many unique user-defined hardware modules by providing communication 
and task scheduling services. In turn, this allows developers to focus on implementing the modules, instead of worrying 
about data transmission and control flow. This is achieved via the 2 main parts of the library: the TransportLayer class
and the microcontroller Core classes (Communication, Kernel, and Module).

The TransportLayer provides the serial protocol for bidirectionally communicating with a host-computer running the 
[ataraxis-transport-layer](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer) Python library. It is tuned 
for performance, achieving microsecond communication speeds while ensuring the integrity of transmitted payloads. While 
the class is originally designed to support the microcontroller Core classes, it can also be used as a standalone 
module for independent projects.

The microcontroller Core provides the framework that integrates user-defined hardware with Python clients. To do so, it 
defines a shared API that can be integrated into user-defined modules by subclassing the (base) Module class. It also 
provides the Kernel class that manages task scheduling during runtime, and the Communication class, which allows custom 
modules to communicate to Python clients via the TransportLayer binding.
___

## Features

- Supports Arduino and Teensy microcontrollers built on 
[ARM architecture](https://en.wikipedia.org/wiki/Atmel_ARM-based_processors#Products).
- Provides an easy-to-implement API that integrates any user-defined hardware with Python clients running Ataraxis
  software.
- Uses robust communication protocol that ensures data integrity via the Circular Redundancy Check (up to CRC-32) and 
  Consistent Overhead Byte Stuffing payload encoding.
- Abstracts communication and microcontroller runtime management through a set of Core classes that can be tuned to 
  optimize latency or throughput.
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
- An IDE or Framework capable of uploading microcontroller software. Use [Platformio](https://platformio.org/install) if
  you want to use the project the way it was built, tested, and used by the authors. Overall, it is highly encouraged to
  at least give this framework a try, as it greatly simplifies working with embedded projects.
  Alternatively, [Arduino IDE](https://www.arduino.cc/en/software) is also fully compatible with this project and
  satisfies this dependency, although its use is discouraged.

### Additional Dependencies
These dependencies will be automatically resolved whenever the library is installed via Platformio. ***They are 
mandatory for all other IDEs / Frameworks!***

- [digitalWriteFast](https://github.com/ArminJo/digitalWriteFast).
- [elapsedMillis](https://github.com/pfeerick/elapsedMillis/blob/master/elapsedMillis.h).
- [Encoder](https://github.com/PaulStoffregen/Encoder). This dependency is **optional** if you do not intend to use the 
  default EncoderModule class.

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
3. Move all remaining '[src](./src)' contents, required by your project, into the appropriate destination 
   ('include,' 'src' or 'libs') directory of your project.
4. Add header file inclusions to the files that need access to the classes from this library, e.g.: 
   ```include <module.h>```.

### Platformio

1. Navigate to your platformio.ini file and add the following line to your target environment specification:
   ```lib_deps = inkaros/ataraxis-micro-controller@^1.0.0```. If you already have lib_deps specification, add the 
   library specification to the existing list of used libraries.
2. Add ```include <ataraxis_micro_controller.h>``` to the top of the file(s) that need to access classes from this 
   library.
___

## Usage

### TransportLayer
The TransportLayer class provides an intermediate-level API for bidirectional communication over USB or UART interfaces. 
It ensures proper encoding and decoding of data packets using the COBS (Consistent Overhead Byte Stuffing) protocol for 
framing and CRC (Cyclic Redundancy Check) for integrity verification. Internal buffers are used to stage outgoing and 
incoming payloads.

#### Packet Anatomy:
This class sends and receives data in the form of packets. Each packet is expected to adhere to the following general 
layout: \
\
`[START] [PAYLOAD SIZE] [COBS OVERHEAD] [PAYLOAD (1 to 254 bytes)] [DELIMITER] [CRC CHECKSUM (1 to 4 bytes)]`

When using WriteData() and ReadData() methods, the users are only working with the payload section of the overall
packet. The rest of the packet anatomy is controlled internally by this class and is not exposed to the users.

#### Quickstart
This is a minimal example of how to use this library. See the [examples](./examples) folder for .cpp implementations:

```
// Note, this example should run on both Arduino and Teensy boards.

// First, include the main STP class to access its' API.
include <transport_layer.h>

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

### Communication

The Communication class provides a high-level interface for bidirectional communication between a microcontroller
(e.g., Arduino, Teensy) and a PC over a serial USB or UART port. To do so, it wraps the **TransportLayer** class and
uses it to serialize and transmit data using one of the predefined message layout protocols to determine payload
microstructure.

This class is tightly integrated with the Kernel and (base) Module classes, together forming the 'Core Triad' of the
AtaraxisMicroController library. Therefore, it is specifically optimized to transfer data between Kernel, Modules, and
their PC interfaces.

#### Prototypes
Prototypes are byte-codes packaged in Message packets that specify the data structure object that can be used to parse
the included object data.Currently, there are only six prototype codes supported, which define the types of data that can
be transmitted. All messages sent must conform to one of the supported prototype codes. The Prototype codes and the types 
of objects they encode are available from **communication_assets**. For prototype codes to work as expected, the microcontroller
and the PC need to share the same `prototype_code` to object mapping. \
Here’s an example of sending a message using a prototype code that specifies a  `TwoUnsignedBytes` object:
```
//TwoUnsignedBytes
uint8_t test_object[2] = {0}; 

const uint8_t module_type = 112;       // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const communication_assets::kPrototypes prototype = communication_assets::kPrototypes::kTwoUnsignedBytes;

comm_class.SendDataMessage(command, event_code, prototype, test_object);
```

#### Outgoing Message Structures

The Communication class supports sending a fixed number of predefined message payload structures, each optimized for
a specific use case.

Each message type is associated with a unique message protocol code, which is used to instruct the receiver on how to
parse the message. The Communication class automatically handles the parsing and serialization of these message formats.

- **ModuleData:** Communicates the event state-code of the sender Module and includes an additional data object.\
  Use `SendDataMessage` to send Module messages to the PC. This method is specialized to send Module messages. 

```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t module_type = 112        // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const uint8_t prototype = 1;           // Prototype for OneUnsignedByte. Protocol codes are available from
                                       // communication_assets namespace.
const uint8_t test_object = 1;  // OneUnsignedByte

comm.SendDataMessage(module_type, module_id, command, event_code, prototype, test_object );
```

- **KernelData:** Similar to ModuleData, but used for messages sent by the Kernel.
  There is an overloaded version of `SendDataMessage` that does not take `module_type` and `module_id` arguments,
  which allows sending data messages from the Kernel class.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code
const uint8_t prototype = 1;           // Prototype for OneUnsignedByte. Protocol codes are available from
                                       // communication_assets namespace.
const uint8_t test_object = 1;  // OneUnsignedByte

comm.SendDataMessage(command, event_code, prototype, test_object );
```

- **ModuleState:** Used for sending event state codes from modules without additional data.\
  Use `SendStateMessage` to send Module states to the PC. This method is specialized to send Module messages. It
  packages the input data into the ModuleState structure and sends it to the connected PC.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t module_type = 112        // Example module type
const uint8_t module_id = 12;          // Example module ID
const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code

comm_class.SendStateMessage(module_type, module_id, command, event_code);
```

- **KernelState:** Similar to ModuleState, but used for Kernel messages.\
  There is an overloaded version of  `SendStateMessage` that only takes `command` and `event_code` arguments, which
  allows sending data messages from the Kernel class.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

const uint8_t command = 88;            // Example command code
const uint8_t event_code = 221;        // Example event code

comm_class.SendStateMessage(command, event_code);
```

#### Incoming Message Structures
When receiving incoming messages, there are two key functions to keep in mind:
- `ExtractModuleParameters()` extracts the parameter data transmitted with the ModuleParameters message and uses it to set
the input structure values. This method will fail if it is called for any other message type, including KernelParameters 
message.
- The method `ReceiveMessage()` parses the next available message received from the PC and stored inside the serial port 
reception buffer. If the received message is a ModuleParameters message, call `ExtractModuleParameters()` method to
finalize message parsing since `ReceiveMessage()` DOES NOT extract Module parameter data from the received message.

```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

comm_class.ReceviveMessage(); 

struct DataMessage
{
uint8_t id = 1;
uint8_t data = 10;
} data_message;

bool success = comm_class.ExtractParameters(data_message);
```

#### Commands
This class supports various command types for controlling the behavior of modules and the Kernel. These commands
are sent through this class and are specified within different message types (e.g., **ModuleData**, **KernelData**). 
Each command typically contains a **command code**, which is a unique identifier for the operation to perform. Commands
can also include **return codes** to notify the sender that the command was received and processed successfully.

- **Module Commands**: These commands are sent to specific modules to perform certain actions. There are three main types:
    - **RepeatedModuleCommand**: A command that runs repeatedly or in cycles. It allows controlling module behavior
      on a timed interval.
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;

    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.repeated_module_command.module_type);  // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.repeated_module_command.module_id);      // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.repeated_module_command.return_code); // Extract return_code
    uint8_t command = static_cast<uint8_t>(comm_class.repeated_module_command.command);          // Extract command
    bool noblock = static_cast<uint8_t>(comm_class.repeated_module_command.noblock);             // Extract noblock
    uint32_t cycle_delay = static_cast<uint32_t>(comm_class.repeated_module_command.cycle_delay);// Extract cycle_delay
    ```

    - **OneOffModuleCommand**: A single execution command that instructs the addressed Module to run the specified command
      exactly once (non-recurrently).
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;
    
    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.one_off_module_command.module_type);  // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.one_off_module_command.module_id);      // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.one_off_module_command.return_code); // Extract return_code
    uint8_t command = static_cast<uint8_t>(comm_class.one_off_module_command.command);          // Extract command
    bool noblock = static_cast<bool>(comm_class.one_off_module_command.noblock);                // Extract noblock
    ```
    - **DequeModuleCommand**: A command that instructs the addressed Module to clear (empty) its command queue. Note that
      this does not terminate any active commands, and any active commands will eventually be allowed to finish.
    ```
    Communication comm_class(Serial);  // Instantiates the Communication class.
    Serial.begin(9600);  // Initializes serial interface.

    struct DataMessage
    {
    uint8_t id = 1;
    uint8_t data = 10;
    } data_message;
    
    comm_class.ReceviveMessage(); 
    comm_class.ExtractParameters(data_message);
    
    uint8_t module_type = static_cast<uint8_t>(comm_class.module_dequeue.module_type);    // Extract module_type
    uint8_t module_id = static_cast<uint8_t>(comm_class.module_dequeue.module_id);        // Extract module_id
    uint8_t return_code = static_cast<uint8_t>( comm_class.module_dequeue.return_code);   // Extract return_code
    ```

- **Kernel Commands**: These are commands sent to the Kernel to perform system-level operations. These commands are
  always one-off and execute immediately upon receipt.
```
Communication comm_class(Serial);  // Instantiates the Communication class.
Serial.begin(9600);  // Initializes serial interface.

struct DataMessage
{
uint8_t id = 1;
uint8_t data = 10;
} data_message;

comm_class.ReceviveMessage(); 
comm_class.ExtractParameters(data_message);

uint8_t return_code = static_cast<uint8_t>( comm_class.module_dequeue.return_code);   // Extract return_code
uint8_t command = static_cast<uint8_t>(comm_class.repeated_module_command.command);   // Extract command
```
___

## API Documentation

See the [API documentation](https://ataraxis-micro-controller-api-docs.netlify.app/) for the detailed description of 
the methods and classes exposed by components of this library. The API documentation includes the custom hardware 
modules shipped with the library.
___

## Developers

This section provides additional installation, dependency, and build-system instructions for the developers that want to
modify the source code of this library.

### Installing the library

1. If you do not already have it installed, install [Platformio](https://platformio.org/install/integration) either as
   a standalone IDE or as a plugin for your main C++ IDE. As part of this process, you may need to install a standalone
   version of [Python](https://www.python.org/downloads/). Note, for the rest of this guide, installing platformio CLI
   is
   sufficient.
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
- [Doxygen](https://www.doxygen.nl/manual/install.html), if you want to generate C++ code documentation. Note, if you
  are not installing Tox, you will also need [Breathe](https://breathe.readthedocs.io/en/latest/) and
  [Sphinx](https://docs.readthedocs.io/en/stable/intro/getting-started-with-sphinx.html).

### Development Automation

To assist developers, this project comes with a set of fully configured 'tox'-based pipelines for verifying and building
the project. Each of the tox commands builds the necessary project dependencies in the isolated environment prior to
carrying out its tasks.

Below is a list of all available commands and their purpose:

- ```tox -e test-ENVNAME``` Builds the project and executes the tests stored in the /test directory using 'Unity' test
  framework. Note, replace the ```ENVNAME``` with the name of the tested environment. By default, the tox is configured
  to
  run tests for 'teensy41,' 'mega' and 'due' platforms. To add different environments, edit the tox.ini file.
- ```tox -e docs``` Uses Doxygen, Breathe and Sphinx to build the source code documentation from Doxygen-formatted
  docstrings, rendering a static API .html file.
- ```tox -e build-ENVNAME``` Builds the project for the specified environment (platform). Does not upload the built hex
  file to the board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox -e upload-ENVNAME``` Builds the project for the specified environment (platform) and uploads it to the
  connected board. Same ```ENVNAME``` directions apply as to the 'test' command.
- ```tox --parallel``` Carries out all commands listed above in-parallel (where possible). Remove the '--parallel'
  argument to run the commands sequentially. Note, this command will test, build and upload the library for all
  development platforms, which currently includes: 'teensy41,' 'mega' and 'due.'

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- All Sun Lab [WEBSITE LINK STUB] members for providing the inspiration and comments during the development of this
  library.
- [PowerBroker2](https://github.com/PowerBroker2) and his 
[SerialTransfer](https://github.com/PowerBroker2/SerialTransfer) for inspiring this library and serving as an example 
and benchmark. Check SerialTransfer as a good alternative with non-overlapping functionality that may be better for your
project.
