# Claude Code Instructions

## Session start behavior

At the beginning of each coding session, before making any code changes, you should build a comprehensive understanding
of the codebase by invoking the `/explore-codebase` skill.

This ensures you:
- Understand the project architecture before modifying code
- Follow existing patterns and conventions
- Do not introduce inconsistencies or break integrations

## Style guide compliance

You MUST invoke the appropriate style skill before performing ANY of the following tasks:

| Task                                   | Skill to invoke    |
|----------------------------------------|--------------------|
| Writing or modifying C++ code          | `/cpp-style`       |
| Writing or modifying README files      | `/readme-style`    |
| Writing or modifying Sphinx docs files | `/api-docs`        |
| Writing git commit messages            | `/commit`          |
| Writing or modifying skill files       | `/skill-design`    |

Each skill contains a verification checklist that you MUST complete before submitting any work. Failure to invoke the
appropriate skill results in style violations.

## Companion library synchronization

This library (`ataraxis-transport-layer-mc`) and its Python counterpart (`ataraxis-transport-layer-pc`) implement the
same serial communication protocol on opposite ends of the connection. Any change to the packet format, status codes,
COBS encoding, CRC computation, or buffer layout in this library MUST be synchronized with the corresponding change
in `ataraxis-transport-layer-pc`, and vice versa.

**Before modifying any protocol-level behavior, you MUST:**

1. **Identify the companion repository**: Check for a local copy at `../ataraxis-transport-layer-pc/`. If unavailable,
   use `gh api repos/Sun-Lab-NBB/ataraxis-transport-layer-pc` to access the remote repository.

2. **Review the corresponding implementation**: Read the Python source that implements the same functionality you are
   modifying. Verify that the current MC and PC implementations are in sync before making changes.

3. **Plan synchronized changes**: Document what must change in both libraries. Notify the user of the required
   companion changes so they can be applied together.

4. **Never modify protocol behavior unilaterally**: A change applied to only one side of the connection will cause
   communication failures. Both libraries must agree on start byte value, delimiter byte value, payload size
   constraints, COBS encoding/decoding logic, CRC polynomial and computation, packet structure and field ordering,
   and status code definitions.

**What requires synchronization:**
- Packet format fields (start byte, delimiter, payload size encoding, CRC postamble)
- COBS encoding/decoding algorithm
- CRC polynomial, initial value, final XOR value, and lookup table generation
- `kTransportStatusCodes` values and meanings
- Buffer size calculations and payload size constraints
- Data serialization byte ordering and type representations

**What does NOT require synchronization:**
- Platform-specific buffer size detection (`kSerialBufferSize` preprocessor logic)
- Test infrastructure (`StreamMock`, Unity test fixtures)
- Build system, documentation, and PlatformIO configuration
- Arduino/Teensy-specific timing and `elapsedMillis` usage

## Available skills

| Skill                    | Description                                                                    |
|--------------------------|--------------------------------------------------------------------------------|
| `/explore-codebase`      | Perform in-depth codebase exploration at session start                         |
| `/cpp-style`             | Apply Ataraxis framework C++ coding conventions (REQUIRED for all C++ changes) |
| `/readme-style`          | Apply Ataraxis framework README conventions (REQUIRED for README changes)      |
| `/api-docs`              | Apply Ataraxis framework API documentation conventions                         |
| `/commit`                | Draft Ataraxis framework style-compliant git commit messages                   |
| `/skill-design`          | Generate and verify skill files and CLAUDE.md project instructions             |

## Project context

This is **ataraxis-transport-layer-mc**, a header-only C++17 library for Arduino and Teensy microcontrollers that
provides bidirectional serial communication with PC clients over USB and UART interfaces. It is the microcontroller-side
counterpart to the companion Python library `ataraxis-transport-layer-pc`. The library targets time-critical scientific
applications with microsecond-level communication speeds, optimized for the
[Ataraxis](https://github.com/Sun-Lab-NBB/ataraxis) framework.

### Key areas

| Directory    | Purpose                                                       |
|--------------|---------------------------------------------------------------|
| `src/`       | Header-only library source code (5 headers + main.cpp)        |
| `test/`      | Unity test suite for all library components                   |
| `examples/`  | Quickstart example (`rx_tx_loop.cpp`)                         |
| `docs/`      | Sphinx + Breathe documentation source (consumes Doxygen XML)  |

### Architecture

- **TransportLayer** (`transport_layer.h`): Main user-facing class providing `SendData()` and `ReceiveData()` methods.
  Manages dual staging buffers (transmission and reception), packet construction with COBS encoding and CRC checksums,
  and multi-stage packet parsing with configurable reception timeout. Templated on CRC polynomial type and buffer sizes.
- **COBSProcessor** (`cobs_processor.h`): Static methods for in-place Consistent Overhead Byte Stuffing encoding and
  decoding. Internal component used by TransportLayer; not intended for direct use.
- **CRCProcessor** (`crc_processor.h`): Templated class for CRC-8/16/32 checksum computation using a precomputed
  256-entry lookup table. Supports configurable polynomial, initial value, and final XOR value.
- **Shared assets** (`axtlmc_shared_assets.h`): Namespace `axtlmc_shared_assets` containing `kTransportStatusCodes`
  enum, `kBufferLayout` struct with protocol constants, `PACKED_STRUCT` macro, and reimplemented type traits for
  Arduino Mega compatibility.
- **StreamMock** (`stream_mock.h`): Templated mock of the Arduino `Stream` interface with publicly exposed reception
  and transmission buffers. Enables full unit testing without hardware.

### Packet format

```
[START BYTE (129)] [PAYLOAD SIZE] [COBS OVERHEAD] [PAYLOAD (1-254 bytes)] [DELIMITER (0)] [CRC CHECKSUM]
```

### Core components

| Component               | File                     | Purpose                                                     |
|-------------------------|--------------------------|-------------------------------------------------------------|
| `TransportLayer`        | `transport_layer.h`      | Bidirectional serial communication with packet framing      |
| `COBSProcessor`         | `cobs_processor.h`       | In-place COBS encoding and decoding                         |
| `CRCProcessor`          | `crc_processor.h`        | CRC-8/16/32 lookup table generation and checksum validation |
| `kTransportStatusCodes` | `axtlmc_shared_assets.h` | Status codes for packet parsing and buffer operations       |
| `kBufferLayout`         | `axtlmc_shared_assets.h` | Protocol constants (start byte, delimiter, size limits)     |
| `StreamMock`            | `stream_mock.h`          | Mock Stream interface for hardware-free testing             |

### Key patterns

- **Header-only library**: All source lives in `.h` files under `src/`. The `main.cpp` is excluded from library
  export and serves as a development entry point.
- **Template-heavy design**: `TransportLayer` is templated on CRC polynomial type (`uint8_t`, `uint16_t`, `uint32_t`)
  and buffer sizes. `CRCProcessor` and `StreamMock` are also templated classes.
- **Platform-conditional compilation**: `kSerialBufferSize` is resolved at compile time via preprocessor macros
  detecting the target microcontroller architecture (Teensy, Arduino Due, Mega, etc.).
- **Static processing classes**: `COBSProcessor` uses only static methods with no instance state. `CRCProcessor`
  maintains a lookup table computed at construction.
- **Status code returns**: All operations return `kTransportStatusCodes` enum values rather than throwing exceptions,
  consistent with embedded C++ patterns.
- **`PACKED_STRUCT` macro**: Applied to user-defined structures for serialization, ensuring no padding bytes are
  inserted by the compiler.

### Dependencies

| Library       | Purpose                                                        | Platforms                |
|---------------|----------------------------------------------------------------|--------------------------|
| Arduino.h     | Core Arduino framework (Serial, Stream, types)                 | All                      |
| elapsedMillis | Non-blocking timer for reception timeout on non-Teensy boards  | atmelsam, atmelavr       |

### Build system

This is a PlatformIO library project. The `platformio.ini` defines three board environments:

| Environment | Board          | Platform   | Monitor speed |
|-------------|----------------|------------|---------------|
| `teensy41`  | Teensy 4.1     | teensy     | 115200        |
| `due`       | Arduino Due    | atmelsam   | 5250000       |
| `mega`      | Arduino Mega   | atmelavr   | 1000000       |

All environments use the Arduino framework, Unity test framework, and `-std=c++17` build flag.

### Development commands

```bash
pio run -e teensy41              # Build for Teensy 4.1
pio run -e due                   # Build for Arduino Due
pio run -e mega                  # Build for Arduino Mega
pio test -e teensy41             # Run Unity tests on Teensy 4.1
pio check -e teensy41            # Run static analysis
tox -e docs                      # Build Sphinx API documentation (Doxygen + Breathe)
```

### Workflow guidance

**Modifying TransportLayer:**

1. Review `src/transport_layer.h` for the current implementation
2. Understand the dual-buffer architecture and packet format (start byte, payload size, COBS overhead, payload,
   delimiter, CRC checksum)
3. All methods return `kTransportStatusCodes` — do not introduce exception-based error handling
4. Parameters must match the companion `ataraxis-transport-layer-pc` Python library exactly; mismatches cause
   unrecoverable packet corruption

**Modifying COBS or CRC processing:**

1. Review the corresponding processor header (`cobs_processor.h` or `crc_processor.h`)
2. These are internal components — their APIs are consumed only by `TransportLayer`
3. COBS enforces a 254-byte maximum payload size (protocol hard limit)
4. CRC lookup tables are computed at construction and persist for the instance lifetime (256-1024 bytes RAM)

**Adding platform support:**

1. Add a new preprocessor branch in `transport_layer.h` for the `kSerialBufferSize` detection
2. Add a corresponding environment in `platformio.ini`
3. Verify that the board's Arduino framework provides a compatible `Stream` interface

**Important considerations:**

- Maximum payload size is 254 bytes (COBS protocol hard limit)
- RAM budget: up to 524 bytes for staging buffers + up to 1024 bytes for CRC lookup table
- The `using namespace axtlmc_shared_assets;` in source files is intentional for readability in the embedded context
- The reimplemented type traits in `axtlmc_shared_assets.h` exist because Arduino Mega lacks the `<type_traits>` header
- `library.json` controls what gets exported to the PlatformIO registry — `main.cpp` is explicitly excluded
