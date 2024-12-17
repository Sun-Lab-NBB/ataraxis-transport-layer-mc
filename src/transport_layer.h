/**
 * @file
 * @brief The header file for the TransportLayer class, which aggregates all intermediate-level methods for sending and
 * receiving serialized data over a USB / UART connection.
 *
 * @section tl_description Description:
 * This class provides an intermediate-level API that enables receiving and sending data over the USB or UART serial
 * ports. It conducts all necessary steps to properly encode and decode payloads, verifies their integrity and moves
 * them to and from the transmission interface buffers. This class instantiates private _transmission_buffer and
 * _reception_buffer arrays, which are used as the intermediate storage / staging areas for the processed payloads.
 * Both buffers are cleared during data transmission (for _transmission_buffer) and reception (for _reception_buffer).
 *
 * This class contains the following methods:
 * - Available(): To check whether the serial interface has received data that can be moved to the reception_buffer.
 * - ResetTransmissionBuffer(): To reset the tracker variables of the transmission_buffer.
 * - ResetReceptionBuffer(): To reset the tracker variables of the reception_buffer.
 * - SendData(): To package and send the contents of the transmission_buffer via the transmission interface.
 * - ReceiveData(): To unpack the data received by the transmission interface into the reception_buffer.
 * - WriteData(): To write an arbitrary object to the transmission_buffer as bytes.
 * - ReadData(): To read an arbitrary object from the reception_buffer as bytes.
 * - CopyTxDataToBuffer() and CopyRXDataToBuffer: To help testing by copying the contents of the _transmission_buffer
 * or the _reception_buffer, into the input buffer. Indirectly exposes the buffers for testing purposes.
 * - CopyTxBufferPayloadToRxBuffer: To help testing by copying the payload bytes inside the _transmission_buffer to
 * the _reception_buffer. This method is a safe way of simulating data reception from data written to
 * _transmission_buffer (via WriteData() method).
 * - Multiple minor methods that are used by the listed major methods and some accessor methods that communicate
 * the values of private tracker variables (such as the payload-size trackers for the class buffers).
 *
 * @attention This class is implemented as a template, and many methods adapt to the template arguments used during
 * class instantiation. See developer notes below for more information.
 *
 * @section tl_developer_notes Developer Notes:
 * This class exposes the main library API, relying on CRCProcessor and COBSProcessor helper-classes to cary out the
 * specific packet processing and integrity verification steps. The TransportLayer class abstracts these two
 * classes and a transmission interface instance by providing an API that can be used to carry out serial communication
 * through 4 major methods (SendData(), ReceiveData(), WriteData() and ReadData()).
 *
 * The class uses template parameters to allow users to pseudo-dynamically configure various class runtime aspects,
 * usually in combination with Constructor arguments. This approach complicates class instantiation, but, since
 * this optimization simplifies code maintainability, it is considered an acceptable tradeoff.
 *
 * @attention This class permanently reserves ~550 bytes (depending on the postamble size and support variables) for the
 * internal buffers. This value can be reduced by changing the maximum transmission / reception buffer sizes (by
 * altering kMaximumTransmittedPayloadSize and kMaximumReceivedPayloadSize template parameters). It is generally
 * encouraged to adjust the size to ensure the buffer at full capacity with the added preamble / postamble bytes fits
 * into the transmission interface buffers, as this often allows for more efficient data transmission.
 *
 * @section tl_packet_anatomy Packet Anatomy:
 * This class sends and receives data in the form of packets. Each packet is expected to adhere to the following general
 * layout:
 * [START] [PAYLOAD SIZE] [COBS OVERHEAD] [PAYLOAD (1 to 254 bytes)] [DELIMITER] [CRC CHECKSUM (1 to 4 bytes)]
 *
 * When using WriteData() and ReadData() methods, the users are only working with the payload section of the overall
 * packet. The rest of the packet anatomy is controlled internally by this class and is not exposed to the users.
 *
 * @section tl_dependencies Dependencies:
 * - Arduino.h for Arduino platform methods and macros and cross-compatibility with Arduino IDE (to an extent).
 * - cobs_processor.h for COBS encoding and decoding methods.
 * - crc_processor.h for CRC calculation method, as well as crc-specific buffer manipulation methods.
 * - elapsedMillis.h for managing packet reception timers.
 * - shared_assets.h for shared library assets (status byte-codes for the class and is_same_v for static guards).
 */

#ifndef AXMC_TRANSPORT_LAYER_H
#define AXMC_TRANSPORT_LAYER_H

// Dependencies
#include <Arduino.h>
#include <elapsedMillis.h>
#include "axtlmc_shared_assets.h"
#include "cobs_processor.h"
#include "crc_processor.h"

// Statically defines the size of the Serial class reception buffer associated with different Arduino and Teensy board
// architectures. This is required to ensure the TransportLayer class is configured appropriately. If you need to adjust
// the TransportLayer class buffers (for example, because you manually increased the buffer size used by the Serial
// class of your board), do it by editing or specifying a new preprocessor directive below. It is HIGHLY advised not to
// tamper with these settings, however, and to always have the kSerialBufferSize set exactly to the size of the Serial
// class reception buffer.
#if defined(ARDUINO_ARCH_SAM)
// Arduino Due (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_SAMD)
// Arduino Zero, MKR series (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

#elif defined(ARDUINO_ARCH_NRF52)
// Arduino Nano 33 BLE (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;

// Note, teensies are identified based on the processor model. This would need to be updated for future versions of
// Teensy boards.
#elif defined(CORE_TEENSY)
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__IMXRT1062__)
// Teensy 3.x, 4.x (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 1024;
#else
// Teensy 2.0, Teensy++ 2.0 (USB serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 256;
#endif

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__)
// Arduino Uno, Mega, and other AVR-based boards (UART serial) maximum reception buffer size in bytes.
static constexpr uint16_t kSerialBufferSize = 64;

#else
// Default fallback for unsupported boards is the reasonable minimum buffer size
static constexpr uint16_t kSerialBufferSize = 64;

#endif

/**
 * @class TransportLayer
 * @brief Exposes methods that can be used to send and receive serialized data over the USB or UART transmission
 * interface.
 *
 * This class wraps other low-level helper classes of the library that are used to encode, validate, and bidirectionally
 * transmit arbitrary data over the USB or UART interface. To facilitate this process, the class provides two internal
 * buffers: _transmission_buffer (stages the data to be transmitted) and _reception_buffer (stores the received data).
 * Both buffers are treated as temporary storage areas and are reset by each SendData() and ReceiveData() call.
 *
 * @warning Since the buffers follow a very specific layout pattern required for this class to work properly,
 * they are stored as private members of this class. The buffers can be manipulated using ReadData() and WriteData()
 * methods to read received data and write the data to be transmitted. They can also be reset at any time by calling
 * ResetTransmissionBuffer() and ResetReceptionBuffer() respectively. Additionally, GetTransmissionBufferBytes() and
 * GetReceptionBufferBytes() can be used to retrieve the number of bytes currently stored inside each of the buffers.
 *
 * @attention The class tracks how many bytes are stored in each of the buffers. Specifically for the
 * _transmission_buffer, this is critical, as the tracker determines how many bytes are packed and transmitted. The
 * tracker is reset by calling ResetTransmissionBuffer() or SendData() methods and is only incremented when the payload
 * size is increased (overwriting already counted bytes does not increment the counter). For example, if you add 50
 * bytes to the buffer and then overwrite the first 20, the class will remember and send all 50 bytes unless you reset
 * the tracker before overwriting the bytes. Additionally, the tracker always assumes that the bytes to send stretch
 * from the beginning of the buffer. So, if you write 10 bytes to the middle of the buffer (say, at index 100+), the
 * tracker will assume that 100 bytes were added before the 10 bytes you provided and send 110 bytes, including
 * potentially meaningless 100 bytes. See the documentation for the WriteData() and ReadData() methods for more
 * information on byte trackers.
 *
 * @note The class is broadly configured through the combination of class template parameters and constructor arguments.
 * The template parameters (see below) need to be defined at compilation time and are necessary to support proper static
 * initialization of local arrays and subclasses. All currently used template parameters indirectly control how much RAM
 * is reserved by the class for its buffers and the CRC lookup table (via the local CRCProcessor class instance). The
 * constructor arguments allow further configuring class runtime behavior in a way that does not mandate compile-time
 * definition.
 *
 * @tparam PolynomialType The datatype of the CRC polynomial to be used by the local CRCProcessor class instance.
 * Valid types are uint8_t, uint16_t, and uint32_t. The class contains a compile-time guard against any other input
 * datatype. See CRCProcessor documentation for more details.
 * @tparam kMaximumTransmittedPayloadSize The maximum size of the payload that is expected to be transmitted during
 * class runtime. Note, the number is capped to 254 bytes due to COBS protocol, and it is used to determine the size of
 * the _transmission_buffer array. Use this number to indirectly control the memory reserved by the _transmission_buffer
 * (at a maximum of 254 + 8 = 262 bytes).
 * @tparam kMaximumReceivedPayloadSize The maximum size of the payload that is expected to be received during class
 * runtime. Works the same way as kMaximumTransmittedPayloadSize, but allows to independently control the size of the
 * _reception_buffer.
 * @tparam kMinimumPayloadSize The minimum expected payload size (in bytes) for each incoming packet. This variable
 * is used to calculate the minimum number of bytes that has to be stored inside the transmission interface
 * reception buffer to trigger a reception procedure (and for the class Available() method to return 'true'). This
 * maximizes the chances of each ReceiveData() method call to return a valid packet. This optimization is intended
 * for some platforms to save CPU time by not wasting it on running reception attempts that are unlikely to succeed.
 * Note, this value has to be between 1 and 254.
 *
 * Example instantiation:
 * @code
 * Serial.begin(9600);
 *
 * uint8_t maximum_tx_payload_size = 254;
 * uint8_t maximum_rx_payload_size = 200;
 * uint16_t polynomial = 0x1021;
 * uint16_t initial_value = 0xFFFF;
 * uint16_t final_xor_value = 0x0000;
 * uint8_t start_byte = 129;
 * uint8_t delimiter_byte = 0;
 * uint8_t minimum_payload_size = 1;
 * uint32_t timeout = 20000; // In microseconds
 *
 * // Instantiates a new TransportLayer object
 * TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size, minimum_payload_size>
 * tl_class(
 *  Serial,
 *  polynomial,
 *  initial_value,
 *  final_xor_value,
 *  start_byte,
 *  delimiter_byte,
 *  timeout
 * );
 * @endcode
 */
template <
    typename PolynomialType                      = uint8_t,                          // Defaults to uint8_t polynomials
    const uint8_t kMaximumTransmittedPayloadSize = min(kSerialBufferSize - 8, 254),  // Intelligently caps at 254 bytes
    const uint8_t kMaximumReceivedPayloadSize    = min(kSerialBufferSize - 8, 254),  // Intelligently caps at 254 bytes
    const uint8_t kMinimumPayloadSize            = 1>  // 1 essentially means no cap
class TransportLayer
{
        // Ensures that the class only accepts uint8, 16 or 32 as valid CRC types, as no other type can be used to
        // store the CRC polynomial at the time of writing.
        static_assert(
            axtlmc_shared_assets::is_same_v<PolynomialType, uint8_t> ||
                axtlmc_shared_assets::is_same_v<PolynomialType, uint16_t> ||
                axtlmc_shared_assets::is_same_v<PolynomialType, uint32_t>,
            "TransportLayer class PolynomialType template parameter must be either uint8_t, uint16_t, or "
            "uint32_t."
        );

        // Verifies that the maximum Transmitted and Received payload sizes do not exceed 254 bytes (due to COBS, this
        // is the maximum supported size).
        static_assert(
            kMaximumTransmittedPayloadSize < 255,
            "TransportLayer class kMaximumTransmittedPayloadSize template parameter must be less than 255."
        );
        static_assert(
            kMaximumReceivedPayloadSize < 255,
            "TransportLayer class kMaximumReceivedPayloadSize template parameter must be less than 255."
        );

        // Verifies that the minimum payload size is within the valid range of values.
        static_assert(
            kMinimumPayloadSize > 0,
            "TransportLayer class kMinimumPayloadSize template parameter must be above 0."
        );
        static_assert(
            kMinimumPayloadSize < 255,
            "TransportLayer class kMinimumPayloadSize template parameter must be less than 255."
        );

    public:
        /// Stores the runtime status of the most recently called method. Note, this variable stores the status
        /// derived from the kTransportLayerCodes enumeration if it originates from a native method of
        /// this class. Alternatively, it uses the enumerations for the COBSProcessor and CRCProcessor helper classes
        /// if the status (error) originates from one of these classes. As such, you may need to use all the
        /// enumerations available through shared_assets namespace to determine the status of the most recently called
        /// method. All status codes used by this library are unique across the library, so any returned byte-code
        /// always has a single meaning.
        uint8_t transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kStandby);

        /**
         * @brief Instantiates a new TransportLayer class object.
         *
         * The constructor resets the _transmission_buffer and _reception_buffer of the instantiated class to 0
         * following initialization. Also initializes the CRCProcessor class using the provided CRC parameters. Note,
         * the CRCProcessor class is defined using the PolynomialType template parameter and, as such, expects and
         * casts all input CRC arguments to the same type.
         *
         * @param communication_port A reference to the fully configured instance of stream interface class, such as
         * Serial or USB Serial. This class is used as a low-level access point that physically manages the hardware
         * used to transmit and receive the serialized data.
         * @param crc_polynomial The polynomial to use for the generation of the CRC lookup table used by the internal
         * CRCProcessor class. Can be provided as an appropriately sized HEX number (e.g., 0x1021). Note, currently only
         * non-reversed polynomials are supported.
         * @param crc_initial_value The initial value to which the CRC checksum variable is initialized during
         * calculation. This value is based on the polynomial parameter. Can be provided as an appropriately sized
         * HEX number (e.g., 0xFFFF).
         * @param crc_final_xor_value The final XOR value to be applied to the calculated CRC checksum value. This
         * value is based on the polynomial parameter. Can be provided as an appropriately sized HEX number
         * (e.g., 0x0000).
         * @param start_byte The byte-range value (from 0 through 255) to be used as the start byte of each transmitted
         * and received packet. The presence of this value inside the incoming byte-stream instructs the receiver to
         * enter packet parsing mode. This value ideally should be different from the delimiter_byte to maintain higher
         * packet reliability, but it does not have to be. Also, it is advised to use a value that is unlikely to be
         * encountered due to random communication line noise.
         * @param delimiter_byte The byte-range value (from 0 through 255) to be used as the delimiter (stop) byte of
         * each packet. Encountering a delimiter_byte value is the only non-error way of ending the packet reception
         * loop. During packet construction, this value is eliminated from the payload using COBS encoding.
         * It is advised to use the value of 0x00 (0) as this is the only value that is guaranteed to not occur
         * anywhere in the packet. All other values may also show up as the overhead byte
         * (see COBSProcessor documentation for more details).
         * @param timeout The number of microseconds to wait between receiving any two consecutive bytes of the packet.
         * The algorithm uses this value to detect stale packets, as it expects all bytes of the same packet to arrive
         * close in time to each other. Primarily, this is a safeguard to break out of stale packet reception cycles
         * and avoid deadlocking the controller into the packet reception mode. Defaults to 20000 us (20 ms).
         * @param allow_start_byte_errors A boolean flag that determines whether the class raises errors when it is
         * unable to find the start_byte value in the incoming byte-stream. It is advised to keep this set to False for
         * most use cases. This is because it is fairly common to see noise-generated bytes inside the reception buffer
         * that are then silently cleared by the algorithm until a real packet becomes available. However, enabling this
         * option may be helpful for certain debugging scenarios.
         *
         * Example instantiation:
         * @code
         * Serial.begin(9600);
         * constexpr uint8_t maximum_tx_payload_size = 254;
         * constexpr uint8_t maximum_rx_payload_size = 200;
         * constexpr uint16_t polynomial = 0x1021;
         * constexpr uint16_t initial_value = 0xFFFF;
         * constexpr uint16_t final_xor_value = 0x0000;
         * constexpr uint8_t start_byte = 129;
         * constexpr uint8_t delimiter_byte = 0;
         * constexpr uint8_t minimum_payload_size = 1;
         * constexpr uint32_t timeout = 20000; // In microseconds
         * constexpr bool allow_start_byte_errors = false;
         *
         * // Instantiates a new TransportLayer object
         * TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size, minimum_payload_size> tl_class(
         * Serial,
         * polynomial,
         * initial_value,
         * final_xor_value,
         * start_byte,
         * delimiter_byte,
         * timeout,
         * allow_start_byte_errors
         * );
         * @endcode
         */
        explicit TransportLayer(
            Stream& communication_port,
            const PolynomialType crc_polynomial      = 0x07,
            const PolynomialType crc_initial_value   = 0x00,
            const PolynomialType crc_final_xor_value = 0x00,
            const uint8_t start_byte                 = 129,
            const uint8_t delimiter_byte             = 0,
            const uint32_t timeout                   = 20000,
            const bool allow_start_byte_errors       = false
        ) :
            _port(communication_port),
            _crc_processor(crc_polynomial, crc_initial_value, crc_final_xor_value),
            kStartByte(start_byte),
            kDelimiterByte(delimiter_byte),
            kTimeout(timeout),
            allow_start_byte_errors(allow_start_byte_errors),
            _transmission_buffer {},  // Initialization doubles up as resetting buffers to 0
            _reception_buffer {}
        {
            // Sets the start_byte placeholder to the actual start byte value. This is set at class instantiation and
            // kept constant throughout the lifetime of the class.
            _transmission_buffer[0] = kStartByte;

            // Ensures that the requested reception and transmission buffers do not exceed the microcontroller's serial
            // buffer size.
            static_assert(
                kTransmissionBufferSize <= kSerialBufferSize,
                "TransportLayer's transmission buffer size exceeds the serial buffer size for this type for "
                "microcontroller boards. If you manually increased the serial buffer size, edit the preprocessor "
                "directives at the top of the transport_layer.h file. Otherwise, set the "
                "kMaximumTransmittedPayloadSize template argument of the TransportLayer class to the value appropriate "
                "for your microcontroller board. Note, in addition to the payload, the buffer may need up to 8 extra "
                "bytes to store packet metadata."
            );
            static_assert(
                kReceptionBufferSize <= kSerialBufferSize,
                "TransportLayer's reception buffer size exceeds the serial buffer size for this type for "
                "microcontroller boards. If you manually increased the serial buffer size, edit the preprocessor "
                "directives at the top of the transport_layer.h file. Otherwise, set the "
                "kMaximumReceivedPayloadSize template argument of the TransportLayer class to the value appropriate "
                "for your microcontroller board. Note, in addition to the payload, the buffer may need up to 8 extra "
                "bytes to store packet metadata."
            );
        }

        /**
         * @brief Evaluates whether the reception buffer of the transmission interface has bytes to read (checks whether
         * bytes have been received).
         *
         * This is a simple wrapper around an accessor method of the transmission interface class that can be used
         * to quickly evaluate whether ReceiveData() method needs to be called to parse the incoming data. This allows
         * saving computation time by avoiding unnecessary ReceiveData() calls.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * @returns bool True if there are bytes to be read from the transmission interface reception buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * bool available = serial_protocol.Available();
         * @endcode
         */
        [[nodiscard]]
        bool Available() const
        {
            // If enough bytes are available to potentially represent a complete packet, returns 'true' to indicate
            // there are enough bytes to justify a read operation
            if (_port.available() >= static_cast<int>(kMinimumPacketSize))
            {
                return true;
            }

            // Otherwise returns 'false' to indicate that the read call is not justified
            return false;
        }

        /**
         * @brief Resets the _transmission_buffer's payload_size tracker variable (index 1) and the overhead byte
         * variable (index 2) to 0.
         *
         * This method is primarily used by other class methods to prepare the buffer to stage a new payload after
         * sending the old payload or encountering a transmission error. It can be called externally if a particular
         * pipeline requires forcibly resetting the buffer.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * serial_protocol.ResetTransmissionBuffer();
         * @endcode
         */
        void ResetTransmissionBuffer()
        {
            _transmission_buffer[kPayloadSizeIndex]  = 0;  // Payload Size
            _transmission_buffer[kOverheadByteIndex] = 0;  // Overhead Byte
        }

        /**
         * @brief Resets the _reception_buffer's payload_size tracker variable (index 1) and the overhead byte variable
         * (index 2) to 0.
         *
         * This method is primarily used by other class methods to prepare the buffer for the reception of a new data
         * packet after fully consuming a received payload or encountering reception error. It can be called externally
         * if a particular pipeline requires forcibly resetting the buffer's overhead byte variable and tracker.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * serial_protocol.ResetReceptionBuffer();
         * @endcode
         */
        void ResetReceptionBuffer()
        {
            _reception_buffer[kPayloadSizeIndex]  = 0;  // Payload Size
            _reception_buffer[kOverheadByteIndex] = 0;  // Overhead Byte
        }

        /**
         * @brief Copies the _transmission_buffer contents into the input destination buffer.
         *
         * This method is designed to help developers writing test functions. It accepts a buffer of the same type and
         * size as the _transmission_buffer via reference and copies the contents of the _transmission_buffer into the
         * input buffer. This way, the _transmission_buffer can be indirectly exposed for evaluation without in any way
         * modifying the buffer's contents.
         *
         * @warning Do not use this method for anything other than testing! It does not explicitly control buffer
         * layout and tracker variable setting behavior, which is important for the production code execution.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * @tparam DestinationSize The byte-size of the destination buffer array. Inferred automatically and used to
         * ensure the input buffer size exactly matches the _transmission_buffer size.
         *
         * @param destination The destination buffer to copy the _transmission_buffer contents to.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * uint16_t tx_buffer_size = tl_class.get_tx_buffer_size();  // Obtains the transmission buffer size
         * // The size of the buffer has to match the size of the transmission_buffer
         * uint8_t test_buffer[tx_buffer_size];
         * serial_protocol.CopyTxDataToBuffer(test_buffer);
         * @endcode
         */
        template <size_t DestinationSize>
        void CopyTxDataToBuffer(uint8_t (&destination)[DestinationSize])
        {
            // Ensures that the destination is the same size as the _transmission_buffer
            static_assert(
                DestinationSize == kTransmissionBufferSize,
                "Destination buffer size must be equal to the _transmission_buffer size."
            );

            // Copies the _transmission_buffer into the referenced destination buffer
            memcpy(destination, _transmission_buffer, DestinationSize);
        }

        /**
         * @brief Copies the _reception_buffer contents into the input destination buffer.
         *
         * This method is designed to help developers writing test functions. It accepts a buffer of the same type and
         * size as the _reception_buffer via reference and copies the contents of the _reception_buffer into the
         * input buffer. This way, the _reception_buffer can be indirectly exposed for evaluation without in any way
         * modifying the buffer's contents.
         *
         * @warning Do not use this method for anything other than testing! It does not explicitly control buffer
         * layout and tracker variable setting behavior, which is important for the production code execution.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * @tparam DestinationSize The byte-size of the destination buffer array. Inferred automatically and used to
         * ensure the input buffer size exactly matches the _reception_buffer size.
         *
         * @param destination The destination buffer to copy the _reception_buffer contents to.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * uint16_t rx_buffer_size = tl_class.get_rx_buffer_size();  // Obtains the reception buffer size
         * uint8_t test_buffer[rx_buffer_size];  // The size of the buffer has to match the size of the reception_buffer
         * serial_protocol.CopyRxDataToBuffer(test_buffer);
         * @endcode
         */
        template <size_t DestinationSize>
        void CopyRxDataToBuffer(uint8_t (&destination)[DestinationSize])
        {
            // Ensures that the destination is the same size as the _reception_buffer
            static_assert(
                DestinationSize == kReceptionBufferSize,
                "Destination buffer size must be equal to the _reception_buffer size."
            );

            // Copies the _reception_buffer contents into the referenced destination buffer
            memcpy(destination, _reception_buffer, DestinationSize);
        }

        /**
         * @brief Copies the payload bytes from _transmission_buffer to the _reception_buffer.
         *
         * This method is designed to help developers writing test functions. It checks that the payload written to
         * _transmission_buffer (via WriteData()) is small enough to fit into the payload region of the
         * _reception_buffer and, if so, copies it to the _reception_buffer. This allows to safely 'write' to the
         * _reception_buffer.
         *
         * @warning Do not use this method for anything other than testing! It upholds all safety standards, but, in
         * general, there should never be a need to write to reception_buffer outside testing scenarios.
         *
         * @note This is a public utility method and, as such, it does not modify internal class instance
         * transfer_status variable.
         *
         * @returns bool True if the _reception_buffer was successfully updated, False otherwise.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * uint8_t test_value = 50;
         * serial_protocol.WriteData(test_value); // Saves the test value to the _transmission_buffer payload
         * serial_protocol.CopyTxBufferPayloadToRxBuffer();  // Moves the payload over to the _reception_buffer
         * @endcode
         */
        bool CopyTxBufferPayloadToRxBuffer()
        {
            // Ensures that the payload size to move will fit inside the payload region of the _reception_buffer.
            if (_transmission_buffer[kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
            {
                return false;  // If not aborts by returning false
            }

            // Copies the payload from _transmission_buffer to _reception_buffer. Note, this excludes(!) most metadata
            // and crc checksum postamble.
            memcpy(
                &_reception_buffer[kPayloadStartIndex],
                &_transmission_buffer[kPayloadStartIndex],
                _transmission_buffer[kPayloadSizeIndex]
            );

            // Updates the payload_size tracker of the _reception_buffer to match the copied payload size.
            _reception_buffer[kPayloadSizeIndex] = _transmission_buffer[kPayloadSizeIndex];

            return true;  // Returns true to indicate the payload was successfully copied
        }

        /// Returns the current value of the _transmission_buffer's payload_size tracker variable.
        [[nodiscard]]
        uint8_t get_tx_payload_size() const
        {
            return _transmission_buffer[kPayloadSizeIndex];
        }

        /// Returns the current value of the _reception_buffer's payload_size tracker variable.
        [[nodiscard]]
        uint8_t get_rx_payload_size() const
        {
            return _reception_buffer[kPayloadSizeIndex];
        }

        /// Sets the allow_start_byte_errors flag to the input boolean value.
        void set_allow_start_byte_errors(const bool flag_value)
        {
            allow_start_byte_errors = flag_value;
        }

        /// Returns the value of the class kMaximumTransmittedPayloadSize template parameter.
        static constexpr uint8_t get_maximum_tx_payload_size()
        {
            return kMaximumTransmittedPayloadSize;
        }

        /// Returns the value of the class kMaximumReceivedPayloadSize template parameter.
        static constexpr uint8_t get_maximum_rx_payload_size()
        {
            return kMaximumReceivedPayloadSize;
        }

        /// Returns the size of the _transmission_buffer used by the class
        static constexpr uint16_t get_tx_buffer_size()
        {
            return kTransmissionBufferSize;
        }

        /// Returns the size of the _reception_buffer used by the class
        static constexpr uint16_t get_rx_buffer_size()
        {
            return kReceptionBufferSize;
        }

        /**
         * @brief Packages the data inside the _transmission_buffer into a serialized packet and transmits it using the
         * transmission interface class.
         *
         * This is the main transmission method that aggregates all steps necessary to correctly transmit the payload
         * stored in the _transmission_buffer as a serialized byte-stream using the transmission interface class.
         * Specifically, it first encodes the payload using COBS protocol and then calculates and adds the CRC checksum
         * for the encoded packet to the end of the packet. If all packet construction steps are successful, the method
         * then transmits the data using the transmission interface.
         *
         * @attention This method relies on the payload_size tracker variable of the _transmission_buffer (index 1) to
         * determine how many bytes inside the _transmission_buffer need to be encoded and added to the packet. That
         * value can be obtained by using the get_tx_payload_size() getter method if you need to determine the number
         * of the tracked payload bytes at any point in time.
         *
         * @returns bool True if the packet was successfully constructed and sent and False otherwise. If method runtime
         * fails, use the transfer_status variable to determine the reason for the failure, as it would be set to the
         * specific error code of the failed operation. Status values are guaranteed to uniquely match one of the
         * enumerators stored inside the kCOBSProcessorCodes, kCRCProcessorCodes, or kTransportLayerCodes
         * enumerations available through the shared_assets namespace.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * bool packet_sent = serial_protocol.SendData();
         * @endcode
         */
        bool SendData()
        {
            // Constructs the packet to be transmitted using the _transmission_buffer. Note, during this process, a CRC
            // checksum is calculated for the packet and appended to the end of the encoded packet. The returned size of
            // the data to send includes the CRC checksum size. If the returned combined_size is not 0, this indicates
            // that the packet has been constructed successfully.
            const uint16_t combined_size = ConstructPacket();
            if (combined_size != 0)
            {
                // Sends the data using the transmission interface class.
                _port.write(_transmission_buffer, combined_size);

                // Communicates that the packet has been sent via the transfer_status variable
                transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketSent);

                // Resets the transmission_buffer after every successful transmission
                ResetTransmissionBuffer();

                return true;  // Returns true to indicate that the packet has been sent
            }

            // If the combined_size is 0, this indicates that the packet has not been constructed successfully. Then,
            // returns 0 to indicate data sending failed. Since ConstructPacket() method automatically sets the
            // transfer_status variable to the appropriate error code, does not modify the status code.
            return false;
        }

        /**
         * @brief Receives the packet from the transmission interface class and decodes its payload into the
         * _reception_buffer.
         *
         * This is the main reception method that aggregates all steps necessary to correctly receive a data packet from
         * bytes stored inside the circular reception buffer of the transmission interface class. Specifically, when
         * called, the method first reads the packet from raw bytes received by the transmission interface into the
         * _reception_buffer. If this operation succeeds, the method then validates the integrity of the packet
         * using the CRC checksum and then unpacks the payload of the packet using COBS decoding in-place. The method
         * will only run all these steps if enough bytes are available to potentially encode a packet of at least the
         * minimum packet size.
         *
         * @note Following the successful runtime of this method, the number of payload bytes received from the PC can
         * be obtained using get_rx_payload_size() method.
         *
         * @returns bool True if the packet was successfully received and unpacked, False otherwise. If method runtime
         * fails, use the transfer_status variable to determine the reason for the failure, as it would be set to the
         * specific error code of the failed operation. Status values are guaranteed to uniquely match one of the
         * enumerators stored inside the kCOBSProcessorCodes, kCRCProcessorCodes, or kTransportLayerCodes
         * enumerations available through the shared_assets namespace.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * bool data_received = serial_protocol.ReceiveData();
         * @endcode
         */
        bool ReceiveData()
        {
            // Returns 'false' if not enough bytes are available to justify the parsing attempt. Specifically, if the
            // available number of bytes is lower than the minimum expected packet size.
            if (!Available())
            {
                // Also sets the status appropriately
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer);
                return false;
            }

            // Resets the reception buffer to prepare for receiving the next packet.
            ResetReceptionBuffer();

            // Attempts to parse a packet (and its CRC checksum postamble) from the transmission interface rx buffer if
            // bytes to read are available.
            const uint16_t packet_size = ParsePacket();

            // If the returned packet_size is 0, that indicates an error has occurred during the parsing process.
            // ParsePacket() automatically sets the class transfer_status variable to the appropriate error code, so
            // just returns 'false' top explicitly indicate failure and break method runtime.
            if (packet_size == 0)
            {
                return false;
            }

            // If the returned packet size is not 0, this indicates that the packet has been successfully parsed and now
            // occupies packet_size number of bytes inside the _reception_buffer. Additionally, this means that
            // kPostambleSize bytes immediately following the packet are filled with the CRC checksum for the packet. In
            // this case, verifies the integrity of the packet by running the CRC checksum calculator on the packet and
            // the appended CRC checksum. Similar to the above, if returned payload_size is 0, that indicates that an
            // error was encountered during packet validation or decoding, which is communicated to the caller process
            // by returning false. The transfer_status is automatically set to error code during ValidatePacket()
            // runtime.
            const uint16_t payload_size = ValidatePacket(packet_size);
            if (payload_size == 0)
            {
                return false;
            }

            // If the method reaches this point, the packet has been successfully received, validated and unpacked. The
            // payload is now available for consumption through the _reception_buffer. Sets the status appropriately and
            // returns 'true' to indicate successful runtime.
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketReceived);
            return true;
        }

        /**
         * @brief Writes the input object as bytes into the _transmission_buffer payload region starting at the
         * specified start_index.
         *
         * This method modifies the _transmission_buffer by (over)writing the specific portion of the buffer with the
         * bytes copied from the input object. The buffer remains intact (as-is) everywhere except for the overwritten
         * area. To reset the whole buffer, use the ResetTransmissionBuffer() method. The input object is not modified
         * in any way by this method.
         *
         * @warning If the requested start_index and object size (in bytes) combination exceeds
         * kMaximumTransmittedPayloadSize class template parameter value, the method will abort and return 0 to
         * indicate no bytes were written.
         *
         * @note This method operates specifically on the region allocated for the payload of the packet. It implicitly
         * handles the necessary transformations of the start_index to make sure start_index 0 corresponds to the start
         * index of the payload (kPayloadStartIndex) and that the end index never exceeds the maximum end_index of the
         * payload. This makes this method a safe way of modifying the payload with respect to the _transmission_buffer
         * layout heuristics necessary for other class methods to work as intended.
         *
         * @tparam ObjectType The type of the object from which the bytes will be copied over to the
         * _transmission_buffer. The template uses this parameter to instantiate the appropriate version of the method
         * for any valid object.
         * @param object The object from which the bytes are copied. Passed as a constant reference to reduce memory
         * overhead as the object itself is not modified in any way.
         * @param start_index The index inside the _transmission_buffer payload region, from which to start writing
         * bytes. Minimum value is 0, maximum value is defined by kMaximumTransmittedPayloadSize - 1
         * (but no more than 253). This index specifically applies to the payload, not the buffer as a whole.
         * @param provided_bytes The number of bytes to write to the _transmission_buffer. In most cases this should be
         * left blank as it allows the method to use the value returned by sizeOf() of the ObjectType
         * (writing as many bytes as supported by the object type).
         *
         * @returns uint16_t The index immediately following the last overwritten index of the payload region. This
         * value can be used as the starting index for later write operations to ensure data contiguity. If method
         * runtime fails, returns 0 to indicate no bytes were written to the payload and saves the specific error code
         * that describes the failure to transfer_status class variable.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * uint16_t value = 44321;
         * uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
         * struct MyStruct
         * {
         *    uint8_t a = 60;
         *    uint16_t b = 12345;
         *    uint32_t c = 1234567890;
         * } test_structure;
         *
         * // Writes tested objects to the _transmission_buffer
         * uint16_t next_index = serial_protocol.WriteData(value);
         * uint16_t next_index = serial_protocol.WriteData(array, next_index);
         * uint16_t next_index = serial_protocol.WriteData(test_structure, next_index);
         * @endcode
         */
        template <typename ObjectType>
        uint16_t WriteData(
            const ObjectType& object,
            const uint16_t& start_index    = 0,
            const uint16_t& provided_bytes = sizeof(ObjectType)
        )
        {
            // Calculates the total size of the payload that would be required to accommodate provided_bytes number of
            // bytes inserted starting with start_index.
            const uint16_t required_size = start_index + provided_bytes;

            // Verifies that the payload region of the buffer has enough space to accommodate the provided bytes inside
            // the payload region.
            if (required_size > kMaximumTransmittedPayloadSize)
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kWriteObjectBufferError);
                return 0;
            }

            // Shifts the input start index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            uint16_t local_start_index = start_index + kPayloadStartIndex;

            // If there is enough buffer space to accommodate the data, uses memcpy() to efficiently copy the data
            // into the _transmission_buffer.
            memcpy(
                static_cast<void*>(&_transmission_buffer[local_start_index]
                ),                                  // Destination in the buffer to start writing to
                static_cast<const void*>(&object),  // Source object address to copy from
                provided_bytes                      // The number of bytes to write into the buffer
            );

            // If writing to buffer caused the size of the payload to become larger than the size tracked by the
            // payload_size variable (index 1 of the buffer), updates the tracker to store the new size. This way, the
            // tracker is only updated whenever the used size of the payload increases and ignores overwrite operations.
            // The downside of this approach is that the only way to decrease the tracker value is by completely
            // resetting the buffer, which is nevertheless considered to be a fairly niche use case.
            _transmission_buffer[kPayloadSizeIndex] =
                max(_transmission_buffer[kPayloadSizeIndex], static_cast<uint8_t>(required_size));

            // Sets the status code to indicate writing to buffer was successful
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kObjectWrittenToBuffer);

            // Also returns the index immediately following the last updated (overwritten) index (relative to the start
            // of the payload) to caller to support chained method calls.
            return required_size;
        }

        /**
         * @brief Reads the requested_bytes number of bytes from the _reception_buffer payload region starting at the
         * start_index into the provided object.
         *
         * @note This operation does not modify _reception_buffer.
         *
         * This method copies the data from the _reception_buffer into the provided object, modifying the object.
         * Data can be copied from the _reception_buffer any number of times using arbitrary byte-counts and starting
         * positions, as long as the ending index of the read operation remains within the payload region boundaries.
         * The method explicitly prevents reading the data outside the payload region as it may be set to valid values
         * that nevertheless are meaningless as they are leftover from previously processed payloads. This stems from
         * the fact the buffer is never fully reset and instead only partially overwritten with data. To actually
         * update the _reception_buffer, you need to use ReceiveData() or ResetReceptionBuffer() methods.
         *
         * @warning If the requested start_index and requested_bytes combination exceeds the size of the received
         * payload, provided by the value of the payload_size tracker variable (index 1 of the buffer), the method will
         * abort and return 0 to indicate no bytes were read. Use get_rx_payload_size() method to get the size of the
         * currently stored payload.
         *
         * @note This method operates specifically on the region allocated for the payload of the packet. It implicitly
         * handles the necessary transformations of the start_index to make sure start_index 0 corresponds to the start
         * index of the payload (kPayloadStartIndex) and that the end index never exceeds the maximum end_index of the
         * payload. This makes this method a safe way of accessing the payload with respect to the _transmission_buffer
         * layout heuristics necessary for other class methods to work as intended.
         *
         * @tparam ObjectType The type of the object to which the read bytes would be written. The template uses this
         * parameter to instantiate the appropriate version of the method for any valid object.
         * @param object The object to write the data to. Passed as the reference to the object to enable direct object
         * data manipulation.
         * @param start_index The index inside the received payload, from which to start reading bytes. Minimum value
         * is 0, maximum value is defined by kMaximumReceivedPayloadSize - 1 (but no more than 253). This index
         * specifically applies to the payload, not the buffer as a whole.
         * @param requested_bytes The number of bytes to read from the _reception_buffer. In most cases, this should be
         * left blank as it allows the method to use the value returned by sizeOf() of the ObjectType
         * (requesting as many bytes as supported by the object type).
         *
         * @returns uint16_t The index inside the payload region of the _reception_buffer that immediately follows the
         * final index of the variable that was read by the method into the provided object. This allows using the
         * output of the method as a start_index for further read operations to ensure data contiguity. If method
         * runtime fails, returns 0 to indicate no bytes were read from the payload and saves the specific error code
         * that describes the failure to transfer_status class variable.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 254, 254> tl_class(Serial, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000);
         * uint16_t value = 44321;
         * uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
         * struct MyStruct
         * {
         *    uint8_t a = 60;
         *    uint16_t b = 12345;
         *    uint32_t c = 1234567890;
         * } test_structure;
         *
         * // Overwrites the test objects with the data stored inside the buffer
         * uint16_t next_index = serial_protocol.ReadData(value);
         * uint16_t next_index = serial_protocol.ReadData(array, next_index);
         * uint16_t next_index = serial_protocol.ReadData(test_structure, next_index);
         * @endcode
         */
        template <typename ObjectType>
        uint16_t ReadData(
            ObjectType&
                object,  // Not constant by design, while the reference is constant, the object itself is mutable.
            const uint16_t& start_index     = 0,
            const uint16_t& requested_bytes = sizeof(ObjectType)
        )
        {
            // Calculates the total size of the payload that is enough to accommodate _requested_bytes number of bytes
            // stored upstream of the start_index.
            uint16_t required_size = start_index + requested_bytes;

            // Verifies that the payload region of the buffer has enough bytes to accommodate reading the requested
            // number of bytes from the start_index. Gets the current number of payload bytes from the tracker variable
            // inside the _reception_buffer.
            if (required_size > _reception_buffer[kPayloadSizeIndex])
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kReadObjectBufferError);
                return 0;
            }

            // Shifts the input start_index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            uint16_t local_start_index = start_index + kPayloadStartIndex;

            // If there are enough bytes in the payload to read, uses memcpy to efficiently copy the data into the
            // object from the _reception_buffer.
            memcpy(
                static_cast<void*>(&object),  // Destination object to write the data to
                static_cast<const void*>(&_reception_buffer[local_start_index]),  // Source to read the data from
                requested_bytes  // The number of bytes to read into the object
            );

            // Sets the status code to indicate reading from buffer was successful
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kObjectReadFromBuffer);

            // Also returns the index immediately following the index of the final read byte (relative to the payload)
            // to caller. This index can be used as the next input start_index if multiple read calls are chained
            // together.
            return required_size;
        }

    private:
        /// Tracks the position of the variable that stores the payload size inside the class buffers. This value is
        /// expected to remain constant in future versions of the class.
        static constexpr uint8_t kPayloadSizeIndex = 1;

        /// Stores the position of the variable that stores the overhead byte inside the class buffers. This value
        /// doubles-up as the preamble size. The payload is expected to be found immediately after the overhead byte.
        /// This value is expected to remain constant in future versions of the class.
        static constexpr uint8_t kOverheadByteIndex = 2;

        /// Stores the starting position of the payload inside the class buffers. Directly dependent on the position of
        /// the overhead byte.
        static constexpr uint8_t kPayloadStartIndex = kOverheadByteIndex + 1;

        /// The reference to the Stream class object used as the low-level transmission interface by this class. During
        /// testing, this variable can be set to an instance of the StreamMock class, which exposes the low-level
        /// buffers to assist evaluating the behavior of the tested TransportLayer class methods.
        Stream& _port;

        /// The local instance of the COBSProcessor class that provides the methods to encode and decode packets using
        /// COBS protocol. See the class documentation for more details on the process and the functionality of the
        /// class.
        COBSProcessor<> _cobs_processor {};  // Default template argument values match static variables of this class.

        /// The local instance of the CRCProcessor class that provides the methods to calculate the CRC checksum for
        /// packets and save and read the checksum from class buffers. See the class documentation for more details on
        /// the process and the functionality of the class.
        CRCProcessor<PolynomialType> _crc_processor;

        /// The byte-value used to indicate the start of the packet. Encountering this byte in the evaluated incoming
        /// byte-stream is the only trigger that starts packet reception cycle.
        const uint8_t kStartByte;

        /// The byte-value used to indicate the end of the packet. All instances of this byte in the payload are
        /// eliminated using COBS. All valid COBS-encoded packets should end in the delimiter byte value, and this
        /// heuristic is used as a secondary verification step to ensure packet integrity.
        const uint8_t kDelimiterByte;

        /// The maximum number of microseconds (us) to wait between receiving bytes of the packet. In packet reception
        /// mode, the algorithm will wait for the specified number of microseconds before declaring the packet stale and
        /// aborting the reception procedure, when waiting for more bytes to become available. This is the only way to
        /// abort packet reception cycle other than encountering the delimiter byte value.
        const uint32_t kTimeout;

        /// A boolean flag that controls whether ParsePacket() method raises errors when it is unable to find the start
        /// byte of the packet. The default behavior is to disable such errors as they are somewhat common due to having
        /// noise in the communication lines. However, some users m,ay want to enable these errors to help debugging,
        /// so the option is preserved to be user-controllable.
        bool allow_start_byte_errors;

        /// Stores the byte-size of the postamble, which currently statically depends on the declared PolynomialType.
        /// The postamble is the portion of the data that immediately follows the delimiter byte of the encoded payload
        /// and, at the time of writing, only includes the CRC checksum for the packet. To optimize data transfer, the
        /// postamble is appended to the specifically reserved portion of the _transmission_buffer and received into
        /// the reserved portion of the _reception_buffer, rather than being stored in a separate buffer.
        static constexpr uint8_t kPostambleSize = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /// This variable is set as part of class initialization, and it is dependent on the kMinimumPayloadSize
        /// (user-defined) and the preamble and postamble sizes. It allows optimizing class computations by avoiding
        /// low-level IO buffer manipulations that are unlikely to succeed.
        static constexpr uint16_t kMinimumPacketSize =  // NOLINT(*-dynamic-static-initializers)
            kMinimumPayloadSize + kOverheadByteIndex + kPostambleSize;

        /// Stores the size of the _transmission_buffer array, which is derived based on the preamble, encoded payload
        /// and postamble size. The +2 accounts for the overhead byte and delimiter byte of the encoded payload, the
        /// kPostambleSize accounts for the CRC checksum, and the kOverheadByteIndex serves as the preamble size.
        static constexpr uint16_t kTransmissionBufferSize =  // NOLINT(*-dynamic-static-initializers)
            kMaximumTransmittedPayloadSize + kOverheadByteIndex + 2 + kPostambleSize;

        /// Stores the size of the _reception_buffer array, which is statically set to the maximum received payload size
        /// (kMaximumReceivedPayloadSize template parameter) modified to account for the preamble and the postamble
        /// sizes. See kTransmissionBufferSize docstring for more details, the calculation is the same except for the
        /// payload size.
        static constexpr uint16_t kReceptionBufferSize =  // NOLINT(*-dynamic-static-initializers)
            kMaximumReceivedPayloadSize + kOverheadByteIndex + 2 + kPostambleSize;

        /// The buffer that stages the payload data before it is transmitted. The buffer is constructed in a way that
        /// reserves enough space for the user-defined payload_size and all the service variables (preamble, postamble,
        /// COBS variables) that are necessary for the proper functioning of the class.
        uint8_t _transmission_buffer[kTransmissionBufferSize];

        /// The buffer that stores the received data before it is consumed. The buffer is constructed in a way that
        /// reserves enough space for the user-defined payload_size and all the service variables (preamble, postamble,
        /// COBS variables) that are necessary for the proper functioning of the class.
        uint8_t _reception_buffer[kReceptionBufferSize];

        /**
         * @brief Constructs the serialized packet using the payload stored inside the _transmission_buffer.
         *
         * Specifically, first uses COBS encoding to eliminate all instances of the kDelimiterByte value inside the
         * payload. The payload is expected be found starting at index 3 of the buffer, and its size is obtained by
         * reading the payload_size tracker found at index 1. After COBS encoding, the delimiter is appended to the end
         * of the new payload. Note, the packet includes the statically allocated preamble (start byte and payload size)
         * as well as the overhead byte and the delimiter byte. Finally, calculates the CRC checksum for the constructed
         * packet without the preamble (only the overhead + payload + delimiter are included in this calculation) and
         * appends it to the end of the packet.
         *
         * @note This method is intended to be called only by other TransportLayer class methods.
         *
         * @returns uint16_t The combined size of the preamble, packet, and the CRC checksum postamble in bytes
         * (262 bytes maximum). If method runtime fails, returns 0 to indicate that the packet was not constructed and
         * uses transfer_status to communicate the error code of the specific operation that failed.
         *
         * Example usage:
         * @code
         * uint16_t combined_size = ConstructPacket();
         * @endcode
         */
        uint16_t ConstructPacket()
        {
            // Carries out in-place payload encoding using COBS algorithm. Relies on the payload_size variable
            // (index 1) in the _transmission_buffer to communicate the payload size. Implicitly uses overhead byte
            // placeholder at index 2 and expects the buffer to contain enough space to append the delimiter byte to
            // the end of the used payload region. Note, the returned packet size EXCLUDES the preamble
            // (start byte and payload size).
            uint16_t packet_size = _cobs_processor.EncodePayload(_transmission_buffer, kDelimiterByte);

            // If the encoder runs into an error, it returns 0 to indicate that the payload was not encoded. In this
            // case, transfers the error status code from the COBS processor status tracker to transfer_status and
            // returns 0 to indicate packet construction failed.
            if (packet_size == 0)
            {
                transfer_status = _cobs_processor.cobs_status;
                return 0;
            }

            // If COBS encoding succeeded, calculates the CRC checksum on the encoded packet. This includes the overhead
            // byte and the delimiter byte added during COBS encoding. Note, uses the CRC type specified by the
            // PolynomialType class template parameter to automatically scale with all supported CRC types.
            // Note, the start index is set to the index of the overhead byte and the checksummed data length is the
            // same as the encoded data length (includes overhead and delimiter bytes). The method implicitly handles
            // all necessary conversions from payload-centered to buffer-centered indexing.
            PolynomialType checksum =
                _crc_processor.CalculatePacketCRCChecksum(_transmission_buffer, kOverheadByteIndex, packet_size);

            // If the CRC calculator runs into an error, as indicated by its status code not matching the expected
            // success code, transfers the error status to the transfer_status and returns 0 to indicate packet
            // construction failed.
            if (_crc_processor.crc_status !=
                static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated))
            {
                transfer_status = _crc_processor.crc_status;
                return 0;
            }

            // Writes the calculated CRC checksum to the _transmission_buffer at the position immediately following the
            // encoded packet. This way, the PC can run the CRC calculation on the received data and quickly verify its
            // integrity using the zero-expected-return CRC check method. Note, this relies on the storage buffers being
            // constructed in a way that always reserves enough space for the used CRC checksum, regardless of the
            // payload size.
            const uint16_t combined_size =
                _crc_processor.AddCRCChecksumToBuffer(_transmission_buffer, packet_size + kOverheadByteIndex, checksum);

            // If CRC addition fails, as indicated by the returned combined size being 0, transfers the specific error
            // status to the transfer_status and returns 0 to indicate packet construction failed.
            if (combined_size == 0)
            {
                transfer_status = _crc_processor.crc_status;
                return 0;
            }

            // If the algorithm reaches this point, this means that the payload has been successfully encoded and
            // checksummed and the CRC checksum has been added to the end of the encoded packet. Sets the
            // transfer_status appropriately and returns the combined size of the packet and the added CRC checksum
            // to let the caller know how many bytes to transmit to the PC.
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketConstructed);
            return combined_size;
        }

        /**
         * @brief Parses a serialized packet from the bytes stored in the reception buffer of the transmission interface
         * class.
         *
         * Specifically, if bytes are available for reading, scans through all available bytes until a start byte value
         * defined by kStartByte is found. Once the start byte is found, retrieves the incoming packet's payload_size
         * (which is expected to be stored in the byte immediately following the start byte). If the received payload
         * size is valid, enters a while loop that iteratively reads all packet_bytes (derived from payload_size) into
         * the _reception_buffer.
         *
         * @note This method is intended to be called only by other TransportLayer class methods.
         *
         * @returns uint16_t The number of packet bytes read into the _reception_buffer. The packet size includes the
         * overhead and delimiter bytes, the payload bytes, and the CRC checksum postamble (but excludes the start and
         * the payload_size bytes). If method runtime fails, returns 0 to indicate no packet bytes were read. If 0 is
         * returned, transfer_status is set to the appropriate error code that communicates the specific operation that
         * resulted in the failure.
         *
         * Example usage:
         * @code
         * uint16_t packet_size = ParsePacket();
         * @endcode
         */
        uint16_t ParsePacket()
        {
            // The timer that disengages the loop if the packet stales at any stage
            // Tracks the number of bytes read from the transmission interface buffer minus preamble bytes
            uint16_t bytes_read =
                kOverheadByteIndex;  // Initializes to the size of the preamble. See below for details.

            // First, attempts to find the start byte of the packet. The start byte is used to tell the receiver that
            // the following data belongs to a supposedly well-formed packet and should be retained
            // (written to the buffer) and not discarded.
            while (_port.available())
            {
                // Note, the start byte itself is not saved, which is the intended behavior, despite the rx buffer
                // having space for it.
                if (_port.read() == kStartByte)
                {
                    // Sets the status to indicate start byte has been found. The status is immediately used below to
                    // evaluate loop runtime
                    transfer_status =
                        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketStartByteFound);
                    break;
                }
            }

            // If the start byte was not found, aborts the method runtime and returns 0 to indicate that no data was
            // parsed as no packet was available.
            if (transfer_status !=
                static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketStartByteFound))
            {
                // Note, selects the status based on the value of the allow_start_byte_errors flag
                if (allow_start_byte_errors)
                {
                    transfer_status =
                        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketStartByteNotFound);
                }
                else
                {
                    transfer_status =
                        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer);
                }
                return 0;
            }

            // If the start byte was found, attempts to read the next byte, which should be the payload_size byte
            elapsedMicros timeout_timer = 0;  // Resets the timer to 0 before entering the loop
            while (timeout_timer < kTimeout)  // Blocks until timeout is reached or the byte is read
            {
                // If the byte can be read, reads it into the payload_size variable of the buffer and sets the status to
                // the appropriate value.
                if (_port.available())
                {
                    _reception_buffer[kPayloadSizeIndex] = _port.read();  // Reads payload_sie into the storage variable

                    // If payload size is below the declared minimum allowed size or above the declared maximum allowed
                    // size, aborts the reception procedure with an error (and sets the status appropriately).
                    if (_reception_buffer[kPayloadSizeIndex] < kMinimumPayloadSize ||
                        _reception_buffer[kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
                    {
                        transfer_status =
                            static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kInvalidPayloadSize);
                        return 0;
                    }

                    // If the payload size is within allowed limits, advances to packet reception
                    transfer_status =
                        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPayloadSizeByteFound
                        );  // Sets the status
                    break;  // Gracefully breaks out of the loop
                }
            }

            // If the payload_size byte was not found, aborts the method runtime and returns 0 to indicate that packet
            // reception staled at payload size reception.
            if (transfer_status !=
                static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPayloadSizeByteFound))
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPayloadSizeByteNotFound);
                return 0;
            }

            // Calculates how many additional 'packet' bytes are expected to be received. This is dependent on the
            // received payload_size and is modified to account for the additional COBS metadata bytes
            // ('+2,' accounts for the overhead and delimiter bytes).
            const uint16_t remaining_size = _reception_buffer[kPayloadSizeIndex] + kOverheadByteIndex + 2;
            timeout_timer                 = 0;  // Resets the timer to 0 before entering the loop below

            // Enters the packet reception loop. Loops either until the timeout (packet stales), an unencoded
            // delimiter byte value is encountered or the payload is fully received.
            bool delimiter_found = false;  // Tracks whether the loop below is able to find the unencoded delimiter
            while (timeout_timer < kTimeout && bytes_read < remaining_size)
            {
                // Checks whether bytes to parse are available to enable waiting for the packet bytes to be received.
                // In this case, 'while' loop will block in-place until more bytes are received or the timeout is
                // reached.
                if (_port.available())
                {
                    // Saves the byte to the appropriate buffer position
                    const uint8_t byte_value      = _port.read();
                    _reception_buffer[bytes_read] = byte_value;
                    // Increments the bytes_read to iteratively move along the buffer and add new data
                    bytes_read++;
                    timeout_timer = 0;  // Resets the timer whenever a byte is successfully read and the loop is active

                    if (byte_value == kDelimiterByte)
                    {
                        delimiter_found = true;  // Sets the flag to indicate that the delimiter byte was found
                        break;                   // Breaks out of the loop
                    }
                }
            }

            // Issues an error status if the loop above escaped due to a timeout
            if (timeout_timer >= kTimeout)
            {
                transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketTimeoutError);
                return 0;  // Returns 0 to indicate that the packet staled at the packet reception stage
            }

            // Issues an error status if the loop above did not find a delimiter (after receiving all packet bytes).
            if (!delimiter_found)
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kDelimiterNotFoundError);
                return 0;
            }

            // Issues an error status if the loop above escaped early due to finding a delimiter before reaching the end
            // of the packet.
            if (bytes_read != remaining_size)
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kDelimiterFoundTooEarlyError);
                return 0;
            }

            // If the packet parsing loop completed successfully, attempts to parse the CRC postamble. The CRC bytes
            // should be received immediately after receiving the packet delimiter byte.
            timeout_timer                 = 0;  // Resets the timer to 0 before entering the loop below
            const uint16_t postamble_size = remaining_size + static_cast<uint16_t>(kPostambleSize);
            while (timeout_timer < kTimeout && bytes_read < postamble_size)
            {
                if (_port.available())
                {
                    // Saves the byte to the appropriate buffer position
                    _reception_buffer[bytes_read] = _port.read();

                    // Increments the bytes_read to iteratively move along the buffer and add new data
                    bytes_read++;
                    timeout_timer = 0;  // Resets the timer whenever a byte is successfully read and the loop is active
                }
            }

            // Issues an error status if the loop above escaped due to a timeout.
            if (timeout_timer >= kTimeout)
            {
                transfer_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPostambleTimeoutError);
                return 0;  // Returns 0 to indicate that the packet staled at the packet reception stage
            }

            // Otherwise, if the loop above successfully resolved the necessary number of postamble bytes, sets the
            // success status and returns to caller
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketParsed);

            // Since bytes_read directly corresponds to the packet size, returns this to the caller
            return bytes_read - kOverheadByteIndex;  // Note, excludes the start and payload_size bytes
        }

        /**
         * @brief Validates the received packet and decodes the payload using COBS scheme.
         *
         * @attention Assumes that the ParsePacket() appends the CRC checksum postamble to the end of the encoded
         * payload.
         *
         * Specifically, first runs the CRC calculator on the portion of the buffer that holds the encoded payload
         * and the postamble CRC checksum, expecting to get a 0 returned checksum. If the CRC check is successful
         * (returns 0), decodes the payload using COBS to get the original payload. See CRCProcessor documentation if
         * you wonder why this looks for a 0 return value for the CRC check, this is a property of the CRC checksum
         * value.
         *
         * @note This method is intended to be called only by other TransportLayer class methods, and only after
         * the ParsePacket() method has been successfully called.
         *
         * @param packet_size The size of the packet that was parsed into the _reception_buffer by the ParsePacket()
         * method in bytes. The size should exclude the leading metadata bytes (start byte and payload_size). At the
         * time of writing, ValidatePacket() returns the correct packet_size to be input into this method with no
         * further modifications.
         *
         * @returns uint16_t The number of bytes making up the payload. If method runtime fails, returns 0 to indicate
         * that no bytes passed the verification and uses transfer_status to communicate the error code of the specific
         * operation that resulted in the failure.
         *
         * Example usage:
         * @code
         * uint16_t packet_size = ParsePacket();
         * uint16_t payload_size = ValidatePacket(packet_size);
         * @endcode
         */
        uint16_t ValidatePacket(uint16_t packet_size)
        {
            // Runs the CRC calculator on the _reception_buffer stretch that holds the encoded payload and CRC checksum.
            // This relies on the signature property of the CRC, namely that adding the CRC checksum to the data for
            // which the checksum was calculated and re-running the CRC on this combined data will always return 0.
            // Assumes that the CRC was stored as bytes starting with the most significant byte first, otherwise this
            // check will not work. Also assumes that the ParsePacket() method adds the CRC postamble to the portion of
            // the buffer immediately following the packet. Also starts with the overhead byte position expected to be
            // found at index 2.
            PolynomialType packet_checksum =
                _crc_processor.CalculatePacketCRCChecksum(_reception_buffer, kOverheadByteIndex, packet_size);

            // Verifies that the CRC calculator ran without errors and returned the success status. If not, sets
            // the transfer_status to the returned crc status and returns 0 to indicate crc calculator runtime error.
            // This is done to distinguish between failed CRC checks (packet corruption) and failed crc calculator
            // runtime.
            if (_crc_processor.crc_status !=
                static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated))
            {
                transfer_status = _crc_processor.crc_status;
                return 0;
            }

            // If the crc calculation runtime was successful, ensures that the calculated CRC checksum is 0, which is
            // the expected correct checksum for an uncorrupted packet (and an uncorrupted transmitted CRC checksum).
            if (packet_checksum != 0)
            {
                // If the returned checksum is not 0, that means that the packet failed the CRC check and is likely
                // corrupted.
                transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kCRCCheckFailed);
                return 0;
            }

            // If the CRC check succeeds, attempts to decode COBS-encoded data. This serves two purposes. First, it
            // restores any encoded variable that was previously an instance of kDelimiterByte back to the original
            // value. Second, it acts as a secondary verification step, since COBS encoding ensures the data is
            // organized in a particular fashion and if that is not true, the data is likely corrupted and the CRC
            // failed to recognize that.
            const uint16_t payload_size = _cobs_processor.DecodePayload(_reception_buffer, kDelimiterByte);

            // Verifies that the COBS decoder runtime was successful. Uses the heuristic that the successful COBS
            // decoder runtime always returns a non-zero payload_size, and an erroneous one always returns 0 to
            // simplify the 'if' check. If payload_size is 0, sets the transfer_status to the returned status code and
            // returns 0 to indicate runtime error.
            if (payload_size == 0)
            {
                transfer_status = _cobs_processor.cobs_status;
                return 0;
            }

            // If COBS decoding was successful, sets the packet status appropriately and returns the payload size to
            // caller
            transfer_status = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketValidated);
            return payload_size;
        }
};

#endif  //AXMC_TRANSPORT_LAYER_H
