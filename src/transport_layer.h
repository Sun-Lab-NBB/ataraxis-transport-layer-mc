/**
 * @file
 * @brief Provides the TransportLayer class that exposes methods for sending and receiving serialized data
 * over the USB and UART interfaces.
 *
 * @section tl_description Description:
 * The TransportLayer class provides the user-facing API that enables receiving and sending data over the USB or UART
 * serial interfaces. It conducts all necessary operations to properly encode and decode payloads, verify their
 * integrity, and move them to and from the appropriate communication interface buffers.
 *
 * @warning This class permanently reserves up to 550 bytes of RAM for the staging buffers and up to 1024 bytes for
 * storing the CRC lookup table. The number of bytes reserved for the staging buffers can be reduced by adjusting the
 * maximum transmission / reception buffer sizes. The number of bytes reserved for the CRC lookup table can be reduced
 * by adjusting the type of the polynomial used for the CRC checksum calculation.
 *
 * @section tl_packet_anatomy Packet Anatomy:
 * This class sends and receives data in the form of packets. Each packet adheres to the following general layout:
 * [START BYTE] [PAYLOAD SIZE] [OVERHEAD BYTE] [PAYLOAD] [DELIMITER BYTE] [CRC CHECKSUM]
 *
 * @note All user-facing methods only work with the payload portion of the data packet. The rest of the packet anatomy
 * is controlled internally by the TransportLayer instance.
 */

#ifndef AXTLMC_TRANSPORT_LAYER_H
#define AXTLMC_TRANSPORT_LAYER_H

// Dependencies
#include <Arduino.h>
#include <elapsedMillis.h>
#include "axtlmc_shared_assets.h"
#include "cobs_processor.h"
#include "crc_processor.h"

using namespace axtlmc_shared_assets;

// Statically defines the size of the Serial class reception buffer associated with different supported Arduino and
// Teensy board architectures.

// Arduino Due (USB serial).
#if defined(ARDUINO_ARCH_SAM)
static constexpr uint16_t kSerialBufferSize = 256;

// Arduino Zero, MKR series (USB serial).
#elif defined(ARDUINO_ARCH_SAMD)
static constexpr uint16_t kSerialBufferSize = 256;

// Arduino Nano 33 BLE (USB serial).
#elif defined(ARDUINO_ARCH_NRF52)
static constexpr uint16_t kSerialBufferSize = 256;

// Teensy revisions are defined based on the CPU model.
#elif defined(CORE_TEENSY)

// Teensy 3.x, 4.x (USB serial).
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || \
    defined(__IMXRT1062__)
static constexpr uint16_t kSerialBufferSize = 8192;  // Default is 4 x 2048 buffers == 8192 bytes total

// Teensy 2.0, Teensy++ 2.0 (USB serial) maximum reception buffer size in bytes.
#else
static constexpr uint16_t kSerialBufferSize = 256;

#endif

// Arduino Uno, Mega, and other AVR-based boards (UART serial) maximum reception buffer size in bytes.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__)
static constexpr uint16_t kSerialBufferSize = 64;

// The default fallback for unsupported boards is the reasonable minimum buffer size
#else
static constexpr uint16_t kSerialBufferSize = 64;

#endif

/**
 * @class TransportLayer
 * @brief Exposes methods for sending and receiving serialized data over the USB and UART communication interfaces.
 *
 * This class instantiates and manages all library assets used to transcode, validate, and bidirectionally transfer
 * serial data over the target communication interface. Critically, this includes the transmission and reception
 * buffers that are used to temporarily store the outgoing and incoming data payloads. All user-facing class methods
 * interact with the data stored in one of these buffers.
 *
 * @tparam PolynomialType The datatype of the polynomial to use for Cyclic Redundancy Check (CRC) checksum computations.
 * Valid types are uint8_t, uint16_t, and uint32_t.
 * @tparam kMaximumTransmittedPayloadSize The maximum size of the payload that is expected to be transmitted during
 * runtime. This parameter indirectly controls the size of the instance's transmission buffer. Must be a value between
 * 1 and 254.
 * @tparam kMaximumReceivedPayloadSize The maximum size of the payload that is expected to be received during runtime.
 * This parameter indirectly controls the size of the instance's reception buffer. Must be a value between 1 and 254.
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
 *
 * // Instantiates a new TransportLayer object
 * TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size>
 * tl_class(Serial, polynomial, initial_value, final_xor_value);
 * @endcode
 */
template <
    typename PolynomialType                      = uint8_t,                          // Defaults to uint8_t polynomials
    const uint8_t kMaximumTransmittedPayloadSize = min(kSerialBufferSize - 8, 254),  // Intelligently caps at 254 bytes
    const uint8_t kMaximumReceivedPayloadSize    = min(kSerialBufferSize - 8, 254)   // Intelligently caps at 254 bytes
    >
class TransportLayer
{
        // Ensures that the class only accepts uint8, 16 or 32 as valid CRC types, as no other type can be used to
        // store the CRC polynomial at the time of writing.
        static_assert(
            is_same_v<PolynomialType, uint8_t> || is_same_v<PolynomialType, uint16_t> ||
                is_same_v<PolynomialType, uint32_t>,
            "TransportLayer's PolynomialType template parameter must be either uint8_t, uint16_t, or "
            "uint32_t."
        );

        // Verifies that the template parameters specifying the size of the transmitted and received payloads are
        // within a valid range of values.
        static_assert(
            kMaximumTransmittedPayloadSize < 255,
            "TransportLayer's kMaximumTransmittedPayloadSize template parameter must be less than 255."
        );
        static_assert(
            kMaximumTransmittedPayloadSize > 0,
            "TransportLayer's kMaximumTransmittedPayloadSize template parameter must be greater than 0."
        );
        static_assert(
            kMaximumReceivedPayloadSize < 255,
            "TransportLayer's kMaximumReceivedPayloadSize template parameter must be less than 255."
        );
        static_assert(
            kMaximumReceivedPayloadSize > 0,
            "TransportLayer's kMaximumReceivedPayloadSize template parameter must be greater than 0."
        );

    public:
        /// Stores the runtime status of the most recently called method.
        uint8_t runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kStandby);

        /**
         * @brief Initializes all runtime assets that facilitate data transmission and reception.
         *
         * @param communication_port The initialized communication interface instance, such as Serial or USB Serial.
         * @param crc_polynomial The polynomial to use for the generation of the CRC lookup table. The polynomial must
         * be standard (non-reflected / non-reversed).
         * @param crc_initial_value The value to which the CRC checksum is initialized before calculation.
         * @param crc_final_xor_value The value with which the CRC checksum is XORed after calculation.
         *
         * Example instantiation:
         * @code
         * Serial.begin(9600);
         * constexpr uint8_t maximum_tx_payload_size = 254;
         * constexpr uint8_t maximum_rx_payload_size = 200;
         * constexpr uint16_t polynomial = 0x1021;
         * constexpr uint16_t initial_value = 0xFFFF;
         * constexpr uint16_t final_xor_value = 0x0000;
         *
         * // Instantiates a new TransportLayer object
         * TransportLayer<uint16_t, maximum_tx_payload_size, maximum_rx_payload_size>
         * tl_class(Serial, polynomial, initial_value, final_xor_value);
         * @endcode
         */
        explicit TransportLayer(
            Stream& communication_port,
            const PolynomialType crc_polynomial      = 0x07,
            const PolynomialType crc_initial_value   = 0x00,
            const PolynomialType crc_final_xor_value = 0x00
        ) :
            _port(communication_port),
            _crc_processor(crc_polynomial, crc_initial_value, crc_final_xor_value),
            _transmission_buffer {},
            _reception_buffer {}
        {
            // Sets the start_byte placeholder to the actual start byte value. This is set at class instantiation and
            // kept constant throughout the lifetime of the class.
            _transmission_buffer[0] = kBufferLayout::kStartByte;

            // Ensures that the requested reception and transmission buffers do not exceed the microcontroller's serial
            // buffer size.
            static_assert(
                kTransmissionBufferSize <= kSerialBufferSize,
                "TransportLayer's transmission buffer size exceeds the serial buffer size for this type of "
                "microcontroller boards. If the serial buffer size was increased , edit the preprocessor "
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
         * @brief Evaluates whether the communication interface has received enough bytes to justify reading the
         * incoming packet.
         *
         * @returns bool True if the communication interface has received enough bytes to likely contain an incoming
         * data packet and False otherwise.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
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
         * @brief Resets the instance's transmission buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * serial_protocol.ResetTransmissionBuffer();
         * @endcode
         */
        void ResetTransmissionBuffer()
        {
            _transmission_buffer[kBufferLayout::kPayloadSizeIndex]  = 0;  // Payload Size
            _transmission_buffer[kBufferLayout::kOverheadByteIndex] = 0;  // Overhead Byte
        }

        /**
         * @brief Resets the instance's reception buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * serial_protocol.ResetReceptionBuffer();
         * @endcode
         */
        void ResetReceptionBuffer()
        {
            _reception_buffer[kBufferLayout::kPayloadSizeIndex]  = 0;  // Payload Size
            _reception_buffer[kBufferLayout::kOverheadByteIndex] = 0;  // Overhead Byte
        }

        /**
         * @brief Copies the contents of the instance's transmission buffer into the specified destination buffer.
         *
         * @warning This method is intended for testing and debugging purposes and should not be used in production
         * runtimes.
         *
         * @tparam DestinationSize The size of the destination buffer, in bytes.
         *
         * @param destination The buffer where to copy the contents of the transmission buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * uint16_t tx_buffer_size = tl_class.get_tx_buffer_size();  // Gets the transmission buffer size
         * // The size of the destination buffer has to match the size of the transmission buffer
         * uint8_t test_buffer[tx_buffer_size];
         * serial_protocol.CopyTxDataToBuffer(test_buffer);
         * @endcode
         */
        template <size_t DestinationSize>
        void CopyTransmissionData(uint8_t (&destination)[DestinationSize])
        {
            // Ensures that the destination has the same size as the transmission buffer
            static_assert(
                DestinationSize == kTransmissionBufferSize,
                "Destination buffer size must be equal to the instance's transmission buffer size."
            );

            // Copies the transmission buffer's data into the referenced destination buffer
            memcpy(destination, _transmission_buffer, DestinationSize);
        }

        /**
         * @brief Copies the contents of the instance's reception buffer into the specified destination buffer.
         *
         * @warning This method is intended for testing and debugging purposes and should not be used in production
         * runtimes.
         *
         * @tparam DestinationSize The size of the destination buffer, in bytes.
         *
         * @param destination The buffer where to copy the contents of the reception buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * uint16_t rx_buffer_size = tl_class.get_rx_buffer_size();  // Gets the reception buffer size
         * // The size of the destination buffer has to match the size of the reception buffer
         * uint8_t test_buffer[rx_buffer_size];
         * serial_protocol.CopyRxDataToBuffer(test_buffer);
         * @endcode
         */
        template <size_t DestinationSize>
        void CopyReceptionData(uint8_t (&destination)[DestinationSize])
        {
            // Ensures that the destination has the same size as the reception buffer
            static_assert(
                DestinationSize == kReceptionBufferSize,
                "Destination buffer size must be equal to the instance's reception buffer size."
            );

            // Copies the reception buffer's data into the referenced destination buffer
            memcpy(destination, _reception_buffer, DestinationSize);
        }

        /**
         * @brief Copies the payload from the instance's transmission buffer to its reception buffer.
         *
         * This method only copies the payload. It does not copy the metadata (start byte) or the CRC checksum
         * postamble.
         *
         * @warning This method is intended for testing and debugging purposes and should not be used in production
         * runtimes.
         *
         * @returns bool True if the payload was copied to the reception buffer and False otherwise.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * uint8_t test_value = 50;
         * serial_protocol.WriteData(test_value); // Saves the test value to the transmission buffer
         * serial_protocol.CopyTxBufferPayloadToRxBuffer();  // Moves the payload over to the reception buffer
         * @endcode
         */
        bool CopyTxBufferPayloadToRxBuffer()
        {
            // Ensures that the payload to copy fits inside the reception buffer's payload region.
            if (_transmission_buffer[kBufferLayout::kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
            {
                return false;  // If not aborts by returning false
            }

            // Copies the payload from the transmission buffer to the reception buffer. Note, this excludes(!) most
            // metadata and the crc checksum postamble.
            memcpy(
                &_reception_buffer[kBufferLayout::kPayloadStartIndex],
                &_transmission_buffer[kBufferLayout::kPayloadStartIndex],
                _transmission_buffer[kBufferLayout::kPayloadSizeIndex]
            );

            // Updates the payload_size tracker of the reception buffer to match the copied payload size.
            _reception_buffer[kBufferLayout::kPayloadSizeIndex] =
                _transmission_buffer[kBufferLayout::kPayloadSizeIndex];

            return true;  // Returns true to indicate the payload was successfully copied
        }

        /// Returns the size of the payload currently stored in the instance's transmission buffer, in bytes.
        [[nodiscard]]
        uint8_t get_tx_payload_size() const
        {
            return _transmission_buffer[kBufferLayout::kPayloadSizeIndex];
        }

        /// Returns the size of the payload currently stored in the instance's reception buffer, in bytes.
        [[nodiscard]]
        uint8_t get_rx_payload_size() const
        {
            return _reception_buffer[kBufferLayout::kPayloadSizeIndex];
        }

        /// Returns the maximum size of the payload, in bytes, that fits into the instance's transmission buffer.
        static constexpr uint8_t get_maximum_tx_payload_size()
        {
            return kMaximumTransmittedPayloadSize;
        }

        /// Returns the maximum size of the payload, in bytes, that fits into the instance's reception buffer.
        static constexpr uint8_t get_maximum_rx_payload_size()
        {
            return kMaximumReceivedPayloadSize;
        }

        /// Returns the size of the instance's transmission buffer, in bytes.
        static constexpr uint16_t get_tx_buffer_size()
        {
            return kTransmissionBufferSize;
        }

        /// Returns the size of the instance's reception buffer, in bytes.
        static constexpr uint16_t get_rx_buffer_size()
        {
            return kReceptionBufferSize;
        }

        /**
         * @brief Packages the data inside the instance's transmission buffer into a serialized packet and transmits it
         * over the communication interface.
         *
         * @warning This method resets the instance's transmission buffer after transmitting the data, discarding any
         * data stored inside the buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * serial_protocol.SendData();
         * @endcode
         */
        void SendData()
        {
            // Constructs the packet to be transmitted using the data in the transmission buffer.
            const uint16_t combined_size = ConstructPacket();

            // Sends the data over the communication interface.
            _port.write(_transmission_buffer, combined_size);

            // Sets the status to reflect that the data was successfully transmitted.
            runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketSent);

            // Resets the transmission_buffer after every successful transmission.
            ResetTransmissionBuffer();
        }

        /**
         * @brief Receives a data packet from the communication interface, verifies its integrity, and decodes its
         * payload into the instance's reception buffer.
         *
         * Before attempting to receive the packet, the method uses the Available() method to check whether the
         * communication interface is likely to store a well-formed packet. It is safe to call this method cyclically
         * (as part of a loop) until a packet is received.
         *
         * @warning Calling this method resets the instance's reception buffer, discarding any unprocessed data.
         *
         * @note The size of the received payload can be queried using the get_rx_payload_size() method.
         *
         * @returns bool True if the packet was successfully received and unpacked and false otherwise.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * bool data_received = serial_protocol.ReceiveData();
         * @endcode
         */
        bool ReceiveData()
        {
            // Returns 'false' if the number of bytes available for parsing is lower than the minimum expected packet
            // size.
            if (!Available())
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse);
                return false;
            }

            // Resets the reception buffer to prepare for receiving the packet.
            ResetReceptionBuffer();

            // Parses the packet from the bytes stored in the communication interface's reception buffer.
            if (!ParsePacket()) return false;

            // Validates the integrity of the received packet and decodes its payload.
            if (!ValidatePacket()) return false;

            // The packet has been successfully received and is now ready for consumption.
            runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived);
            return true;
        }

        /**
         * @brief Writes the input object's data to the instance's transmission buffer as bytes, starting at the
         * specified index.
         *
         * This method serialized the input object and writes the resultant byte-sequence to the instance's transmission
         * buffer's payload region.
         *
         * @warning If necessary, this method overwrites any existing payload data already stored in the transmission
         * buffer with the input object's data.
         *
         * @note This method only works with the payload region of the transmission buffer.
         *
         * @tparam ObjectType The datatype of the object to write to the transmission buffer.
         * @param object The object to write to the transmission buffer.
         * @param start_index The index inside the transmission buffer's payload region from which to start writing
         * the object's data.
         * @param object_size The size of the object, in bytes.
         *
         * @returns uint16_t The transmission buffer's index immediately following the written object's data or 0 if
         * the method failed to write the object to the transmission buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
         * uint16_t value = 44321;
         * uint8_t array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
         * struct MyStruct
         * {
         *    uint8_t a = 60;
         *    uint16_t b = 12345;
         *    uint32_t c = 1234567890;
         * } test_structure;
         *
         * // Serializes and adds the objects to the transmission buffer.
         * uint16_t next_index = serial_protocol.WriteData(value);
         * uint16_t next_index = serial_protocol.WriteData(array, next_index);
         * uint16_t next_index = serial_protocol.WriteData(test_structure, next_index);
         * @endcode
         */
        template <typename ObjectType>
        uint16_t WriteData(
            const ObjectType& object,
            const uint16_t& start_index = 0,
            const uint16_t& object_size = sizeof(ObjectType)
        )
        {
            // Calculates the total size of the payload in the transmission buffer including the new bytes to be
            // added to the buffer
            const uint16_t payload_size = start_index + object_size;

            // Verifies that the payload region of the buffer has enough space to accommodate the increased payload.
            if (payload_size > kMaximumTransmittedPayloadSize)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError);
                return 0;
            }

            // Shifts the input start index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            uint16_t local_start_index = start_index + kBufferLayout::kPayloadStartIndex;

            // Uses memcpy() to efficiently copy the data into the buffer.
            memcpy(
                static_cast<void*>(&_transmission_buffer[local_start_index]),
                static_cast<const void*>(&object),
                object_size
            );

            // If necessary, updated the payload size tracker to reflect the increased payload size.
            _transmission_buffer[kBufferLayout::kPayloadSizeIndex] =
                max(_transmission_buffer[kBufferLayout::kPayloadSizeIndex], static_cast<uint8_t>(payload_size));

            // Sets the status code to indicate writing to buffer was successful
            runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kObjectWrittenToBuffer);

            // Returns the index immediately following the last updated (overwritten) index (relative to the start
            // of the payload) to caller to support chained method calls.
            return payload_size;
        }

        /**
         * @brief Overwrites the input object's data with the data from the instance's reception buffer.
         *
         * This method deserializes the objects stored in the reception buffer as a sequence of bytes. While it
         * overwrites the data of the input object, this method does not modify the data stored in the reception buffer.
         *
         * @note This method only works with the payload region of the transmission buffer.
         *
         * @tparam ObjectType The datatype of the object to read from the reception buffer.
         * @param object The object to read from the reception buffer.
         * @param start_index The index inside the reception buffer's payload region, from which to start reading the
         * object's data.
         * @param object_size The size of the object, in bytes.
         *
         * @returns uint16_t The index inside the reception buffer's payload region immediately following the last
         * object's data byte or 0 if the method failed to read the object from the reception buffer.
         *
         * Example usage:
         * @code
         * Serial.begin(9600);
         * TransportLayer<uint16_t, 64, 64> tl_class(Serial, 0x1021, 0xFFFF, 0x0000);
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
        uint16_t
        ReadData(ObjectType& object, const uint16_t& start_index = 0, const uint16_t& object_size = sizeof(ObjectType))
        {
            // Calculates the total size of the payload necessary to accommodate reading the object at the specified
            // index
            uint16_t required_size = start_index + object_size;

            // Verifies that the reception buffer has enough bytes to accommodate reading the object.
            if (required_size > _reception_buffer[kBufferLayout::kPayloadSizeIndex])
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError);
                return 0;
            }

            // Shifts the input start_index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            uint16_t local_start_index = start_index + kBufferLayout::kPayloadStartIndex;

            // Uses memcpy() to efficiently copy the data into the object from the _reception_buffer.
            memcpy(
                static_cast<void*>(&object),
                static_cast<const void*>(&_reception_buffer[local_start_index]),
                object_size
            );

            // Sets the status code to indicate reading from the buffer was successful
            runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kObjectReadFromBuffer);

            // Returns the index immediately following the index of the final read byte (relative to the payload)
            // to caller. This index can be used as the next input start_index if multiple read calls are chained
            // together.
            return required_size;
        }

    private:
        /// The reference to the Stream class instance that works with the communication interface.
        Stream& _port;

        /// The COBSProcessor instance used to encode and decode packets using the COBS scheme.
        COBSProcessor _cobs_processor;

        /// The CRCProcessor instance used to calculate CRC checksums for the incoming and outgoing data packets.
        CRCProcessor<PolynomialType> _crc_processor;

        /// The maximum number of microseconds (us) to wait between receiving any two consecutive bytes of the packet
        /// before declaring the packet stale. This prevents the runtime from getting stuck in the reception cycle.
        static constexpr uint32_t kTimeout = 10000;  // 10 ms

        /// Stores the size of the CRC checksum postamble, in bytes.
        static constexpr uint8_t kPostambleSize = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /// Stores the size of the smallest packet expected to be received at runtime, in bytes.
        static constexpr uint16_t kMinimumPacketSize =  // NOLINT(*-dynamic-static-initializers)
            kBufferLayout::kMinimumPayloadSize + kBufferLayout::kOverheadByteIndex + kPostambleSize;

        /// Stores the size of the instance's transmission staging buffer, in bytes.
        static constexpr uint16_t kTransmissionBufferSize =  // NOLINT(*-dynamic-static-initializers)
            kMaximumTransmittedPayloadSize + kBufferLayout::kOverheadByteIndex + 2 + kPostambleSize;

        /// Stores the size of the instance's reception staging buffer, in bytes.
        static constexpr uint16_t kReceptionBufferSize =  // NOLINT(*-dynamic-static-initializers)
            kMaximumReceivedPayloadSize + kBufferLayout::kOverheadByteIndex + 2 + kPostambleSize;

        /// The buffer that stages the payload data before it is transmitted.
        uint8_t _transmission_buffer[kTransmissionBufferSize];

        /// The buffer that stores the received data before it is consumed.
        uint8_t _reception_buffer[kReceptionBufferSize];

        /**
         * @brief Constructs the serialized packet using the payload stored inside the instance's transmission buffer.
         *
         * @returns uint16_t The combined size of the constructed data packet to be transmitted.
         *
         * Example usage:
         * @code
         * uint16_t combined_size = ConstructPacket();
         * @endcode
         */
        uint16_t ConstructPacket()
        {
            // Encodes the payload into a transmittable packet in-place using the COBS algorithm.
            _cobs_processor.EncodePayload(_transmission_buffer);

            // Calculates the CRC checksum for the encoded packet. Writes the calculated CRC checksum to the end
            // of the encoded packet (to the postamble region).
            const uint16_t combined_size = _crc_processor.template CalculateChecksum<false>(_transmission_buffer);

            // Returns the total size of the resolved packet to be transmitted to the caller.
            return combined_size;
        }

        /**
         * @brief Parses the bytes stored in the reception buffer of the communication interface as a serialized packet
         * and stores it in the instance's reception buffer.
         *
         * @returns bool True if the packet was successfully parsed into the instance's reception buffer and False
         * otherwise.
         *
         * Example usage:
         * @code
         * bool success = ParsePacket();
         * @endcode
         */
        bool ParsePacket()
        {
            // Tracks the number of data bytes read from the transmission interface buffer. Initializes to the size of
            // the preamble, as the preamble is discarded as part of the data reception process.
            uint16_t bytes_read = kBufferLayout::kOverheadByteIndex;

            // First, finds the start byte of the packet. The start byte tells the receiver that the following data
            // belongs to a well-formed packet and should be retained for further processing.
            bool start_byte_found = false;
            while (_port.available())
            {
                // The start byte itself is discarded, as it is only used to parse the packet.
                if (_port.read() == kBufferLayout::kStartByte)
                {
                    start_byte_found = true;
                    break;
                }
            }

            // If the start byte was not found, aborts the runtime and returns false to indicate that no data was
            // parsed as no packet was available.
            if (!start_byte_found)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse);
                return false;
            }

            // If the start byte was found, attempts to read the next byte, which should be the payload size byte.
            elapsedMicros timeout_timer = 0;  // Starts the timer used to prevent getting stuck in packet reception.
            bool payload_size_found     = false;
            while (timeout_timer < kTimeout)  // Blocks until timeout is reached or the payload size is resolved
            {
                if (_port.available())
                {
                    // Reads the payload size into the reception buffer.
                    _reception_buffer[kBufferLayout::kPayloadSizeIndex] = _port.read();

                    // If the payload size is outside the expected payload size range, aborts the reception procedure
                    // with an error.
                    if (_reception_buffer[kBufferLayout::kPayloadSizeIndex] < kBufferLayout::kMinimumPayloadSize ||
                        _reception_buffer[kBufferLayout::kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
                    {
                        runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize);
                        return false;
                    }

                    // If the payload size is within allowed limits, advances to packet reception
                    payload_size_found = true;
                    break;
                }
            }

            // If the payload_size byte was not found, aborts the runtime with an error.
            if (!payload_size_found)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPayloadSizeByteNotFound);
                return false;
            }

            // Calculates the size of the packet's data to be received, in bytes. This is the size of the payload and
            // the COBS overhead and delimiter bytes.
            const uint16_t packet_size =
                _reception_buffer[kBufferLayout::kPayloadSizeIndex] + kBufferLayout::kOverheadByteIndex + 2;

            // Parses the incoming packet until the timeout (packet reception stales), an unencoded
            // delimiter byte value is encountered, or the payload is fully received.
            bool delimiter_found = false;
            timeout_timer        = 0;  // Resets the timer
            while (timeout_timer < kTimeout && bytes_read < packet_size)
            {
                if (_port.available())
                {
                    // Saves each processed byte to the instance's reception buffer
                    const uint8_t byte_value      = _port.read();
                    _reception_buffer[bytes_read] = byte_value;
                    bytes_read++;
                    timeout_timer = 0;

                    // Aborts the processing early if a packet delimiter byte is found
                    if (byte_value == kBufferLayout::kDelimiterByte)
                    {
                        delimiter_found = true;
                        break;
                    }
                }
            }

            // Packet reception stalled (timed out)
            if (timeout_timer >= kTimeout && bytes_read < packet_size)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketTimeoutError);
                return false;
            }

            // Delimiter byte was not found (the packet is corrupted)
            if (!delimiter_found)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDelimiterNotFoundError);
                return false;
            }

            // Delimiter byte was found too early (the packet is corrupted)
            if (bytes_read != packet_size)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDelimiterFoundTooEarlyError);
                return false;
            }

            // If the packet was successfully parsed, attempts to parse the CRC postamble. The CRC bytes
            // should be received immediately after receiving the packet delimiter byte.
            const uint16_t postamble_size = packet_size + static_cast<uint16_t>(kPostambleSize);
            timeout_timer                 = 0;
            while (timeout_timer < kTimeout && bytes_read < postamble_size)
            {
                if (_port.available())
                {
                    _reception_buffer[bytes_read] = _port.read();
                    bytes_read++;
                    timeout_timer = 0;
                }
            }

            // Packet reception stalled (timed out)
            if (timeout_timer >= kTimeout)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPostambleTimeoutError);
                return false;
            }

            // The packet is fully parsed (received).
            runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketParsed);
            return true;
        }

        /**
         * @brief Validates the packet parsed by the ParsePacket() method and decodes its payload using the COBS scheme.
         *
         * @returns bool True if the packet's integrity was verified and its payload was decoded and False otherwise.
         *
         * Example usage:
         * @code
         * bool success = ValidatePacket();
         * @endcode
         */
        bool ValidatePacket()
        {
            // Verifies the received data's integrity using its CRC checksum. The method returns 1 if the verification
            // is passed and 0 otherwise.
            if (_crc_processor.template CalculateChecksum<true>(_reception_buffer) == 0)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed);
                return false;
            }

            // If the CRC check succeeds, decodes the payload from the packet. Successful COBS decoder runtime always
            // returns a non-zero payload_size.
            if (_cobs_processor.DecodePayload(_reception_buffer) == 0)
            {
                runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDecodingFailed);
                return false;
            }

            // The packet is verified and decoded.
            return true;
        }
};

#endif  //AXTLMC_TRANSPORT_LAYER_H
