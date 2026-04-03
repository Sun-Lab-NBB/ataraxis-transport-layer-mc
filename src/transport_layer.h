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
 * @section tl_packet_anatomy Packet Anatomy:
 * This class sends and receives data in the form of packets. Each packet adheres to the following general layout:
 * [START BYTE] [PAYLOAD SIZE] [OVERHEAD BYTE] [PAYLOAD] [DELIMITER BYTE] [CRC CHECKSUM]
 *
 * @warning This class permanently reserves up to 550 bytes of RAM for the staging buffers and up to 1024 bytes for
 * storing the CRC lookup table. The number of bytes reserved for the staging buffers can be reduced by adjusting the
 * maximum transmission / reception buffer sizes. The number of bytes reserved for the CRC lookup table can be reduced
 * by adjusting the type of the polynomial used for the CRC checksum calculation.
 *
 * @note All user-facing methods only work with the payload portion of the data packet. The rest of the packet anatomy
 * is controlled internally by the TransportLayer instance.
 */

#ifndef AXTLMC_TRANSPORT_LAYER_H
#define AXTLMC_TRANSPORT_LAYER_H

#include <Arduino.h>
#include <elapsedMillis.h>
#include "axtlmc_shared_assets.h"
#include "cobs_processor.h"
#include "crc_processor.h"

using namespace axtlmc_shared_assets;

/// Stores the size of the Serial class reception buffer, in bytes, based on the target microcontroller architecture.

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
 * @brief Exposes methods for sending and receiving serialized data over the USB and UART communication interfaces.
 *
 * This class instantiates and manages all library assets used to transcode, validate, and bidirectionally transfer
 * serial data over the target communication interface.
 *
 * @tparam PolynomialType The datatype of the polynomial to use for Cyclic Redundancy Check (CRC) checksum computations.
 * This parameter indirectly controls the size of the instance's CRC lookup table. Valid types are uint8_t, uint16_t,
 * and uint32_t.
 * @tparam kMaximumTransmittedPayloadSize The maximum size of the payload that is expected to be transmitted during
 * runtime. This parameter indirectly controls the size of the instance's transmission buffer. Must be a value between
 * 1 and 254.
 * @tparam kMaximumReceivedPayloadSize The maximum size of the payload that is expected to be received during runtime.
 * This parameter indirectly controls the size of the instance's reception buffer. Must be a value between 1 and 254.
 */
template <
    typename PolynomialType                      = uint8_t,                          // Defaults to uint8_t polynomials
    const uint8_t kMaximumTransmittedPayloadSize = min(kSerialBufferSize - 8, 254),  // Intelligently caps at 254 bytes
    const uint8_t kMaximumReceivedPayloadSize    = min(kSerialBufferSize - 8, 254)   // Intelligently caps at 254 bytes
    >
class TransportLayer final
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
        /**
         * @brief Initializes all runtime assets that facilitate data transmission and reception.
         *
         * @param communication_port The initialized communication interface instance, such as Serial or USB Serial.
         * @param crc_polynomial The polynomial to use for the generation of the CRC lookup table. The polynomial must
         * be standard (non-reflected / non-reversed).
         * @param crc_initial_value The value to which the CRC checksum is initialized before calculation.
         * @param crc_final_xor_value The value with which the CRC checksum is XORed after calculation.
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
            // Sets the start_byte placeholder to the actual start byte value.
            _transmission_buffer[0] = kBufferLayout::kStartByte;
        }

        /**
         * @brief Evaluates whether the communication interface has received enough bytes to justify reading the
         * incoming packet.
         *
         * @returns true if the communication interface has received enough bytes to likely contain an incoming
         * data packet and false otherwise.
         */
        [[nodiscard]]
        bool Available() const
        {
            return _port.available() >= static_cast<int>(kMinimumPacketSize);
        }

        /// Resets the instance's transmission buffer.
        void ResetTransmissionBuffer()
        {
            _transmission_buffer[kBufferLayout::kPayloadSizeIndex]  = 0;  // Payload Size
            _transmission_buffer[kBufferLayout::kOverheadByteIndex] = 0;  // Overhead Byte
        }

        /// Resets the instance's reception buffer.
        void ResetReceptionBuffer()
        {
            _reception_buffer[kBufferLayout::kPayloadSizeIndex]  = 0;  // Payload Size
            _reception_buffer[kBufferLayout::kOverheadByteIndex] = 0;  // Overhead Byte
            _consumed_payload_bytes                              = 0;  // Also resets the consumed payload bytes counter
        }

        /**
         * @brief Copies the contents of the instance's transmission buffer into the specified destination buffer.
         *
         * @warning This method is intended for testing and debugging purposes and should not be used in production
         * runtimes.
         *
         * @tparam DestinationSize The size of the destination buffer, in bytes.
         * @param destination The buffer where to copy the contents of the transmission buffer.
         */
        template <size_t DestinationSize>
        void CopyTransmissionData(uint8_t (&destination)[DestinationSize])
        {
            static_assert(
                DestinationSize == kTransmissionBufferSize,
                "Destination buffer size must be equal to the instance's transmission buffer size."
            );

            memcpy(destination, _transmission_buffer, DestinationSize);
        }

        /**
         * @brief Copies the contents of the instance's reception buffer into the specified destination buffer.
         *
         * @warning This method is intended for testing and debugging purposes and should not be used in production
         * runtimes.
         *
         * @tparam DestinationSize The size of the destination buffer, in bytes.
         * @param destination The buffer where to copy the contents of the reception buffer.
         */
        template <size_t DestinationSize>
        void CopyReceptionData(uint8_t (&destination)[DestinationSize])
        {
            static_assert(
                DestinationSize == kReceptionBufferSize,
                "Destination buffer size must be equal to the instance's reception buffer size."
            );

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
         * @returns true if the payload was copied to the reception buffer and false otherwise.
         */
        bool CopyTxBufferPayloadToRxBuffer()
        {
            // Ensures that the payload to copy fits inside the reception buffer's payload region.
            if (_transmission_buffer[kBufferLayout::kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
            {
                return false;
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

            return true;
        }

        /// Returns the size of the payload currently stored in the instance's transmission buffer, in bytes.
        [[nodiscard]]
        uint8_t get_bytes_in_transmission_buffer() const
        {
            return _transmission_buffer[kBufferLayout::kPayloadSizeIndex];
        }

        /// Returns the size of the payload currently stored in the instance's reception buffer, in bytes.
        [[nodiscard]]
        uint8_t get_bytes_in_reception_buffer() const
        {
            return _reception_buffer[kBufferLayout::kPayloadSizeIndex];
        }

        /// Returns the maximum size of the payload, in bytes, that fits into the instance's transmission buffer.
        [[nodiscard]]
        static constexpr uint8_t get_maximum_transmitted_payload_size()
        {
            return kMaximumTransmittedPayloadSize;
        }

        /// Returns the maximum size of the payload, in bytes, that fits into the instance's reception buffer.
        [[nodiscard]]
        static constexpr uint8_t get_maximum_received_payload_size()
        {
            return kMaximumReceivedPayloadSize;
        }

        /// Returns the size of the instance's transmission buffer, in bytes.
        [[nodiscard]]
        static constexpr uint16_t get_transmission_buffer_size()
        {
            return kTransmissionBufferSize;
        }

        /// Returns the size of the instance's reception buffer, in bytes.
        [[nodiscard]]
        static constexpr uint16_t get_reception_buffer_size()
        {
            return kReceptionBufferSize;
        }

        /// Returns the runtime status of the most recently called method.
        [[nodiscard]]
        uint8_t get_runtime_status() const
        {
            return _runtime_status;
        }

        /**
         * @brief Packages the data inside the instance's transmission buffer into a serialized packet and transmits it
         * over the communication interface.
         *
         * @warning This method resets the instance's transmission buffer after transmitting the data, discarding any
         * data stored inside the buffer.
         */
        void SendData()
        {
            const uint16_t combined_size = ConstructPacket();
            _port.write(_transmission_buffer, combined_size);
            _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketSent);
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
         * @note The size of the received payload can be queried using the get_bytes_in_reception_buffer() method.
         *
         * @returns true if the packet was successfully received and unpacked and false otherwise.
         */
        bool ReceiveData()
        {
            if (!Available())
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse);
                return false;
            }

            ResetReceptionBuffer();

            if (!ParsePacket()) return false;

            if (!ValidatePacket()) return false;

            _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived);
            return true;
        }

        /**
         * @brief Serializes and writes the input object's data to the end of the payload stored in the instance's
         * transmission buffer.
         *
         * @tparam ObjectType The datatype of the object to write to the transmission buffer.
         * @param object The object to write to the transmission buffer.
         * @param object_size The size of the object, in bytes.
         * @returns true if the method successfully writes the object's data to the transmission buffer and false
         * otherwise.
         */
        template <typename ObjectType>
        bool WriteData(const ObjectType& object, const uint16_t object_size = sizeof(ObjectType))
        {
            // Computes the index at which to start writing the input object's bytes based on the size of the payload
            // already stored inside the buffer.
            const auto start_index = static_cast<uint16_t>(_transmission_buffer[kBufferLayout::kPayloadSizeIndex]);

            // Calculates the total size of the payload in the transmission buffer including the new bytes to be
            // added to the buffer.
            const uint16_t payload_size = start_index + object_size;

            // Verifies that the payload region of the buffer has enough space to accommodate the increased payload.
            if (payload_size > kMaximumTransmittedPayloadSize)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError);
                return false;
            }

            // Shifts the global start index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            const uint16_t local_start_index = start_index + kBufferLayout::kPayloadStartIndex;

            memcpy(
                static_cast<void*>(&_transmission_buffer[local_start_index]),
                static_cast<const void*>(&object),
                object_size
            );

            // Updates the payload size tracker to reflect the increased payload size.
            _transmission_buffer[kBufferLayout::kPayloadSizeIndex] =
                max(_transmission_buffer[kBufferLayout::kPayloadSizeIndex], static_cast<uint8_t>(payload_size));

            _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kObjectWrittenToBuffer);
            return true;
        }

        /**
         * @brief Overwrites the input object's data with the data from the instance's reception buffer, consuming
         * (discarding) all read bytes.
         *
         * This method deserializes the objects stored in the reception buffer as a sequence of bytes. Calling this
         * method consumes the read bytes, making it impossible to retrieve the same data from the reception buffer
         * again.
         *
         * @tparam ObjectType The datatype of the object to read from the reception buffer.
         * @param object The object to read from the reception buffer.
         * @param object_size The size of the object, in bytes.
         * @returns true if the method successfully reads the object's data from the reception buffer and false
         * otherwise.
         */
        template <typename ObjectType>
        bool ReadData(ObjectType& object, const uint16_t object_size = sizeof(ObjectType))
        {
            // Computes the index at which to start reading the input object's data based on the number of bytes already
            // consumed from the buffer.
            const uint16_t start_index = _consumed_payload_bytes;

            // Calculates the total size of the payload necessary to accommodate reading the object at the specified
            // index.
            const uint16_t required_size = start_index + object_size;

            // Verifies that the reception buffer has enough bytes to accommodate reading the object.
            if (required_size > _reception_buffer[kBufferLayout::kPayloadSizeIndex])
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError);
                return false;
            }

            // Shifts the input start_index to translate it from payload-centric to buffer-centric. The buffer contains
            // multiple metadata variables not exposed to the user, so any index that is relative to the payload has to
            // be converted to account for the preceding metadata bytes.
            const uint16_t local_start_index = start_index + kBufferLayout::kPayloadStartIndex;

            memcpy(
                static_cast<void*>(&object),
                static_cast<const void*>(&_reception_buffer[local_start_index]),
                object_size
            );

            // Updates the consumed payload bytes tracker to reflect data consumption.
            _consumed_payload_bytes += object_size;

            _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kObjectReadFromBuffer);
            return true;
        }

    private:
        /// The maximum number of microseconds (us) to wait between receiving any two consecutive bytes of the packet
        /// before declaring the packet stale. This prevents the runtime from getting stuck in the reception cycle.
        static constexpr uint32_t kTimeout = 10000;  // 10 ms

        /// Stores the size of the CRC checksum postamble, in bytes.
        static constexpr uint8_t kPostambleSize = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /// Stores the size of the smallest packet expected to be received at runtime, in bytes.
        static constexpr uint16_t kMinimumPacketSize =
            kBufferLayout::kMinimumPayloadSize + kBufferLayout::kOverheadByteIndex
            + kPostambleSize;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the size of the instance's transmission staging buffer, in bytes.
        static constexpr uint16_t kTransmissionBufferSize =
            kMaximumTransmittedPayloadSize + kBufferLayout::kOverheadByteIndex + 2
            + kPostambleSize;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the size of the instance's reception staging buffer, in bytes.
        static constexpr uint16_t kReceptionBufferSize =
            kMaximumReceivedPayloadSize + kBufferLayout::kOverheadByteIndex + 2
            + kPostambleSize;  // NOLINT(*-dynamic-static-initializers)

        // Ensures that the requested transmission buffer does not exceed the microcontroller's serial buffer size.
        static_assert(
            kTransmissionBufferSize <= kSerialBufferSize,
            "TransportLayer's transmission buffer size exceeds the serial buffer size for this type of "
            "microcontroller boards. If you manually increased the serial buffer size, edit the preprocessor "
            "directives at the top of the transport_layer.h file. Otherwise, set the "
            "kMaximumTransmittedPayloadSize template argument of the TransportLayer class to the value appropriate "
            "for your microcontroller board. Note, in addition to the payload, the buffer may need up to 8 extra "
            "bytes to store packet metadata."
        );

        // Ensures that the requested reception buffer does not exceed the microcontroller's serial buffer size.
        static_assert(
            kReceptionBufferSize <= kSerialBufferSize,
            "TransportLayer's reception buffer size exceeds the serial buffer size for this type for "
            "microcontroller boards. If you manually increased the serial buffer size, edit the preprocessor "
            "directives at the top of the transport_layer.h file. Otherwise, set the "
            "kMaximumReceivedPayloadSize template argument of the TransportLayer class to the value appropriate "
            "for your microcontroller board. Note, in addition to the payload, the buffer may need up to 8 extra "
            "bytes to store packet metadata."
        );

        /// The reference to the Stream class instance that works with the communication interface.
        Stream& _port;

        /// The COBSProcessor instance used to encode and decode packets using the COBS scheme.
        COBSProcessor _cobs_processor;

        /// The CRCProcessor instance used to calculate CRC checksums for the incoming and outgoing data packets.
        CRCProcessor<PolynomialType> _crc_processor;

        /// The buffer that stages the payload data before it is transmitted.
        uint8_t _transmission_buffer[kTransmissionBufferSize];

        /// The buffer that stores the received data before it is consumed.
        uint8_t _reception_buffer[kReceptionBufferSize];

        /// Tracks the number of received payload bytes that have been consumed via ReadData().
        uint16_t _consumed_payload_bytes = 0;

        /// Stores the runtime status of the most recently called method.
        uint8_t _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kStandby);

        /**
         * @brief Constructs the serialized packet using the payload stored inside the instance's transmission buffer.
         *
         * @returns the combined size of the constructed data packet to be transmitted.
         */
        uint16_t ConstructPacket()
        {
            // Encodes the payload into a transmittable packet in-place using the COBS algorithm.
            _cobs_processor.EncodePayload(_transmission_buffer);

            // Calculates the CRC checksum for the encoded packet and writes it to the postamble region.
            const uint16_t combined_size = _crc_processor.template CalculateChecksum<false>(_transmission_buffer);

            return combined_size;
        }

        /**
         * @brief Parses the bytes stored in the reception buffer of the communication interface as a serialized packet
         * and stores it in the instance's reception buffer.
         *
         * @returns true if the packet was successfully parsed into the instance's reception buffer and false
         * otherwise.
         */
        bool ParsePacket()
        {
            // Tracks the number of data bytes read from the transmission interface buffer. Initializes to the size of
            // the preamble, as the preamble is discarded as part of the data reception process.
            uint16_t bytes_read = kBufferLayout::kOverheadByteIndex;

            // Finds the start byte of the packet. The start byte tells the receiver that the following data belongs
            // to a well-formed packet and should be retained for further processing.
            bool start_byte_found = false;
            while (_port.available())
            {
                if (_port.read() == kBufferLayout::kStartByte)
                {
                    start_byte_found = true;
                    break;
                }
            }

            if (!start_byte_found)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse);
                return false;
            }

            // Attempts to read the payload size byte, blocking until timeout is reached or the byte is resolved.
            elapsedMicros timeout_timer = 0;
            bool payload_size_found     = false;
            while (timeout_timer < kTimeout)
            {
                if (_port.available())
                {
                    _reception_buffer[kBufferLayout::kPayloadSizeIndex] = _port.read();

                    // Aborts with an error if the payload size is outside the expected range.
                    if (_reception_buffer[kBufferLayout::kPayloadSizeIndex] < kBufferLayout::kMinimumPayloadSize ||
                        _reception_buffer[kBufferLayout::kPayloadSizeIndex] > kMaximumReceivedPayloadSize)
                    {
                        _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize);
                        return false;
                    }

                    payload_size_found = true;
                    break;
                }
            }

            if (!payload_size_found)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPayloadSizeByteNotFound);
                return false;
            }

            // Calculates the size of the packet's data to be received, in bytes. This is the size of the payload and
            // the COBS overhead and delimiter bytes.
            const uint16_t packet_size =
                _reception_buffer[kBufferLayout::kPayloadSizeIndex] + kBufferLayout::kOverheadByteIndex + 2;

            // Parses the incoming packet until the timeout (packet reception stales), an unencoded
            // delimiter byte value is encountered, or the payload is fully received.
            bool delimiter_found = false;
            timeout_timer        = 0;
            while (timeout_timer < kTimeout && bytes_read < packet_size)
            {
                if (_port.available())
                {
                    const uint8_t byte_value      = _port.read();
                    _reception_buffer[bytes_read] = byte_value;
                    bytes_read++;
                    timeout_timer = 0;

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
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketTimeoutError);
                return false;
            }

            // Delimiter byte was not found (the packet is corrupted)
            if (!delimiter_found)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDelimiterNotFoundError);
                return false;
            }

            // Delimiter byte was found too early (the packet is corrupted)
            if (bytes_read != packet_size)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDelimiterFoundTooEarlyError);
                return false;
            }

            // Parses the CRC postamble. The CRC bytes should be received immediately after the packet delimiter byte.
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
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPostambleTimeoutError);
                return false;
            }

            _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kPacketParsed);
            return true;
        }

        /**
         * @brief Validates the packet parsed by the ParsePacket() method and decodes its payload using the COBS scheme.
         *
         * @returns true if the packet's integrity was verified and its payload was decoded and false otherwise.
         */
        bool ValidatePacket()
        {
            // Verifies the received data's integrity using its CRC checksum.
            if (_crc_processor.template CalculateChecksum<true>(_reception_buffer) == 0)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed);
                return false;
            }

            // Decodes the payload from the packet using the COBS algorithm.
            if (_cobs_processor.DecodePayload(_reception_buffer) == 0)
            {
                _runtime_status = static_cast<uint8_t>(kTransportStatusCodes::kDecodingFailed);
                return false;
            }

            return true;
        }
};

#endif  //AXTLMC_TRANSPORT_LAYER_H
