/**
 * @file
 * @brief This file provides the assets shared between all library components.
 */

#ifndef AXTLMC_SHARED_ASSETS_H
#define AXTLMC_SHARED_ASSETS_H

// Dependencies
#include <Arduino.h>

/**
 * @brief Defines the type for a structure that uses the packed memory layout.
 *
 * This structure type is used for creating all library structures used in receiving or transmitting data.
 */
#if defined(__GNUC__) || defined(__clang__)
#define PACKED_STRUCT __attribute__((packed))
#else
#define PACKED_STRUCT
#endif

/**
 * @namespace axtlmc_shared_assets
 * @brief Provides all assets (structures, enumerations, functions) that are intended to be shared between library
 * components.
 */
namespace axtlmc_shared_assets
{
    /**
     * @enum kTransportStatusCodes
     * @brief Defines the codes used by the TransportLayer class to indicate the status of all supported data
     * manipulations.
     */
    enum class kTransportStatusCodes : uint8_t
    {
        kStandby                     = 11,  ///< The value used to initialize the status tracker variable
        kDecodingFailed              = 12,  ///< Unable to decode the payload from the received packet
        kPacketSent                  = 13,  ///< Packet was successfully transmitted
        kPayloadSizeByteNotFound     = 14,  ///< Payload size byte was not found in the incoming stream
        kInvalidPayloadSize          = 15,  ///< Received payload size is not valid
        kPacketTimeoutError          = 16,  ///< Packet parsing failed due to stalling (reception timeout)
        kNoBytesToParse              = 17,  ///< Stream class reception buffer had no packet bytes to parse
        kPacketParsed                = 18,  ///< Packet was successfully parsed
        kCRCCheckFailed              = 19,  ///< CRC check failed, the incoming packet is corrupted
        kPacketReceived              = 20,  ///< Packet was successfully received
        kWriteObjectBufferError      = 21,  ///< Not enough space in the buffer payload region to write the object
        kObjectWrittenToBuffer       = 22,  ///< The object has been written to the buffer
        kReadObjectBufferError       = 23,  ///< Not enough bytes in the buffer payload region to read the object from
        kObjectReadFromBuffer        = 24,  ///< The object has been read from the buffer
        kDelimiterNotFoundError      = 25,  ///< Delimiter byte was not found at the end of the packet
        kDelimiterFoundTooEarlyError = 26,  ///< Delimiter byte was found before reaching the end of the packet
        kPostambleTimeoutError       = 27,  ///< The Postamble was not received within the specified time frame
    };

    /**
     * @struct kBufferLayout
     * @brief Stores the parameters that jointly define the layout and constraints for the data buffers processed by
     * this library.
     *
     * The parameters from this structure are used by all classes exposed by the library to determine how to interact
     * with the input data buffers.
     */
    struct kBufferLayout
    {
            static constexpr uint8_t kMinimumPayloadSize = 1;    ///< Prevents sending or receiving empty payloads
            static constexpr uint8_t kMaximumPayloadSize = 254;  ///< Maximum size is capped due to COBS specification
            static constexpr uint8_t kMinimumPacketSize  = 3;    ///< 1 payload byte, overhead, and delimiter
            static constexpr uint16_t kMaximumPacketSize = 256;  ///< 254 payload bytes, overhead, and delimiter
            static constexpr uint8_t kDelimiterByte      = 0;    ///< The value used as the encoded packet delimiter
            static constexpr uint8_t kStartByte          = 129;  ///< The value used as the packet start byte
            static constexpr uint8_t kStartByteIndex     = 0;    ///< The index of the start byte value
            static constexpr uint8_t kPayloadSizeIndex   = 1;    ///< The index of the payload size value
            static constexpr uint8_t kOverheadByteIndex  = 2;    ///< The index of the overhead byte value
            static constexpr uint8_t kPayloadStartIndex  = 3;    ///< The index of the first payload's data byte
    };

    // Since Arduino Mega (the lower-end board this code was tested with) boards do not have access to the 'cstring'
    // header that is available to Teensy, some assets had to be reimplemented manually. They are implemented in as
    // similar of a way as possible to be drop-in replaceable with std:: namespace.

    /**
     * @brief A type trait that determines if two types are the same.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This struct is used to compare two types at compile-time.
     */
    template <typename T, typename U>
    struct is_same
    {
            /// The default value used by the specification for two different input types.
            static constexpr bool value = false;
    };

    /**
      * @brief A specialization of the 'is_same' structure for the case when both types are the same.
      *
      * @tparam T The type to compare.
      *
      * This specialization is used when both type parameters are the same.
      */
    template <typename T>
    struct is_same<T, T>
    {
            /// The default value used by the specification for two identical types.
            static constexpr bool value = true;
    };

    /**
     * @brief A helper variable template that provides a convenient way to access the value of the 'is_same' structure.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     *
     * This variable template is declared as `constexpr`, allowing it to be used in compile-time expressions. It
     * provides a more concise way to check if two types are the same, without the need to explicitly access the
     * `value` member of the `is_same` struct.
     *
     * Example usage:
     * @code
     * static_assert(is_same_v<int, int>, "int and int are the same");
     * static_assert(!is_same_v<int, float>, "int and float are different");
     * @endcode
     */
    template <typename T, typename U>
    constexpr bool is_same_v = is_same<T, U>::value;  // NOLINT(*-dynamic-static-initializers)
}  // namespace axtlmc_shared_assets

#endif  //AXTLMC_SHARED_ASSETS_H
