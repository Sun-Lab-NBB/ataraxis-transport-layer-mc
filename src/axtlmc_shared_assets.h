/**
 * @file
 * @brief Provides the assets shared between all library components.
 */

#ifndef AXTLMC_SHARED_ASSETS_H
#define AXTLMC_SHARED_ASSETS_H

#include <Arduino.h>

/**
 * @brief Defines the type for a structure that uses the packed memory layout.
 *
 * Applies to all library structures used in receiving or transmitting data.
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
        kStandby                     = 11,  ///< The value used to initialize the status tracker variable.
        kDecodingFailed              = 12,  ///< Unable to decode the payload from the received packet.
        kPacketSent                  = 13,  ///< Packet was successfully transmitted.
        kPayloadSizeByteNotFound     = 14,  ///< Payload size byte was not found in the incoming stream.
        kInvalidPayloadSize          = 15,  ///< Received payload size is not valid.
        kPacketTimeoutError          = 16,  ///< Packet parsing failed due to stalling (reception timeout).
        kNoBytesToParse              = 17,  ///< No parseable packet was found in the Stream reception buffer.
        kPacketParsed                = 18,  ///< Packet was successfully parsed.
        kCRCCheckFailed              = 19,  ///< CRC check failed, the incoming packet is corrupted.
        kPacketReceived              = 20,  ///< Packet was successfully received.
        kWriteObjectBufferError      = 21,  ///< Not enough space in the buffer payload region to write the object.
        kObjectWrittenToBuffer       = 22,  ///< The object has been written to the buffer.
        kReadObjectBufferError       = 23,  ///< Not enough bytes in the buffer payload region to read the object from.
        kObjectReadFromBuffer        = 24,  ///< The object has been read from the buffer.
        kDelimiterNotFoundError      = 25,  ///< Delimiter byte was not found at the end of the packet.
        kDelimiterFoundTooEarlyError = 26,  ///< Delimiter byte was found before reaching the end of the packet.
        kPostambleTimeoutError       = 27,  ///< The Postamble was not received within the specified time frame.
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
            static constexpr uint8_t kMinimumPayloadSize = 1;    ///< Prevents sending or receiving empty payloads.
            static constexpr uint8_t kMaximumPayloadSize = 254;  ///< Caps the maximum size per COBS specification.
            static constexpr uint8_t kMinimumPacketSize  = 3;    ///< The smallest valid COBS-encoded packet, in bytes.
            static constexpr uint16_t kMaximumPacketSize = 256;  ///< The largest valid COBS-encoded packet, in bytes.
            static constexpr uint8_t kDelimiterByte      = 0;    ///< The value used as the encoded packet delimiter.
            static constexpr uint8_t kStartByte          = 129;  ///< The value used as the packet start byte.
            static constexpr uint8_t kStartByteIndex     = 0;    ///< The index of the start byte value.
            static constexpr uint8_t kPayloadSizeIndex   = 1;    ///< The index of the payload size value.
            static constexpr uint8_t kOverheadByteIndex  = 2;    ///< The index of the overhead byte value.
            static constexpr uint8_t kPayloadStartIndex  = 3;    ///< The index of the first payload's data byte.
    };

    // Reimplements standard library type traits for compatibility with Arduino Mega boards, which lack the
    // '<type_traits>' header available on Teensy. Mirrors std:: counterparts to serve as drop-in replacements.

    /**
     * @brief Determines whether two types are the same.
     *
     * Compares two types at compile-time.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     */
    template <typename T, typename U>
    struct is_same
    {
            /// Determines whether the two template type parameters are identical.
            static constexpr bool value = false;
    };

    /**
     * @brief Specializes the 'is_same' structure for identical types.
     *
     * Activates when both type parameters resolve to the same type.
     *
     * @tparam T The type to compare.
     */
    template <typename T>
    struct is_same<T, T>
    {
            /// Determines whether the two template type parameters are identical.
            static constexpr bool value = true;
    };

    /**
     * @brief Provides convenient access to the value of the 'is_same' structure.
     *
     * Enables compile-time type equality checks without explicitly accessing the `value` member of the `is_same`
     * struct.
     *
     * @tparam T The first type.
     * @tparam U The second type.
     */
    template <typename T, typename U>
    constexpr bool is_same_v = is_same<T, U>::value;  // NOLINT(*-dynamic-static-initializers)
}  // namespace axtlmc_shared_assets

#endif  //AXTLMC_SHARED_ASSETS_H
