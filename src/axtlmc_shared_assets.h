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
     * @enum kCOBSProcessorCodes
     * @brief Defines the status codes used by the COBSProcessor class.
     *
     * @note All codes in this enumeration must use values between 11 and 50.
     */
    enum class kCOBSProcessorCodes : uint8_t
    {
        kStandby                       = 11,  ///< The value used to initialize the cobs_status variable
        kEncoderTooSmallPayloadSize    = 12,  ///< Encoder failed to encode payload because its size is too small
        kEncoderTooLargePayloadSize    = 13,  ///< Encoder failed to encode payload because its size is too large
        kEncoderPacketLargerThanBuffer = 14,  ///< Encoded payload buffer is too small to accommodate the packet
        kPayloadAlreadyEncoded         = 15,  ///< Cannot encode payload as it is already encoded (overhead value != 0)
        kPayloadEncoded                = 16,  ///< Payload was successfully encoded into a transmittable packet
        kDecoderTooSmallPacketSize     = 17,  ///< Decoder failed to decode the packet because its size is too small
        kDecoderTooLargePacketSize     = 18,  ///< Decoder failed to decode the packet because its size is too large
        kDecoderPacketLargerThanBuffer = 19,  ///< Packet size to be decoded is larger than the storage buffer size
        kDecoderUnableToFindDelimiter  = 20,  ///< Decoder failed to find the delimiter at the end of the packet
        kDecoderDelimiterFoundTooEarly = 21,  ///< Decoder found a delimiter before reaching the end of the packet
        kPacketAlreadyDecoded          = 22,  ///< Cannot decode the packet as it is already decoded (overhead == 0)
        kPayloadDecoded                = 23,  ///< Payload was successfully decoded from the received packet
    };

    /**
     * @enum kCRCProcessorCodes
     * @brief Defines the status codes used by the CRCProcessor class.
     *
     * @note All codes in this enumeration must use values between 51 and 100.
     */
    enum class kCRCProcessorCodes : uint8_t
    {
        kStandby                            = 51,  ///< The value used to initialize the crc_status variable
        kCalculateCRCChecksumBufferTooSmall = 52,  ///< Checksum calculator failed, the packet exceeds buffer space
        kCRCChecksumCalculated              = 53,  ///< Checksum was successfully calculated
        kAddCRCChecksumBufferTooSmall       = 54,  ///< Not enough remaining buffer space to add checksum to buffer
        kCRCChecksumAddedToBuffer           = 55,  ///< Checksum was successfully added to the buffer
        kReadCRCChecksumBufferTooSmall      = 56,  ///< The remaining buffer space is too small to store the checksum
        kCRCChecksumReadFromBuffer          = 57,  ///< Checksum was successfully read from the buffer
    };

    /**
     * @enum kTransportLayerCodes
     * @brief Defines the status codes used by the TransportLayer class.
     *
     * @note All codes in this enumeration must use values between 101 and 150.
     */
    enum class kTransportLayerCodes : uint8_t
    {
        kStandby                     = 101,  ///< The default value used to initialize the transfer_status variable
        kPacketConstructed           = 102,  ///< Packet was successfully constructed
        kPacketSent                  = 103,  ///< Packet was successfully transmitted
        kPacketStartByteFound        = 104,  ///< Packet start byte was found
        kPacketStartByteNotFound     = 105,  ///< Packet start byte was not found in the incoming stream
        kPayloadSizeByteFound        = 106,  ///< Payload size byte was found
        kPayloadSizeByteNotFound     = 107,  ///< Payload size byte was not found in the incoming stream
        kInvalidPayloadSize          = 108,  ///< Received payload size is not valid
        kPacketTimeoutError          = 109,  ///< Packet parsing failed due to stalling (reception timeout)
        kNoBytesToParseFromBuffer    = 110,  ///< Stream class reception buffer had no packet bytes to parse
        kPacketParsed                = 111,  ///< Packet was successfully parsed
        kCRCCheckFailed              = 112,  ///< CRC check failed, the incoming packet is corrupted
        kPacketValidated             = 113,  ///< Packet was successfully validated
        kPacketReceived              = 114,  ///< Packet was successfully received
        kWriteObjectBufferError      = 115,  ///< Not enough space in the buffer payload region to write the object
        kObjectWrittenToBuffer       = 116,  ///< The object has been written to the buffer
        kReadObjectBufferError       = 117,  ///< Not enough bytes in the buffer payload region to read the object from
        kObjectReadFromBuffer        = 118,  ///< The object has been read from the buffer
        kDelimiterNotFoundError      = 119,  ///< Delimiter byte was not found at the end of the packet
        kDelimiterFoundTooEarlyError = 120,  ///< Delimiter byte was found before reaching the end of the packet
        kPostambleTimeoutError       = 121,  ///< The Postamble was not received within the specified time frame
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
