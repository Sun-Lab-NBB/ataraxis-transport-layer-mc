/**
 * @file
 * @brief This file provides the COBSProcessor class, which is used to encode and decode data arrays (buffers) using
 * the Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @section cobs_description Reference Implementation:
 * The implementation in this file is based on the implementation described in the original paper:
 * S. Cheshire and M. Baker, "Consistent overhead byte stuffing," in IEEE/ACM Transactions on Networking, vol. 7,
 * no. 2, pp. 159-172, April 1999, doi: @a 10.1109/90.769765.
 */

#ifndef AXTLMC_COBS_PROCESSOR_H
#define AXTLMC_COBS_PROCESSOR_H

//Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

/**
 * @struct kCOBSProcessorParameters
 * @brief Stores hardcoded Overhead Byte Stuffing (COBS) scheme parameters used by the COBSProcessor class.
 */
struct kCOBSProcessorParameters
{
        static constexpr uint8_t kMinPayloadSize = 1;    ///< Prevents sending or receiving empty payloads
        static constexpr uint8_t kMaxPayloadSize = 254;  ///< Maximum payload size is 255-1 due to COBS specification
        /// The minimum packet size is 3 to accommodate 1 payload byte + overhead and delimiter bytes
        static constexpr uint8_t kMinPacketSize = 3;
        /// The maximum packet size is 256 to accommodate 254 payload bytes plus overhead and delimiter bytes
        static constexpr uint16_t kMaxPacketSize = 256;
        static constexpr uint8_t kDelimiterByte  = 0xFF;  ///< The values used as the encoded packet delimiter
        static constexpr uint8_t kPayloadSizeIndex =
            1;  ///< The index of the payload buffer array's value that stores the payload region size.
        static constexpr uint8_t kOverheadByteIndex =
            2;  ///< The index of the payload buffer array's value that stores the overhead byte value.
};

/**
 * @class COBSProcessor
 *
 * @brief Provides methods for encoding and decoding payloads using Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @note This class is intended to be used by the TransportLayer class and should not be used directly by the end-users.
 *
 * Example instantiation:
 * @code
 * COBSProcessor cobs_class;
 * @endcode
 */
class COBSProcessor
{
    public:
        /// Stores the latest runtime status of the instance. All COBSProcessor runtime status codes are available
        /// from the kCOBSProcessorCodes enumeration.
        uint8_t cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kStandby);

        /**
         * @brief Uses the COBS scheme to encode the input payload into a packet in-place.
         *
         * This method implements a classic COBS encoding scheme as described by the original paper, with minor
         * modifications. It modifies the input buffer in-place and assumes the buffer is configured in
         * a way that supports in-place COBS-encoding.
         *
         * @tparam buffer_size The size of the input buffer array, in bytes.
         * @param payload_buffer The buffer (array) that stores the payload data to be encoded.
         *
         * @returns uint16_t The size of the encoded packet in bytes or 0 if the method fails to encode the payload.
         *
         * Example usage:
         * @code
         * COBSProcessor cobs_class;
         * uint8_t payload_buffer[7] = {0, 4, 0, 1, 2, 3, 4, 0}; // start, payload size, overhead, payload[4], delimiter
         * uint16_t packet_size = cobs_class.EncodePayload(payload_buffer);
         * @endcode
         */
        template <size_t buffer_size>
        uint16_t EncodePayload(uint8_t (&payload_buffer)[buffer_size])
        {
            // Extracts the payload size from the payload_size variable. Expects that this variable is located at index
            // kPayloadSizeIndex of the buffer
            const uint8_t payload_size = payload_buffer[kCOBSProcessorParameters::kPayloadSizeIndex];

            // Also calculates the minimum buffer size that can store the payload and all metadata bytes. This is based
            // on the index of the overhead, which matches the size of the preamble + 2 to account for the overhead and
            // the delimiter bytes mandated by COBS specification.
            const auto minimum_required_buffer_size =
                static_cast<uint16_t>(payload_size + kCOBSProcessorParameters::kOverheadByteIndex + 2);

            // Prevents encoding empty payloads (as it is generally meaningless)
            if (payload_size < static_cast<uint8_t>(kCOBSProcessorParameters::kMinPayloadSize))
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderTooSmallPayloadSize);
                return 0;
            }

            // Prevents encoding too large payloads (due to COBS limitations)
            if (payload_size > static_cast<uint8_t>(kCOBSProcessorParameters::kMaxPayloadSize))
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderTooLargePayloadSize);
                return 0;
            }

            // Prevents encoding if the input buffer size is not enough to accommodate the packet that would be
            // generated by this method. This guards against out-of-bounds memory access.
            if (static_cast<uint16_t>(buffer_size) < minimum_required_buffer_size)
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderPacketLargerThanBuffer);
                return 0;
            }

            // Checks that the input buffer's overhead byte placeholder is set to 0, which is not a valid value for an
            // encoded buffer's overhead byte. An overhead byte set to 0 indicates that the payload inside the buffer is
            // not encoded. This check is used to prevent accidentally running the encoder on already encoded data,
            // which will corrupt it.
            if (payload_buffer[kCOBSProcessorParameters::kOverheadByteIndex] != 0)
            {
                cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadAlreadyEncoded);
                return 0;
            }

            // Tracks discovered delimiter_byte_value instance indices during the loop below to support iterative COBS
            // encoding.
            uint16_t last_delimiter_index = 0;

            // Determines start and end indices for the loop below based on the requested payload_size. Transforms the
            // indices to be buffer-centric and account for the prepended metadata bytes.
            const uint16_t payload_end_index =
                payload_size + kCOBSProcessorParameters::kOverheadByteIndex;  // INCLUSIVE end index

            // Since payload_end_index is inclusive, delimiter index immediately follows the value of that variable.
            const uint16_t kDelimiterIndex = payload_end_index + 1;

            // Appends the delimiter_byte_value to the end of the payload buffer. Usually, this step is carried out at
            // the end of the encoding sequence, but since this method uses a reverse loop, it starts with the newly
            // added delimiter byte.
            payload_buffer[kDelimiterIndex] = kCOBSProcessorParameters::kDelimiterByte;

            // Loops over the requested payload size in reverse and encodes all instances of delimiter_byte using COBS
            // scheme. Specifically, transforms every instance of delimiter_byte_value into a chain of distance-pointers
            // that allow traversing from the prepended overhead_byte to the appended delimiter_byte. To enable this,
            // overhead_byte stores the distance to the first delimiter_byte_value variable, which then is converted to
            // store the distance to the next delimiter_byte_value, all the way until the appended delimiter_byte is
            // reached. This way, the only instance of delimiter_byte_value will be found at the very end of the
            // payload, and the overhead byte, at worst, will store the distance of 255, to point straight to that
            // value.
            for (uint16_t i = payload_end_index; i >= kPayloadStartIndex; --i)
            {
                if (payload_buffer[i] == kCOBSProcessorParameters::kDelimiterByte)
                {
                    if (last_delimiter_index == 0)
                    {
                        // If delimiter_byte_value is encountered and last_delimiter_index is still set to the default
                        // value of 0, computes the distance from index i to the end of the payload + 1. This is the
                        // distance to the delimiter byte value appended to the end of the payload. Overwrites the
                        // variable with the computed distance, encoding it according to the COBS scheme.
                        payload_buffer[i] = kDelimiterIndex - i;
                    }
                    else
                    {
                        // If last_delimiter_index is set to a non-0 value, uses it to calculate the distance from the
                        // evaluated index to the last (encoded) delimiter byte value and overwrites the variable with
                        // that distance value.
                        payload_buffer[i] = last_delimiter_index - i;
                    }

                    // Updates last_delimiter_index with the index of the last encoded variable
                    last_delimiter_index = i;
                }
            }

            // Once all instances of delimiter_byte_value have been encoded, sets the overhead byte (index 0 of buffer)
            // to store the distance to the closest delimiter_byte_value instance. That is either the
            // last_delimiter_index if it is not 0 or payload_end_index+1 if it is 0 (then overhead byte stores the
            // distance to the appended delimiter_byte_value).
            if (last_delimiter_index != 0)
                // -2 to account for overhead index being 2. Converts the absolute index of the last delimiter to the
                // distance to that value from the overhead byte located at index 2
                payload_buffer[kCOBSProcessorParameters::kOverheadByteIndex] =
                    last_delimiter_index - kCOBSProcessorParameters::kOverheadByteIndex;

            // The delimiter byte is found under kDelimiterIndex, so the distance to the appended delimiter_byte_value
            // from the overhead byte located at index kOverheadByteIndex is kDelimiterIndex - kOverheadByteIndex
            else
                payload_buffer[kCOBSProcessorParameters::kOverheadByteIndex] =
                    kDelimiterIndex - kCOBSProcessorParameters::kOverheadByteIndex;

            // Sets the status to indicate that encoding was successful.
            cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadEncoded);

            // Returns the size of the packet accounting for the addition of the overhead byte and the delimiter byte.
            // Once this method is done running, the buffer looks like this:
            // [start byte] [payload_size byte] [overhead byte] [payload] [delimiter byte] [Any remaining data].
            // The maximum total size of the packet is 256 bytes, which becomes 258 if the start and payload size
            // preamble is added to the total size.
            return minimum_required_buffer_size -
                   kCOBSProcessorParameters::kOverheadByteIndex;  // Subtracts the preamble size.

            // Note: The return statement can be statically re-written as payload_size + 2 (overhead and delimiter)
        }

        /**
         * @brief Uses the COBS scheme to decode the payload from the input packet in-place.
         *
         * This method finds and decode all payload data values that were encoded by the COBS encoding scheme stored
         * in the input buffer in-place. It modifies the input buffer in-place and assumes the buffer is configured in
         * a way that supports in-place COBS-decoding.
         *
         * @tparam buffer_size The size of the input buffer, in bytes.
         * @param packet_buffer The buffer (array) that stores the packet data from which to decode the payload.
         *
         * @returns uint16_t The size of the decoded payload in bytes or 0 if the method fails to decode the payload.
         *
         * Example usage:
         * @code
         * COBSProcessor cobs_class;
         * uint8_t packet_buffer[8] = {129, 4, 5, 1, 2, 3, 4, 0};
         * uint16_t payload_size = cobs_class.DecodePayload(storage_buffer);
         * @endcode
         */
        template <size_t buffer_size>
        uint16_t DecodePayload(uint8_t (&packet_buffer)[buffer_size])
        {
            // Extracts payload size expected to be found under index 1 of the input packet_buffer and uses it to
            // calculate the packet size (by adding the overhead and delimiter bytes to the payload size).
            const uint8_t kPayloadSize = packet_buffer[kCOBSProcessorParameters::kPayloadSizeIndex];
            const uint16_t kPacketSize = kPayloadSize + 2;  // Includes overhead and delimiter bytes
            // This is based on the index of the overhead, which matches the size of the preamble + 2 to account for the
            // overhead and the delimiter bytes mandated by COBS specification.
            const uint16_t kMinimumRequiredBufferSize = kPayloadSize + kCOBSProcessorParameters::kOverheadByteIndex + 2;
            const uint16_t kDelimiterIndex            = kPacketSize + 1;

            // Ensures input packets are at least a minimum valid size in length (at least 3: overhead byte, 1 payload
            // byte and delimiter byte).
            if (kPacketSize < static_cast<uint16_t>(kCOBSProcessorParameters::kMinPacketSize))
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderTooSmallPacketSize);
                return 0;
            }

            // Ensures input packets do not exceed the maximum allowed size (up to 256: due to byte-encoding using COBS
            // scheme).
            if (kPacketSize > kCOBSProcessorParameters::kMaxPacketSize)
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderTooLargePacketSize);
                return 0;
            }

            // Ensures that the buffer is large enough to store the declared packet size plus the start byte and payload
            // size bytes. This guards against accessing memory outside the buffer boundaries during runtime.
            if (static_cast<uint16_t>(buffer_size) < kMinimumRequiredBufferSize)
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderPacketLargerThanBuffer);
                return 0;
            }

            // Verifies that the packet's overhead byte is not set to 0. This method resets the overhead byte of
            // decoded buffers to 0 to indicate that the packet has been decoded. Running decoding on the same data
            // twice will corrupt the data, which is avoided via this check.
            if (packet_buffer[kCOBSProcessorParameters::kOverheadByteIndex] == 0)
            {
                cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPacketAlreadyDecoded);
                return 0;
            }

            // Tracks the index inside the packet buffer to be read. Starts reading data from the overhead byte and uses
            // uint16_t to deal with overflows
            uint16_t read_index = kCOBSProcessorParameters::kOverheadByteIndex;

            // Tracks distance to the next delimiter_byte_value. Initializes to the value obtained from reading the
            // overhead byte, which points to the first (or only) occurrence of the delimiter_byte_value in the packet.
            auto next_index = static_cast<uint16_t>(packet_buffer[read_index]);

            // Resets the overhead byte to 0 to indicate that the buffer has been through a decoding cycle, even if the
            // cycle (see below) fails.
            packet_buffer[read_index] = 0;

            // Increments the read_index to point either to the next encoded variable or to the delimiter_byte_value
            // found at the end of the packet.
            read_index += next_index;

            // This loop basically repeats the steps carried out above, but for each encoded variable until the end of
            // the packet is reached. Specifically, it reads the distance encoded in the variable pointed by the
            // read_index, replaces the variable with the delimiter_byte_value and increments read_index by the distance
            // obtained from reading the encoded variable value. If the packet is intact and correctly COBS-encoded,
            // this process will eventually reach the unencoded delimiter_byte_value at the very end of the packet.
            while (read_index < kMinimumRequiredBufferSize)
            {
                // Checks if the value obtained from read_index matches the packet delimiter value
                if (packet_buffer[read_index] == kCOBSProcessorParameters::kDelimiterByte)
                {
                    // If the read_index matches the kDelimiterIndex returns the size of the decoded payload, which is
                    // equal to read_index - kPayloadStartIndex.
                    if (read_index == kDelimiterIndex)
                    {
                        // Sets the status to indicate that encoding was successful.
                        cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadDecoded);

                        // Returns the decoded payload size
                        return kPayloadSize;
                    }

                    // If the delimiter byte was found earlier than expected, this indicates data corruption that was
                    // not caught by the CRC check. In this case, sets the status to the appropriate error code and
                    // breaks the method runtime.
                    cobs_status =
                        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderDelimiterFoundTooEarly);
                    return 0;
                }

                // If the loop has not been broken, updates next_index with the next jump distance by reading the
                // value of the encoded variable
                next_index = packet_buffer[read_index];

                // Restores the original delimiter_byte_value (decodes the variable value)
                packet_buffer[read_index] = kCOBSProcessorParameters::kDelimiterByte;

                // Jumps to the next encoded delimiter byte's position by distance aggregation.
                read_index += next_index;
            }

            // If a point is reached where the read_index does not point at the delimiter_byte_value variable, but is
            // greater than or equal to the minimum_required_buffer_size (at most, >= 258), this means that the packet
            // is in some way malformed. Well-formed packets should always end in delimiter_byte_value reachable by
            // traversing COBS-encoding variables. In this case, sets the error status and returns 0.
            cobs_status =
                static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderUnableToFindDelimiter);
            return 0;
        }

    private:
        /// Stores the first index of the payload, which is expected to always immediately follow the overhead byte
        /// index.
        static constexpr uint8_t kPayloadStartIndex =
            kCOBSProcessorParameters::kOverheadByteIndex + 1;  // NOLINT(*-dynamic-static-initializers)
};

#endif  //AXTLMC_COBS_PROCESSOR_H
