/**
 * @file
 *
 * @brief Provides the COBSProcessor class used to encode and decode data payloads during transmission using
 * the Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @section cobs_implementation Reference Implementation:
 * The implementation in this file is based on the implementation described in the original paper:
 * S. Cheshire and M. Baker, "Consistent overhead byte stuffing," in IEEE/ACM Transactions on Networking, vol. 7,
 * no. 2, pp. 159-172, April 1999, doi: 10.1109/90.769765.
 */

#ifndef AXTLMC_COBS_PROCESSOR_H
#define AXTLMC_COBS_PROCESSOR_H

// Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

using namespace axtlmc_shared_assets;

/**
 * @brief Provides methods for encoding and decoding payloads using the Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @warning This class is intended to be used by the TransportLayer class and should not be used directly by the
 * end-users. It makes specific assumptions about the layout and contents of the processed data buffers that are
 * not verified during runtime and must be enforced through the use of the TransportLayer class.
 */
class COBSProcessor final
{
        // Guarantees that every distance value the encoder computes fits into the byte it overwrites. The largest is
        // the overhead byte's distance to the delimiter appended past the payload, which is kMaximumPayloadSize + 1.
        static_assert(
            kBufferLayout::kMaximumPayloadSize + 1 <= UINT8_MAX,
            "COBSProcessor requires the maximum payload size to keep every COBS distance value within a single byte."
        );

    public:
        /**
         * @brief Uses the COBS scheme to encode the input payload into a packet in-place.
         *
         * @note This method performs no bounds checking. The payload size stored at the payload-size index must be
         * within the valid range, and the buffer must have room for the appended overhead and delimiter bytes.
         *
         * @tparam kBufferSize the size of the input buffer array, in bytes.
         * @param buffer the buffer that stores the payload data to be encoded.
         *
         * @returns the size of the encoded packet, in bytes.
         */
        template <const size_t kBufferSize>
        static uint16_t EncodePayload(uint8_t (&buffer)[kBufferSize])
        {
            const uint8_t payload_size = buffer[kBufferLayout::kPayloadSizeIndex];

            // Determines start and end indices for the loop below based on the requested payload_size. Transforms the
            // indices to be buffer-centric and account for the prepended metadata bytes.
            const size_t payload_end_index = payload_size + kBufferLayout::kOverheadByteIndex;  // INCLUSIVE end index

            // Since payload_end_index is inclusive, the delimiter index immediately follows the value of that variable.
            const size_t delimiter_index = payload_end_index + 1;

            buffer[delimiter_index] = kBufferLayout::kDelimiterByte;

            // Tracks the index of the delimiter byte encoded most recently. Seeding the tracker with the delimiter
            // byte appended past the payload makes the first encoded delimiter measure its distance against that
            // byte, which is the distance the COBS scheme requires at that position.
            size_t last_delimiter_index = delimiter_index;

            // Loops over the requested payload size in reverse and encodes all instances of the delimiter byte
            // using the COBS scheme.
            for (size_t index = payload_end_index; index >= kBufferLayout::kPayloadStartIndex; --index)
            {
                if (buffer[index] == kBufferLayout::kDelimiterByte)
                {
                    // Overwrites the delimiter byte with the distance to the delimiter byte encoded after it, which
                    // is the chain the decoder follows from one encoded delimiter to the next.
                    buffer[index]        = static_cast<uint8_t>(last_delimiter_index - index);
                    last_delimiter_index = index;
                }
            }

            // Once all delimiter bytes have been encoded, sets the overhead byte (index 2 of buffer) to store the
            // distance to the closest encoded delimiter byte. A payload holding no delimiter bytes leaves the tracker
            // at the appended delimiter byte, which is the distance the scheme requires in that case.
            buffer[kBufferLayout::kOverheadByteIndex] =
                static_cast<uint8_t>(last_delimiter_index - kBufferLayout::kOverheadByteIndex);

            // Returns the size of the COBS-encoded frame, accounting for the added overhead byte and delimiter byte.
            return payload_size + 2;
        }

        /**
         * @brief Uses the COBS scheme to decode the payload from the input packet in-place.
         *
         * @note A return value of 0 indicates packet corruption, whether the delimiter is encountered early or never
         * reached.
         *
         * @tparam kBufferSize the size of the input buffer, in bytes.
         * @param buffer the buffer that stores the packet data from which to decode the payload.
         *
         * @returns the size of the decoded payload in bytes, or 0 if the method fails to decode the payload.
         */
        template <const size_t kBufferSize>
        static uint16_t DecodePayload(uint8_t (&buffer)[kBufferSize])
        {
            // Extracts payload size and uses it to calculate the packet size by adding the overhead and delimiter
            // bytes to the payload size.
            const uint8_t payload_size = buffer[kBufferLayout::kPayloadSizeIndex];
            const size_t packet_size   = payload_size + 2;

            const size_t delimiter_index = packet_size + 1;

            // Tracks the index inside the packet buffer read at each decoding cycle iteration.
            size_t read_index = kBufferLayout::kOverheadByteIndex;

            // Tracks distance to the next delimiter byte. Initializes to the value obtained from reading the
            // overhead byte, which points to the first (or only) occurrence of the delimiter byte in the packet.
            auto next_index = static_cast<size_t>(buffer[read_index]);

            // Resets the overhead byte to 0 to indicate that the buffer has been through a decoding cycle.
            buffer[read_index] = 0;

            // Increments the read_index to point either to the next encoded value or to the delimiter byte
            // found at the end of the packet.
            read_index += next_index;

            // Loops over the encoded values until reaching the unencoded delimiter value at the end of the packet.
            while (read_index <= delimiter_index)
            {
                if (buffer[read_index] == kBufferLayout::kDelimiterByte)
                {
                    // If the read_index matches the delimiter_index, returns the size of the decoded payload as
                    // decoding is complete.
                    if (read_index == delimiter_index)
                    {
                        return payload_size;
                    }

                    // If the delimiter byte was found earlier than expected, indicates data corruption.
                    return 0;
                }

                next_index = buffer[read_index];

                // Restores the original delimiter byte (decodes the variable value).
                buffer[read_index] = kBufferLayout::kDelimiterByte;

                // Jumps to the next encoded delimiter byte's position by distance aggregation.
                read_index += next_index;
            }

            // If decoding the packet does not result in reaching the unencoded delimiter, indicates data corruption.
            return 0;
        }
};

#endif  // AXTLMC_COBS_PROCESSOR_H
