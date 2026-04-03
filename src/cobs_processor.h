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
    public:
        /**
         * @brief Uses the COBS scheme to encode the input payload into a packet in-place.
         *
         * @tparam kBufferSize the size of the input buffer array, in bytes.
         * @param buffer the buffer that stores the payload data to be encoded.
         *
         * @returns the size of the encoded packet, in bytes.
         */
        template <const size_t kBufferSize>
        static uint16_t EncodePayload(uint8_t (&buffer)[kBufferSize])
        {
            // Extracts the payload size from the payload_size variable.
            const uint8_t payload_size = buffer[kBufferLayout::kPayloadSizeIndex];

            // Determines start and end indices for the loop below based on the requested payload_size. Transforms the
            // indices to be buffer-centric and account for the prepended metadata bytes.
            const uint16_t payload_end_index = payload_size + kBufferLayout::kOverheadByteIndex;  // INCLUSIVE end index

            // Since payload_end_index is inclusive, the delimiter index immediately follows the value of that variable.
            const uint16_t delimiter_index = payload_end_index + 1;

            // Appends the delimiter_byte_value to the end of the payload buffer.
            buffer[delimiter_index] = kBufferLayout::kDelimiterByte;

            // Tracks discovered delimiter_byte_value instance indices during the loop below to support iterative COBS
            // encoding.
            uint16_t last_delimiter_index = 0;

            // Loops over the requested payload size in reverse and encodes all instances of delimiter_byte using the
            // COBS scheme.
            for (uint16_t index = payload_end_index; index >= kBufferLayout::kPayloadStartIndex; --index)
            {
                if (buffer[index] == kBufferLayout::kDelimiterByte)
                {
                    if (last_delimiter_index == 0)
                    {
                        // If delimiter_byte_value is encountered and last_delimiter_index is still set to the default
                        // value of 0, computes the distance from the current index to the end of the payload + 1,
                        // which is the distance to the delimiter byte value appended to the end of the payload.
                        buffer[index] = delimiter_index - index;
                    }
                    else
                    {
                        // If last_delimiter_index is set to a non-0 value, uses it to calculate the distance from the
                        // current index to the last (encoded) delimiter byte value and overwrites the variable with
                        // that distance value.
                        buffer[index] = last_delimiter_index - index;
                    }

                    // Updates last_delimiter_index with the index of the last encoded variable
                    last_delimiter_index = index;
                }
            }

            // Once all instances of delimiter_byte_value have been encoded, sets the overhead byte (index 0 of buffer)
            // to store the distance to the closest delimiter_byte_value instance.
            if (last_delimiter_index != 0)
            {
                // Converts the absolute index of the last delimiter to the distance to that value from the overhead
                // byte located at index 2
                buffer[kBufferLayout::kOverheadByteIndex] = last_delimiter_index - kBufferLayout::kOverheadByteIndex;
            }
            else
            {
                // Calculates the distance from the overhead byte to the appended delimiter byte value, since no
                // encoded delimiter bytes were found in the payload.
                buffer[kBufferLayout::kOverheadByteIndex] = delimiter_index - kBufferLayout::kOverheadByteIndex;
            }

            // Returns the size of the packet accounting for the addition of the overhead byte and the delimiter byte.
            return payload_size + 2;
        }

        /**
         * @brief Uses the COBS scheme to decode the payload from the input packet in-place.
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
            const uint16_t packet_size = payload_size + 2;

            // Determines the expected index of the delimiter value
            const uint16_t delimiter_index = packet_size + 1;

            // Tracks the index inside the packet buffer read at each decoding cycle iteration.
            uint16_t read_index = kBufferLayout::kOverheadByteIndex;

            // Tracks distance to the next delimiter_byte_value. Initializes to the value obtained from reading the
            // overhead byte, which points to the first (or only) occurrence of the delimiter_byte_value in the packet.
            auto next_index = static_cast<uint16_t>(buffer[read_index]);

            // Resets the overhead byte to 0 to indicate that the buffer has been through a decoding cycle
            buffer[read_index] = 0;

            // Increments the read_index to point either to the next encoded value or to the delimiter_byte_value
            // found at the end of the packet.
            read_index += next_index;

            // Loops over the encoded values until reaching the unencoded delimiter value at the end of the packet.
            while (read_index <= delimiter_index)
            {
                // Checks if the value obtained from read_index matches the packet delimiter value
                if (buffer[read_index] == kBufferLayout::kDelimiterByte)
                {
                    // If the read_index matches the delimiter_index, returns the size of the decoded payload as
                    // decoding is complete.
                    if (read_index == delimiter_index)
                    {
                        // Returns the decoded payload size
                        return payload_size;
                    }

                    // If the delimiter byte was found earlier than expected, indicates data corruption.
                    return 0;
                }

                // If the loop has not been broken, updates next_index with the next jump distance by reading the
                // value of the encoded variable
                next_index = buffer[read_index];

                // Restores the original delimiter_byte_value (decodes the variable value)
                buffer[read_index] = kBufferLayout::kDelimiterByte;

                // Jumps to the next encoded delimiter byte's position by distance aggregation.
                read_index += next_index;
            }

            // If decoding the packet does not result in reaching the unencoded delimiter, indicates data corruption.
            return 0;
        }
};

#endif  //AXTLMC_COBS_PROCESSOR_H
