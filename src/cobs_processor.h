/**
 * @file
 * @brief The header file for the COBSProcessor class, which is used to encode and decode data arrays (buffers) using
 * the Consistent Overhead Byte Stuffing (COBS) scheme.
 *
 * @section cobs_description Description:
 * COBS is a widely used byte-stuffing protocol that ensures a particular byte value is not present in the input data
 * array (payload). In the broader scope of serial communication, COBS is used to force a particular byte value, known
 * as packet delimiter, to only be present at specific points of the transmitted packets, making it suitable for
 * reliably separating (delimiting) packets.
 *
 * For COBS definition, see the original paper:
 * S. Cheshire and M. Baker, "Consistent overhead byte stuffing," in IEEE/ACM Transactions on Networking, vol. 7,
 * no. 2, pp. 159-172, April 1999, doi: @a 10.1109/90.769765.
 *
 * This file contains two methods packaged into the COBSProcessor class namespace:
 * - EncodePayload(): takes in the buffer containing an arbitrary sized payload between 1 and 254 bytes in length and
 * encodes it using COBS.
 * - DecodePayload(): takes in the buffer that contains a COBS-encoded payload between 1 and 254 bytes in length, and
 * restores it to the un-encoded state (decodes it).
 * - Also contains the kCOBSProcessorLimits structure that stores statically defined payload and packet size constrains
 * used for error checking.
 *
 * @section cobs_developer_notes Developer Notes:
 * This class is a helper class used by the main TransportLayer class to encode and decode payloads
 * using COBS scheme. It is not meant to be used on its own and should always be called from the TransportLayer
 * class. The class methods expect particularly formatted and organized inputs to function properly and will behave
 * unexpectedly if input expectations are not met.
 *
 * The decoder method of the class contains two payload integrity checks, which are intended to be a fallback for the
 * CRC checker. Between the CRC and COBS decoding verification, it should be very unlikely that data corruption during
 * transmission is not detected and reported to caller.
 *
 * @note Due to the limitations of transmitting data as byte-values and COBS specifications, the maximum payload size
 * the class can handle is 254 bytes. The payload buffer itself is expected to accommodate at least 258 bytes to account
 * for the start, payload_size, overhead and delimiter bytes, in addition to the 254 data-bytes of the payload. See
 * the method descriptions below or the API documentation for more information. This class will work with any valid
 * payload size (from 1 to 254 bytes). While the maximum payload size is capped at 254, the payloads do not have to be
 * that long.
 *
 * @section cobs_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - shared_assets.h For COBS-related status codes.
 */

#ifndef AXTLMC_COBS_PROCESSOR_H
#define AXTLMC_COBS_PROCESSOR_H

//Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

/**
 * @struct kCOBSProcessorLimits
 * @brief Stores hardcoded COBS encoding parameters that specify packet and payload size limits.
 *
 * These parameters are mostly used for error-checking inputs to COBS processing methods in an effort to minimize
 * the potential to generate invalid packets.
 *
 * @attention It is generally not recommended to change these parameters as they are currently configured to allow
 * any valid input to be COBS-encoded. These parameters only control maximum and minimum input sizes, within these
 * limits the input can be of any supported size. The input itself can be modified through configuring appropriate
 * TransportLayer parameters.
 */
struct kCOBSProcessorLimits
{
        static constexpr uint8_t kMinPayloadSize = 1;    ///< Prevents sending or receiving empty payloads
        static constexpr uint8_t kMaxPayloadSize = 254;  ///< Maximum payload size is 255-1 due to COBS specification
        /// Minimum packet size is 3 to accommodate 1 payload byte + overhead and delimiter bytes
        static constexpr uint8_t kMinPacketSize = 3;
        /// Maximum packet size is 256 to accommodate 254 payload bytes plus overhead and delimiter bytes
        static constexpr uint16_t kMaxPacketSize = 256;
};

/**
 * @class COBSProcessor
 *
 * @brief Provides methods for in-place encoding and decoding input payload arrays with sizes ranging from 1 to 254
 * bytes using the input delimiter byte value and payload_size.
 *
 * @attention This class assumes that the input buffer is configured in a certain way, specified by the template
 * parameters at class instantiation (see below). The use of template parameters allows fine-tuning the class to work
 * for almost any buffer layout. In turn, this supports static buffer instantiation, which is critical for memory
 * efficiency reasons.
 *
 * @note Do not use this class outside the TransportLayer class unless you know what you are doing. It
 * is the job of the TransportLayer class to make sure the data buffer(s) used by the methods of this class
 * are configured appropriately. If buffers are not appropriately sized, this can lead to undefined behavior and data
 * corruption. The class expects the input buffer to reserve the space for the overhead and delimiter bytes flanking
 * the payload region, in addition to any preamble and postamble variables.
 *
 * @tparam kPayloadSizeIndex The index inside the input buffer(s) this class will work with that holds the payload
 * region size (in bytes).
 * @tparam kOverheadByteIndex The index inside the buffer(s) this class will work with that holds the overhead byte
 * placeholder variable. The payload is expected to be found immediately after the overhead byte.
 *
 * Example instantiation:
 * @code
 * kPayloadSizeIndex = 1;
 * kOverheadByteIndex = 2;
 * COBSProcessor<kPayloadSizeIndex, kOverheadByteIndex> cobs_class;
 * @endcode
 */
template <const uint8_t kPayloadSizeIndex = 1, const uint8_t kOverheadByteIndex = 2>
class COBSProcessor
{
    public:
        /// Stores the latest runtime status of the COBSProcessor. This variable is primarily designed to communicate
        /// the specific errors encountered during encoding or decoding in the form of byte-codes taken from the
        /// kCOBSProcessorCodes enumeration (available through axtlmc_shared_assets namespace). Use the communicated
        /// status to precisely determine the runtime status of any class method.
        uint8_t cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kStandby);

        /**
         * @brief Encodes the input payload in-place, according to COBS scheme.
         *
         * Specifically, loops over the payload stored in the input payload_buffer and replaces every instance of the
         * input delimiter_byte_value with the distance to the next delimiter_byte_value or the end of the payload
         * (whichever is closer).
         *
         * Then, updates the overhead byte expected to be found under the index kOverheadByteIndex of the input
         * payload_buffer to store the distance to the nearest delimiter_byte_value or the end of the packet. Finally,
         * inserts an unencoded delimiter_byte_value at the end of the payload (last index of the payload + 1).
         *
         * This implements a classic COBS encoding scheme as described by the original paper. Uses a slightly modified
         * approach of backward-looping over the payload with project-specific heuristics. Instead of dynamically
         * recreating the buffer, works in-place and assumes the buffer is already configured in a way that supports
         * in-place COBS-encoding.
         *
         * @warning Expects the overhead byte placeholder of the input buffer to be set to 0. Otherwise, considers the
         * call an attempt to re-encode already encoded data and aborts with an error to prevent data corruption from
         * re-encoding.
         *
         * @attention This method makes certain assumptions about the layout of the input buffer. To ensure assumptions
         * are met, this method is not intended to be called directly. Instead, it should be called by the
         * TransportLayer class methods that instantiate the appropriate template specification of this processor class.
         *
         * @tparam buffer_size The size of the input buffer array. This size is used to ensure memory modifications
         * carried out using the buffer stay within the bounds of the buffer. This prevents undefined behavior
         * and / or data corruption due to modifying memory allocated for other objects.
         * @param payload_buffer The buffer (array) that holds the payload to be encoded. Should conform to the layout
         * specified by the class template parameters.
         * @param delimiter_byte_value The value between 0 and 255 (any value that fits into uint8_t range) that will be
         * used as packet delimiter. All instances of this value inside the input payload will be eliminated as per COBS
         * scheme. It is highly advised to use the value of 0 (0x00), since this is the only value that the overhead
         * byte cannot be set to. Any other value runs the risk of being present both at the end of the encoded packet
         * (delimiter byte) and the overhead byte position.
         *
         * @returns uint16_t The size of the encoded packet in bytes, which includes the overhead and delimiter byte
         * values. Failed runtimes return 0. Use cobs_status class variable to get specific runtime error code if the
         * method fails (can be interpreted by using kCOBSProcessorCodes enumeration available through shared_assets
         * namespace).
         *
         * Example usage:
         * @code
         * kPayloadSizeIndex = 1;
         * kOverheadByteIndex = 2;
         * COBSProcessor<kPayloadSizeIndex, kOverheadByteIndex> cobs_class;
         * uint8_t payload_buffer[7] = {0, 4, 0, 1, 2, 3, 4, 0}; // start, payload size, overhead, payload[4], delimiter
         * uint8_t delimiter_byte_value = 0;
         * uint16_t packet_size = cobs_class.EncodePayload(payload_buffer, delimiter_byte_value);
         * @endcode
         */
        template <size_t buffer_size>
        uint16_t EncodePayload(uint8_t (&payload_buffer)[buffer_size], const uint8_t delimiter_byte_value)
        {
            // Extracts the payload size from the payload_size variable. Expects that this variable is located at index
            // kPayloadSizeIndex of the buffer
            const uint8_t payload_size = payload_buffer[kPayloadSizeIndex];

            // Also calculates the minimum buffer size that can store the payload and all metadata bytes. This is based
            // on the index of the overhead, which matches the size of the preamble + 2 to account for the overhead and
            // the delimiter bytes mandated by COBS specification.
            const auto minimum_required_buffer_size = static_cast<uint16_t>(payload_size + kOverheadByteIndex + 2);

            // Prevents encoding empty payloads (as it is generally meaningless)
            if (payload_size < static_cast<uint8_t>(kCOBSProcessorLimits::kMinPayloadSize))
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderTooSmallPayloadSize);
                return 0;
            }

            // Prevents encoding too large payloads (due to COBS limitations)
            if (payload_size > static_cast<uint8_t>(kCOBSProcessorLimits::kMaxPayloadSize))
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
            if (payload_buffer[kOverheadByteIndex] != 0)
            {
                cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadAlreadyEncoded);
                return 0;
            }

            // Tracks discovered delimiter_byte_value instance indices during the loop below to support iterative COBS
            // encoding.
            uint16_t last_delimiter_index = 0;

            // Determines start and end indices for the loop below based on the requested payload_size. Transforms the
            // indices to be buffer-centric and account for the prepended metadata bytes.
            const uint16_t payload_end_index = payload_size + kOverheadByteIndex;  // INCLUSIVE end index

            // Since payload_end_index is inclusive, delimiter index immediately follows the value of that variable.
            const uint16_t kDelimiterIndex = payload_end_index + 1;

            // Appends the delimiter_byte_value to the end of the payload buffer. Usually, this step is carried out at
            // the end of the encoding sequence, but since this method uses a reverse loop, it starts with the newly
            // added delimiter byte.
            payload_buffer[kDelimiterIndex] = delimiter_byte_value;

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
                if (payload_buffer[i] == delimiter_byte_value)
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
                payload_buffer[kOverheadByteIndex] = last_delimiter_index - kOverheadByteIndex;

            // The delimiter byte is found under kDelimiterIndex, so the distance to the appended delimiter_byte_value
            // from the overhead byte located at index kOverheadByteIndex is kDelimiterIndex - kOverheadByteIndex
            else payload_buffer[kOverheadByteIndex] = kDelimiterIndex - kOverheadByteIndex;

            // Sets the status to indicate that encoding was successful.
            cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadEncoded);

            // Returns the size of the packet accounting for the addition of the overhead byte and the delimiter byte.
            // Once this method is done running, the buffer looks like this:
            // [start byte] [payload_size byte] [overhead byte] [payload] [delimiter byte] [Any remaining data].
            // The maximum total size of the packet is 256 bytes, which becomes 258 if the start and payload size
            // preamble is added to the total size.
            return minimum_required_buffer_size - kOverheadByteIndex;  // Subtracts the preamble size.

            // Note: The return statement can be statically re-written as payload_size + 2 (overhead and delimiter)
        }

        /**
         * @brief Decodes the payload from the input packet in-place, using COBS scheme.
         *
         * Specifically, uses COBS-derived heuristics to find and decode all values that were encoded by the COBS
         * encoding scheme. To do so, finds the overhead_byte assumed to be located under index kOverheadByteIndex of
         * the input packet_buffer and uses it to traverse the payload by jumping across the distances encoded by each
         * successively sampled variable. During this process, replaces each traversed variable with the input
         * delimiter_byte_value. This process is carried out until the method encounters an unencoded
         * delimiter_byte_value, at which point it, by definition, has reached the end of the packet
         * (provided the packet has been correctly COBS-encoded).
         *
         * The method automatically calculates the packet_size using the payload_size variable expected to be found at
         * index kPayloadSizeIndex and uses it to ensure that the method aborts if it is not able to find
         * delimiter_byte_value at the end of the packet. If this happens, that indicates the data is corrupted and the
         * CRC check (expected to be used together with COBS encoding) failed to recognize the error. As such, this
         * method doubles as a data corruption checker and, together with the CRC check, it makes uncaught data
         * corruption extremely unlikely.
         *
         * @warning Expects the input packet's overhead byte to be set to a value other than 0. If it is set to 0, the
         * method interprets this call as an attempt to decode already decoded data and aborts with an error to prevent
         * data corruption. The method sets the overhead byte of any validly encoded packet to 0 before entering the
         * decoding loop (regardless of decoding result) to indicate decoding has been attempted.
         *
         * @attention This method makes certain assumptions about the layout of the input buffer, which are set using
         * the class template parameters. To ensure assumptions are met, this method is not intended to be called
         * directly. Instead, it should be called by the TransportLayer class methods that instantiate the appropriate
         * specification of this processor class.
         *
         * @tparam buffer_size The size of the input buffer, calculated automatically via template. This ensures that
         * buffer-manipulations are performed within the buffer boundaries. This is crucial for avoiding
         * unexpected behavior and/or data corruption.
         * @param packet_buffer The buffer (array) that holds the COBS-encoded packet to be decoded. Should conform to
         * the layout specified by the class template parameters.
         * @param delimiter_byte_value The value between 0 and 255 (any value that fits into uint8_t range) that was
         * used as packet delimiter. The method assumes that all instances of this value inside the payload are
         * replaced with COBS-encoded values and the only instance of this value is found at the very end of the
         * payload. The only exception to this rule is the overhead byte, the decoder is designed to work around the
         * overhead byte being set to the delimiter_byte_value. This value is used both to restore the encoded variables
         * during decoding forward pass and as an extra corruption-check, as the algorithm expects to only find the
         * instance of this value at the end of the packet.
         *
         * @returns The size of the payload in bytes. Failed runtimes return 0. Use cobs_status class variable to get
         * specific runtime error code if the method fails (can be interpreted by using kCOBSProcessorCodes enumeration
         * available through shared_assets namespace).
         *
         * Example usage:
         * @code
         * kPayloadSizeIndex = 1;
         * kOverheadByteIndex = 2;
         * COBSProcessor<kPayloadSizeIndex, kOverheadByteIndex> cobs_class;
         * uint8_t packet_buffer[8] = {129, 4, 5, 1, 2, 3, 4, 0};
         * uint8_t delimiter_byte_value = 0;
         * uint16_t payload_size = cobs_class.DecodePayload(storage_buffer, delimiter_byte_value);
         * @endcode
         */
        template <size_t buffer_size>
        uint16_t DecodePayload(uint8_t (&packet_buffer)[buffer_size], const uint8_t delimiter_byte_value)
        {
            // Extracts payload size expected to be found under index 1 of the input packet_buffer and uses it to
            // calculate the packet size (by adding the overhead and delimiter bytes to the payload size).
            const uint8_t kPayloadSize = packet_buffer[kPayloadSizeIndex];
            const uint16_t kPacketSize = kPayloadSize + 2;  // Includes overhead and delimiter bytes
            // This is based on the index of the overhead, which matches the size of the preamble + 2 to account for the
            // overhead and the delimiter bytes mandated by COBS specification.
            const uint16_t kMinimumRequiredBufferSize = kPayloadSize + kOverheadByteIndex + 2;
            const uint16_t kDelimiterIndex            = kPacketSize + 1;

            // Ensures input packets are at least a minimum valid size in length (at least 3: overhead byte, 1 payload
            // byte and delimiter byte).
            if (kPacketSize < static_cast<uint16_t>(kCOBSProcessorLimits::kMinPacketSize))
            {
                cobs_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderTooSmallPacketSize);
                return 0;
            }

            // Ensures input packets do not exceed the maximum allowed size (up to 256: due to byte-encoding using COBS
            // scheme).
            if (kPacketSize > kCOBSProcessorLimits::kMaxPacketSize)
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
            if (packet_buffer[kOverheadByteIndex] == 0)
            {
                cobs_status = static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPacketAlreadyDecoded);
                return 0;
            }

            // Tracks the index inside the packet buffer to be read. Starts reading data from the overhead byte and uses
            // uint16_t to deal with overflows
            uint16_t read_index = kOverheadByteIndex;

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
                if (packet_buffer[read_index] == delimiter_byte_value)
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
                packet_buffer[read_index] = delimiter_byte_value;

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
        static constexpr uint8_t kPayloadStartIndex = kOverheadByteIndex + 1;  // NOLINT(*-dynamic-static-initializers)
};

#endif  //AXTLMC_COBS_PROCESSOR_H
