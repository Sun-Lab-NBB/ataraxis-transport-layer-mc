/**
 * @file
 * @brief This file provides the CRCProcessor class used to verify transmitted data integrity by calculating
 * the Cyclic Redundancy Check (CRC) checksums for the outgoing and incoming data packets.
 *
* @section crc_implementation Reference Implementation:
 * The implementation in this file is based on the implementation described in the original paper:
 * W. W. Peterson and D. T. Brown, "Cyclic Codes for Error Detection," in Proceedings of the IRE, vol. 49, no. 1,
 * pp. 228-235, Jan. 1961, @a doi: 10.1109/JRPROC.1961.287814.
 */

#ifndef AXTLMC_CRC_PROCESSOR_H
#define AXTLMC_CRC_PROCESSOR_H

// Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

using namespace axtlmc_shared_assets;

/**
 * @class CRCProcessor
 * @brief Provides methods for calculating Cyclic Redundancy Check (CRC) checksums and reading them from / writing them
 * to data buffers.
 *
 * @warning This class is intended to be used by the TransportLayer class and should not be used directly by the
 * end-users.
 *
 * @note Each class instance computes a CRC lookup table at initialization. The table reserves 256, 512, or 1024 bytes
 * of memory depending on the PolynomialType template parameter for the entire lifetime of the instance.
 *
 * @tparam PolynomialType The datatype of the CRC polynomial used by the class instance. Valid types are uint8_t,
 * uint16_t, and uint32_t.
 *
 * Example instantiation:
 * @code
 * uint16_t polynomial = 0x1021;
 * uint16_t initial_value = 0xFFFF;
 * uint16_t final_xor_value = 0x0000;
 * CRCProcessor<uin16_t> crc_class(polynomial, initial_value, final_xor_value);
 * @endcode
 */
template <typename PolynomialType>
class CRCProcessor
{
        // Ensures that the class only accepts uint8, 16, or 32 as valid CRC types.
        static_assert(
            is_same_v<PolynomialType, uint8_t> || is_same_v<PolynomialType, uint16_t> ||
                is_same_v<PolynomialType, uint32_t>,
            "CRCProcessor class template PolynomialType argument must be either uint8_t, uint16_t, or uint32_t."
        );

    public:
        /// Stores the latest runtime status of the instance.
        uint8_t crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kStandby);

        /// Stores the CRC lookup table.
        PolynomialType crc_table[256];

        /**
         * @brief Generates a static CRC lookup table used by the instance to speed up future CRC checksum calculations.
         *
         * @param polynomial The polynomial to use for the generation of the CRC lookup table. Currently, only
         * non-reversed polynomials are supported.
         * @param initial_value The value to which the CRC checksum is initialized before calculation.
         * @param final_xor_value The value with which the CRC checksum is XORed after calculation.
         */
        CRCProcessor(
            const PolynomialType polynomial,
            const PolynomialType initial_value,
            const PolynomialType final_xor_value
        ) :
            kInitialValue(initial_value), kFinalXORValue(final_xor_value)
        {
            GenerateCRCTable(polynomial);
        }

        /**
         * @brief Uses class-instance-specific CRC lookup table to calculate the CRC checksum for the specified stretch
         * of data inside the input buffer.
         *
         * This method loops over packet_size bytes starting with start_index inside the buffer and iteratively
         * computes a CRC checksum for the data.
         *
         * The method is primarily intended to be used on the buffers of the TransportLayer class and was
         * constructed to work with their specific layout. However, the method is implemented in a way that it can be
         * flexibly configured for any buffer layout.
         *
         * @note Currently, only supports polynomials that do not require bit-reversal for inputs and outputs.
         *
         * @attention The method returns the generated CRC checksum as the type specified by the PolynomialType template
         * argument of the class. As such, it automatically scales with each supported polynomial type. Make sure that
         * the caller uses the appropriate type (or auto) to handle the returned checksum value.
         *
         * @warning Any value returned by this method is potentially a valid value. To determine if the method runtime
         * was successful or failed, use crc_status variable of the class instance. Unlike for many other methods of
         * this library, the returned value is not meaningful until it is verified using the status code.
         *
         * @tparam buffer_size The size of the input buffer. This value is used to verify that the specified packet_size
         * fits inside the buffer and will not lead to out-of-bounds access. This guards against undefined behavior and
         * potential data corruption.
         * @param buffer The buffer that stores the data to be checksummed. This is intended to be a well-formed and
         * COBS-encoded packet to be sent to the PC using TransportLayer class. That said, the method will
         * checksum any valid byte-array.
         * @param start_index The index that points to the first byte-value of the portion of the data inside the input
         * buffer to be checksummed (where to start check-summing from). This is helpful to limit the checksum
         * calculation if the buffer contains additional data prior to the portion to be checksummed.
         * @param packet_size The byte-size (stretch) of the data to be checksummed. This method works from start_index
         * (see above) inside the input buffer up to the last index defined by this value (packet_size-1). This is
         * helpful if the buffer contains additional data after the data portion to be checksummed.
         *
         * @returns PolynomialType The CRC checksum of the requested data cast to the appropriate type based on the
         * polynomial type (uint8_t, uint16_t, uint32_t). Make sure to use the crc_status class variable to determine
         * the success or failure status of the method based on the byte-code it is set to after the method's runtime.
         * The crc_status can be interpreted using kCRCProcessorCodes enumeration available through shared_assets
         * namespace.
         *
         * Example usage:
         * @code
         * CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
         * uint8_t packet_buffer[5] = {1, 2, 3, 4, 5};
         * uint16_t start_index = 0;
         * uint16_t packet_size = 5;
         * uint16_t checksum = crc_class.CalculateCRCChecksum(packet_buffer, start_index, packet_size);
         * @endcode
         */
        template <size_t buffer_size>
        PolynomialType CalculatePacketCRCChecksum(
            const uint8_t (&buffer)[buffer_size],
            const uint16_t start_index,
            const uint16_t packet_size
        )
        {
            // Ensures that the byte-reading operation will not overflow the buffer or read past the allowed buffer
            // size. Note, uses the start_index to offset the buffer_size before comparing it to the packet_size.
            if (static_cast<uint16_t>(buffer_size) - start_index < packet_size)
            {
                crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kCalculateCRCChecksumBufferTooSmall);

                // NOTE, unlike most other methods, ANY returned value of this method is potentially valid, so 0 here is
                // just a placeholder. If the method returns 0, this DOES NOT mean the method failed its runtime.
                // That can only be determined by using the crc_status.
                return 0;
            }

            // Initializes the checksum to the initial value of the polynomial that was used to generate the crc table.
            // Also uses the template polynomial type to automatically scale the checksum to the correct size.
            PolynomialType crc_checksum = kInitialValue;

            // Loops over each byte inside the packet and iteratively calculates CRC checksum for the packet
            for (uint16_t i = start_index; i < start_index + packet_size; i++)
            {
                // Saves the data byte being processed into a separate variable
                uint8_t data_byte = buffer[i];

                // Calculates the index to retrieve from CRC table. To do so, combines the high byte of the CRC checksum
                // with the (possibly) modified (corrupted) data_byte using bitwise XOR.
                uint8_t table_index = crc_checksum >> 8 * (kCRCByteLength - 1) ^ data_byte;

                // Extracts the byte-specific CRC value from the table using the result of the operation above. The
                // retrieved CRC value from the table is then XORed with the checksum that is shifted back to the
                // original position to generate an updated checksum.
                crc_checksum = crc_checksum << 8 ^ crc_table[table_index];
            }

            // The Final XOR operation may or may not be used (depending on the polynomial). The default polynomial
            // 0x1021 has it set to 0x0000 (0), so it is actually not used. Other polynomials may require this step, so
            // it is kept here for compatibility reasons. The exact algorithmic purpose of the XOR depends on the
            // specific polynomial used.
            crc_checksum ^= kFinalXORValue;

            // Sets the status to indicate runtime success and returns calculated checksum to the caller.
            crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kCRCChecksumCalculated);
            return crc_checksum;
        }

        /**
         * @brief Adds the input crc_checksum to the input buffer, at a position specified by the start_index.
         *
         * This method converts multi-byte CRC checksums into multiple bytes, starting with the most significant byte,
         * and iteratively overwrites the buffer bytes with CRC bytes starting with the start_index.
         *
         * @note The method automatically scales with the byte-size of the PolynomialType that was used as the template
         * argument during class instantiation. As such, make sure that the input checksum uses the same polynomial
         * (and, by extension, the crc checksum) datatype, otherwise unexpected behavior and / or data corruption may
         * occur.
         *
         * @attention This method feeds the data starting with the most significant byte of the multi-byte CRC checksum
         * first. When reading the data from buffer, make sure to use the companion ReadCRCChecksumFromBuffer() to
         * retrieve the data in the appropriate order. Otherwise, the read CRC checksum will be incorrect.
         *
         * @tparam buffer_size The size of the input buffer. This value is used to verify that the CRC checksum will fit
         * inside the buffer if it is written starting at the start_index. This guards against undefined behavior and
         * potential data corruption.
         * @param buffer The buffer to which the CRC should be appended. Generally, this should either be the
         * packet-filled payload buffer or a postamble buffer to be sent right after the packet buffer, depending on
         * the particular packet anatomy used in your transmission protocol.
         * @param start_index The index inside the buffer with which to start writing the CRC checksum. Specifically,
         * the most significant checksum byte will be written to that index and all lower bytes will be trailed behind,
         * until the entire checksum is written.
         * @param crc_checksum The CRC checksum value to be appended to the buffer. The method automatically sets the
         * input type according to class instance PolynomialType template argument (e.g., if the class was initialized
         * with uint8_t PolynomialType, the input crc_checksum will also be cast to uint8_t).
         *
         * @returns uint16_t The size of the buffer occupied by the preceding data and the appended CRC checksum.
         * Ignores any data that may be found after the appended CRC checksum, the returned size will always be equal
         * to the start_index + crc_checksum byte-size. Returns 0 if method runtime fails to indicate no data has been
         * added to the buffer. Use crc_status to determine the particular error that led to runtime failure
         * (or success code if the method succeeds). The status byte-codes can be interpreted using kCRCProcessorCodes
         * enumeration available through shared_assets namespace.
         *
         * Example usage:
         * @code
         * CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
         * uint8_t postamble_buffer[2];
         * uint16_t start_index = 0;
         * uint16_t crc_checksum = 12345; // Non-real example value
         * uint16_t postamble_size = crc_class.AddCRCChecksumToBuffer(postamble_buffer, start_index, crc_checksum);
         * @endcode
         */
        template <size_t buffer_size>
        uint16_t AddCRCChecksumToBuffer(
            uint8_t (&buffer)[buffer_size],
            const uint16_t start_index,
            const PolynomialType crc_checksum
        )
        {
            // Ensures there is enough space in the buffer for the CRC to be added at the start index.
            if (kCRCByteLength > static_cast<uint16_t>(buffer_size) - start_index)
            {
                crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kAddCRCChecksumBufferTooSmall);
                return 0;
            }

            // Appends the CRC checksum to the buffer, starting with the most significant byte (loops over each byte and
            // iteratively adds it to the buffer).
            for (uint16_t i = 0; i < kCRCByteLength; ++i)
            {
                // Extracts the byte from the checksum and inserts it into the buffer. Most of this instruction controls
                // which byte making up the CRC checksum is processed by each iteration of the loop
                buffer[start_index + i] = crc_checksum >> 8 * (kCRCByteLength - i - 1) & 0xFF;
            }

            // Returns the new size of the buffer after appending the CRC checksum to it and also sets the crc_status
            // appropriately.
            crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kCRCChecksumAddedToBuffer);
            return start_index + kCRCByteLength;
        }

        /**
         * @brief Reads the CRC checksum from the input buffer, starting at the start_index.
         *
         * This method loops over the input buffer starting at the stat_index index and extracts and combines the
         * required number of bytes to generate the appropriately sized CRC checksum.
         *
         * @warning Any value returned by this method is potentially a valid value. To determine if the method runtime
         * was successful or failed, use crc_status variable of the class instance. Unlike for many other methods of
         * this library, the returned value is not meaningful until it is verified using the status code.
         *
         * @note The method automatically scales with the byte-size of the PolynomialType that was used as the template
         * argument during class instantiation. As such, make sure that the caller uses appropriately datatyped variable
         * to save the returned crc checksum. Otherwise, unexpected behavior and / or data corruption may occur.
         *
         * @attention This method expects the CRC checksum data stretch to start with the most significant byte of the
         * multi-byte CRC checksum first. When writing the data to buffer, make sure to use the AddCRCChecksumToBuffer()
         * method to write the data in the appropriate order. Otherwise, the read CRC checksum will be incorrect.
         *
         * @tparam buffer_size The size of the input buffer. This value is used to verify that the there are enough
         * bytes available from the start_index to the end of the buffer to accommodate the expected CRC checksum size
         * to be read. This guards against undefined behavior and potential data corruption that would result from
         * attempting to retrieve data outside the buffer boundaries.
         * @param buffer The buffer from which the CRC checksum needs to be extracted. Generally, this should either be
         * the packet buffer or a postamble buffer sent right after the packet buffer, depending on the particular
         * packet anatomy used in your transmission protocol.
         * @param start_index The position inside the buffer to start reading CRC bytes from. The first CRC checksum
         * byte is read from the start_index, and the remaining bytes are extracted iteratively moving toward the end
         * of the buffer.
         *
         * @returns PolynomialType The CRC checksum value cast to the type specified by the class instance
         * PolynomialType template argument (so if the class was initialized with uint8_t PolynomialType, the returned
         * CRC checksum will also use uint8_t, etc.). The returned value itself is not meaningful until it is verified
         * using the status code available through the crc_status variable of the class. The returned status byte-code
         * can be interpreted using the kCRCProcessorCodes enumeration available through shared_assets namespace.
         *
         * Example usage:
         * @code
         * CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
         * uint8_t postamble_buffer[2] = {123, 65}; // Non-real example value encoded by two bytes!
         * uint16_t start_index = 0;
         * uint16_t crc_checksum = crc_class.AddCRCChecksumToBuffer(postamble_buffer, start_index);
         * @endcode
         */
        template <size_t buffer_size>
        PolynomialType ReadCRCChecksumFromBuffer(const uint8_t (&buffer)[buffer_size], const uint16_t start_index)
        {
            // Ensures the CRC checksum to read is fully within the bounds of the buffer
            if (kCRCByteLength > static_cast<uint16_t>(buffer_size) - start_index)
            {
                crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kReadCRCChecksumBufferTooSmall);
                // Note. Similar to checksum calculator method, 0 is a perfectly valid number. The only way to
                // determine the runtime status of the method is to use the crc_status byte-code.
                return 0;
            }

            // Reconstructs the CRC checksum from the buffer
            PolynomialType extracted_crc = 0;  // Initializes to the type specified by the crc polynomial type
            for (uint16_t i = 0; i < kCRCByteLength; ++i)
            {
                // Constructs the CRC checksum from the buffer, starting from the most significant byte and moving
                // towards the least significant byte. This matches the process of how it was appended to the buffer
                // by the AddCRCChecksumToBuffer() or an equivalent PC method.
                extracted_crc |= static_cast<PolynomialType>(buffer[start_index + i]) << 8 * (kCRCByteLength - i - 1);
            }

            // Returns the extracted crc to caller and sets the crc_status appropriately.
            crc_status = static_cast<uint8_t>(kCRCProcessorCodes::kCRCChecksumReadFromBuffer);
            return static_cast<PolynomialType>(extracted_crc);
        }

    private:
        /// Stores the initial value used for the CRC checksum calculation.
        const PolynomialType kInitialValue;

        /// Stores the final XOR value used for the CRC checksum calculation.
        const PolynomialType kFinalXORValue;

        /// Stores the size of the CRC polynomial in bytes.
        static constexpr uint8_t kCRCByteLength = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /**
         * @brief Computes the CRC lookup table for the given polynomial and saves it to the crc_table attribute.
         *
         * Example usage:
         * @code
         * // Assumes that the instance's PolynomialType template argument was set to uint16_t!
         * uint16_t polynomial = 0x1021;
         * GenerateCRCTable(polynomial);
         * @endcode
         */
        void GenerateCRCTable(PolynomialType polynomial)
        {
            // Determines the number of bits in the CRC type
            static constexpr size_t crc_bits = kCRCByteLength * 8;  // NOLINT(*-dynamic-static-initializers)

            // Determines the Most Significant Bit (MSB) mask based on the CRC type
            static constexpr PolynomialType msb_mask =             // NOLINT(*-dynamic-static-initializers)
                static_cast<PolynomialType>(1) << (crc_bits - 1);  // Keep the parentheses to avoid compiler warnings

            // Iterates over each possible value of a byte variable
            for (uint16_t byte = 0; byte < 256; ++byte)
            {
                // Initializes the byte CRC value based on the CRC (Polynomial) datatype
                auto crc = static_cast<PolynomialType>(byte);

                // Shifts the CRC value left by the appropriate number of bits based on the CRC type to align the
                // initial value to the highest byte of the CRC variable.
                if (crc_bits > 8)
                {
                    crc <<= crc_bits - 8;
                }

                // Loops over each of the 8 bits making up the byte-value being processed
                for (uint8_t bit = 0; bit < 8; ++bit)
                {
                    // Checks if the top bit (MSB) is set
                    if (crc & msb_mask)
                    {
                        // If the top bit is set, shifts the crc value left to bring the next bit into the top position,
                        // then XORs it with the polynomial. This simulates polynomial division where bits are checked
                        // from top to bottom.
                        crc = static_cast<PolynomialType>(crc << 1 ^ polynomial);
                    }
                    else
                    {
                        // If the top bit is not set, shifts the crc value left. This moves to the next bit
                        // without changing the current crc value, as division by polynomial wouldn't modify it.
                        crc <<= 1;
                    }
                }

                // Adds the calculated CRC value for the byte to the storage table using byte-value as the key (index).
                // This value is the remainder of the polynomial division of the byte (treated as a CRC-sized number),
                // by the CRC polynomial.
                crc_table[byte] = crc;
            }
        }
};

#endif  //AXTLMC_CRC_PROCESSOR_H
