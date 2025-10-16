/**
 * @file
 * @brief Provides the CRCProcessor class used to verify transmitted data integrity by calculating
 * the Cyclic Redundancy Check (CRC) checksums for the outgoing and incoming data packets.
 *
 * @section crc_implementation Reference Implementation:
 * The implementation in this file is based on the implementation described in the original paper:
 * W. W. Peterson and D. T. Brown, "Cyclic Codes for Error Detection," in Proceedings of the IRE, vol. 49, no. 1,
 * pp. 228-235, Jan. 1961, doi: 10.1109/JRPROC.1961.287814.
 */

#ifndef AXTLMC_CRC_PROCESSOR_H
#define AXTLMC_CRC_PROCESSOR_H

// Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

using namespace axtlmc_shared_assets;

/**
 * @class CRCProcessor
 * @brief Provides methods for calculating Cyclic Redundancy Check (CRC) checksums and using them to verify the
 * integrity of the incoming and outgoing data packets.
 *
 * @warning This class is intended to be used by the TransportLayer class and should not be used directly by the
 * end-users. It makes specific assumptions about the layout and contents of the processed data buffers that are
 * not verified during runtime and must be enforced through the use of the TransportLayer class.
 *
 * @note Each class instance computes a CRC lookup table at initialization. The table reserves 256, 512, or 1024 bytes
 * of memory depending on the type of the CRC polynomial for the entire lifetime of the instance.
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
        // Prevents passing unsupported data types as the PolynomialType parameter.
        static_assert(
            is_same_v<PolynomialType, uint8_t> || is_same_v<PolynomialType, uint16_t> ||
                is_same_v<PolynomialType, uint32_t>,
            "CRCProcessor class template PolynomialType argument must be either uint8_t, uint16_t, or uint32_t."
        );

    public:
        /// Stores the lookup table used to speed up CRC computation at runtime.
        PolynomialType crc_table[256];

        /**
         * @brief Generates the lookup table used by the instance to speed up future CRC checksum calculations.
         *
         * @param polynomial The polynomial to use for the generation of the CRC lookup table. The polynomial must
         * be standard (non-reflected / non-reversed).
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
         * @brief Calculates the checksum for the data stored in the input buffer.
         *
         * Depending on configuration, this method either verifies the data's integrity based on the checksum included
         * with the data or generates and writes the new checksum value to the end of the data's region.
         *
         * @tparam check Determines whether the method is called to verify the incoming packet's data integrity or to
         * generate and write the CRC checksum to the outgoing packet's postamble section.
         * @tparam buffer_size The size of the input buffer.
         * @param buffer The buffer that stores the COBS-encoded packet for which to calculate the checksum.
         *
         * @returns uint16_t The size of the buffer occupied by the packet's data and the appended CRC checksum if
         * the method is called to calculate the new CRC checksum. The value '1' if the method is configured to verify
         * the packet's data integrity and the data is intact and '0' otherwise.
         *
         * Example usage:
         * @code
         * CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
         * uint8_t packet_buffer[5] = {1, 2, 3, 4, 5};
         * uint16_t start_index = 0;
         * uint16_t end_index = 5;
         * bool check_integrity = false;
         * uint16_t checksum = crc_class.CalculateChecksum<check_integrity>(packet_buffer, start_index, end_index);
         * @endcode
         */
        template <bool check, size_t buffer_size>
        uint16_t CalculateChecksum(
            uint8_t (&buffer)[buffer_size]
        )
        {
            // Initializes the checksum to the initial value of the polynomial that was used to generate the crc table
            PolynomialType crc_checksum = kInitialValue;

            // Statically sets the start index to the position of the overhead byte. This specializes this function to
            // work exclusively with the buffers defined in this library, similar to how COBSProcessor's methods are
            // implemented.
            constexpr uint16_t start_index = kBufferLayout::kOverheadByteIndex;

            // When the function is called in the data checking code, the processed data stretch needs to include the
            // CRC checksum postamble. Otherwise, the processed data needs to include just the packet itself.
            constexpr uint16_t adjustment = check ? kCRCByteLength : 0;

            // Uses the start index and the packet size to determine the stretch of data-values to process
            const uint16_t end_index = start_index + buffer[kBufferLayout::kPayloadSizeIndex] + 2 + adjustment;

            // Loops over each byte inside the packet and iteratively calculates CRC checksum for the packet
            for (uint16_t i = start_index; i < end_index; i++)
            {
                // Saves the data byte being processed into a separate variable
                uint8_t data_byte = buffer[i];

                // Calculates the index to retrieve from the CRC table. To do so, combines the high byte of the CRC
                // checksum with the (possibly) modified (corrupted) data_byte using bitwise XOR.
                uint8_t table_index = crc_checksum >> 8 * (kCRCByteLength - 1) ^ data_byte;

                // Extracts the byte-specific CRC value from the table using the result of the operation above. The
                // retrieved CRC value from the table is then XORed with the checksum that is shifted back to the
                // original position to generate an updated checksum.
                crc_checksum = crc_checksum << 8 ^ crc_table[table_index];
            }

            // The Final XOR operation may or may not be used (depending on the polynomial). The exact algorithmic
            // purpose of the XOR depends on the specific polynomial used.
            crc_checksum ^= kFinalXORValue;

            // If the method is called to generate the checksum for the packet, adds the computed checksum to the buffer
            // immediately after the processed packet. Note; the checksum is always appended to the end of the data
            // packet, overwriting any already existing data.
            if (!check)
            {
                // Appends the CRC checksum to the buffer, starting with the most significant byte (loops over each
                // byte and iteratively adds it to the buffer).
                for (uint16_t i = 0; i < kCRCByteLength; ++i)
                {
                    // Extracts the byte from the checksum and inserts it into the buffer. Most of this instruction
                    // controls which byte making up the CRC checksum is processed by each iteration of the loop
                    buffer[end_index + i] = crc_checksum >> 8 * (kCRCByteLength - i - 1) & 0xFF;
                }

                // Returns the total size of the data stored in the buffer, including the newly appended CRC checksum.
                return end_index + kCRCByteLength;
            }

            // If the method is called to verify an existing checksum and running the CRC calculation on the packet
            // and its checksum postamble section returns 0, the data is intact. In this case, returns 1.
            if (crc_checksum == 0) return 1;

            // Otherwise, the data is corrupted. Returns 0.
            return 0;
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
