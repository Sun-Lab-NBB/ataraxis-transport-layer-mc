/**
 * @file
 *
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
 */
template <typename PolynomialType>
class CRCProcessor final
{
        // Prevents passing unsupported data types as the PolynomialType parameter.
        static_assert(
            is_same_v<PolynomialType, uint8_t> || is_same_v<PolynomialType, uint16_t> ||
                is_same_v<PolynomialType, uint32_t>,
            "CRCProcessor class template PolynomialType argument must be either uint8_t, uint16_t, or uint32_t."
        );

    public:
        /**
         * @brief Generates the lookup table used by the instance to speed up future CRC checksum calculations.
         *
         * @note The implementation is MSB-first (left-shifting). The polynomial, initial value, and final XOR value
         * must all be expressed in standard non-reflected, MSB-aligned form.
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
            _initial_value(initial_value), _final_xor_value(final_xor_value)
        {
            GenerateCRCTable(polynomial);
        }

        /**
         * @brief Calculates the checksum for the data stored in the input buffer.
         *
         * Depending on configuration, this method either verifies the data's integrity based on the checksum included
         * with the data or generates and writes the new checksum value to the end of the data's region.
         *
         * @tparam kCheck Determines whether the method is called to verify the incoming packet's data integrity or to
         * generate and write the CRC checksum to the outgoing packet's postamble section.
         * @tparam kBufferSize The size of the input buffer.
         * @param buffer The buffer that stores the COBS-encoded packet for which to calculate the checksum. The buffer
         * must conform to kBufferLayout, with a valid payload-size byte read to determine the processing range.
         *
         * @returns the total number of bytes occupied in the buffer, including the appended CRC checksum, when
         * generating a new checksum. Returns '1' when verifying data integrity and the data is intact, and '0'
         * otherwise.
         */
        template <const bool kCheck, const size_t kBufferSize>
        uint16_t CalculateChecksum(uint8_t (&buffer)[kBufferSize])
        {
            // Initializes the checksum to the initial value of the polynomial used to generate the CRC table.
            PolynomialType crc_checksum = _initial_value;

            // Sets the start index to the position of the overhead byte. This specializes the function to work
            // exclusively with the buffers defined in this library, similar to how COBSProcessor's methods are
            // implemented.
            constexpr uint16_t start_index = kBufferLayout::kOverheadByteIndex;

            // Adjusts the end index to include the CRC checksum postamble when verifying data integrity, or to
            // include just the packet itself when generating a new checksum.
            constexpr uint16_t adjustment = kCheck ? kCRCByteLength : 0;

            // Calculates the end index from the start index, the payload size byte, the fixed overhead and delimiter
            // bytes, and the CRC postamble length when verifying.
            const uint16_t end_index = start_index + buffer[kBufferLayout::kPayloadSizeIndex] + 2 + adjustment;

            // Iteratively calculates the CRC checksum for each byte inside the packet.
            for (uint16_t i = start_index; i < end_index; i++)
            {
                // Extracts the data byte being processed into a separate variable.
                const uint8_t data_byte = buffer[i];

                // Combines the high byte of the CRC checksum with the data byte using bitwise XOR to calculate the
                // lookup table index.
                const uint8_t table_index = crc_checksum >> 8 * (kCRCByteLength - 1) ^ data_byte;

                // Retrieves the byte-specific CRC value from the table and XORs it with the shifted checksum to
                // produce an updated checksum.
                crc_checksum = crc_checksum << 8 ^ _crc_table[table_index];
            }

            // Applies the final XOR operation to the checksum. The exact algorithmic purpose depends on the specific
            // polynomial used.
            crc_checksum ^= _final_xor_value;

            // Appends the computed checksum to the buffer immediately after the processed packet when generating a new
            // checksum. The checksum always overwrites any already existing data at the target position.
            if (!kCheck)
            {
                // Iteratively appends each byte of the CRC checksum to the buffer, starting with the most significant
                // byte.
                for (uint16_t i = 0; i < kCRCByteLength; ++i)
                {
                    // Extracts the byte at the current offset from the checksum and inserts it into the buffer.
                    buffer[end_index + i] = crc_checksum >> 8 * (kCRCByteLength - i - 1) & 0xFF;
                }

                // Returns the total size of the data stored in the buffer, including the newly appended CRC checksum.
                return end_index + kCRCByteLength;
            }

            // Returns 1 if the CRC calculation on the packet and its checksum postamble yields 0, indicating the data
            // is intact. Returns 0 otherwise, indicating data corruption.
            if (crc_checksum == 0) return 1;

            return 0;
        }

        /// Returns a const pointer to the CRC lookup table used by the instance.
        [[nodiscard]]
        const PolynomialType* get_crc_table() const
        {
            return _crc_table;
        }

    private:
        /// Stores the size of the CRC polynomial in bytes.
        static constexpr uint8_t kCRCByteLength = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /// Stores the initial value used for the CRC checksum calculation.
        const PolynomialType _initial_value;

        /// Stores the final XOR value used for the CRC checksum calculation.
        const PolynomialType _final_xor_value;

        /// Stores the lookup table used to speed up CRC computation at runtime.
        PolynomialType _crc_table[256];

        /**
         * @brief Computes the CRC lookup table for the given polynomial and saves it to the _crc_table member.
         *
         * @param polynomial The CRC polynomial to use for table generation.
         */
        void GenerateCRCTable(const PolynomialType polynomial)
        {
            // Determines the number of bits in the CRC type.
            static constexpr size_t kCRCBits = kCRCByteLength * 8;  // NOLINT(*-dynamic-static-initializers)

            // Determines the Most Significant Bit (MSB) mask based on the CRC type.
            static constexpr PolynomialType kMSBMask =             // NOLINT(*-dynamic-static-initializers)
                static_cast<PolynomialType>(1) << (kCRCBits - 1);  // Parentheses required to avoid compiler warnings

            // Iterates over each possible value of a byte variable.
            for (uint16_t byte = 0; byte < 256; ++byte)
            {
                // Initializes the byte CRC value based on the CRC (Polynomial) datatype.
                auto crc = static_cast<PolynomialType>(byte);

                // Shifts the CRC value left by the appropriate number of bits based on the CRC type to align the
                // initial value to the highest byte of the CRC variable.
                if (kCRCBits > 8)
                {
                    crc <<= kCRCBits - 8;
                }

                // Loops over each of the 8 bits making up the byte value being processed.
                for (uint8_t bit = 0; bit < 8; ++bit)
                {
                    // Checks if the top bit (MSB) is set.
                    if (crc & kMSBMask)
                    {
                        // Shifts the CRC value left to bring the next bit into the top position, then XORs it with
                        // the polynomial. This simulates polynomial division where bits are checked from top to bottom.
                        crc = static_cast<PolynomialType>(crc << 1 ^ polynomial);
                    }
                    else
                    {
                        // Shifts the CRC value left to move to the next bit without modifying the current value, as
                        // division by the polynomial would not produce a remainder here.
                        crc <<= 1;
                    }
                }

                // Stores the calculated CRC remainder for the byte value into the lookup table.
                _crc_table[byte] = crc;
            }
        }
};

#endif  //AXTLMC_CRC_PROCESSOR_H
