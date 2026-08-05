/**
 * @file
 *
 * @brief Provides the CRCProcessor class used to verify transmitted data integrity by calculating the Cyclic
 * Redundancy Check (CRC) checksums for the outgoing and incoming data packets.
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
 * @tparam PolynomialType the datatype of the CRC polynomial used by the class instance. Valid types are uint8_t,
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
         * @note All three checksum parameters are expressed in the standard non-reflected, MSB-aligned form used by
         * published CRC parameter catalogues. Reflected instances derive the reflected polynomial and the reflected
         * initial value internally, so a catalogue entry is transcribed without any manual bit reversal.
         *
         * @param polynomial the polynomial to use for the generation of the CRC lookup table.
         * @param initial_value the value to which the CRC checksum is initialized before calculation.
         * @param final_xor_value the value with which the CRC checksum is XORed after calculation.
         * @param reflected determines whether the instance consumes each data byte least significant bit first and
         * writes the checksum postamble least significant byte first.
         */
        CRCProcessor(
            const PolynomialType polynomial,
            const PolynomialType initial_value,
            const PolynomialType final_xor_value,
            const bool reflected = false
        ) :
            _reflected(reflected),
            _initial_value(reflected ? ReflectValue(initial_value) : initial_value),
            _final_xor_value(final_xor_value)
        {
            GenerateCRCTable(polynomial);
            _expected_residue = ComputeExpectedResidue();
        }

        /**
         * @brief Calculates the checksum for the data stored in the input buffer.
         *
         * Depending on configuration, this method either verifies the data's integrity based on the checksum included
         * with the data or generates and writes the new checksum value to the end of the data's region.
         *
         * @tparam kCheck determines whether the method is called to verify the incoming packet's data integrity or to
         * generate and write the CRC checksum to the outgoing packet's postamble section.
         * @tparam kBufferSize the size of the input buffer.
         * @param buffer the buffer that stores the COBS-encoded packet for which to calculate the checksum. The buffer
         * must conform to kBufferLayout, with a valid payload-size byte read to determine the processing range.
         *
         * @returns the total number of bytes occupied in the buffer, including the appended CRC checksum, when
         * generating a new checksum. Returns '1' when verifying data integrity and the data is intact, and '0'
         * otherwise.
         */
        template <const bool kCheck, const size_t kBufferSize>
        uint16_t CalculateChecksum(uint8_t (&buffer)[kBufferSize])
        {
            // Resolves the reflection mode once per packet so that the per-byte processing loop is specialized at
            // compile time and carries no branch of its own.
            if (_reflected) return ProcessBuffer<true, kCheck>(buffer);

            return ProcessBuffer<false, kCheck>(buffer);
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

        /// Stores the number of entries in the CRC lookup table, which covers every possible value of a data byte.
        static constexpr uint16_t kTableSize = 256;

        /// Stores the number of bits in a single byte, used to shift the checksum by whole bytes.
        static constexpr uint8_t kBitsPerByte = 8;

        /// Stores the mask that isolates the least significant byte of a checksum.
        static constexpr uint8_t kByteMask = 0xFF;

        /// Determines whether the instance processes the packet and the checksum postamble least significant end first.
        const bool _reflected;

        /// Stores the initial value used for the CRC checksum calculation.
        const PolynomialType _initial_value;

        /// Stores the final XOR value used for the CRC checksum calculation.
        const PolynomialType _final_xor_value;

        /// Stores the lookup table used to speed up CRC computation at runtime.
        PolynomialType _crc_table[kTableSize];

        /// Stores the checksum value that verifying an intact packet produces.
        PolynomialType _expected_residue = 0;

        /**
         * @brief Reverses the bit order of the input value across the full bit width of the CRC polynomial type.
         *
         * @param value the value whose bit order to reverse.
         *
         * @returns the input value with its bit order reversed.
         */
        [[nodiscard]]
        static PolynomialType ReflectValue(const PolynomialType value)
        {
            static constexpr uint8_t kCRCBits = kCRCByteLength * 8;  // NOLINT(*-dynamic-static-initializers)

            PolynomialType reflection = 0;

            for (uint8_t bit = 0; bit < kCRCBits; ++bit)
            {
                if (!(value >> bit & 1)) continue;

                reflection =
                    static_cast<PolynomialType>(reflection | static_cast<PolynomialType>(1) << (kCRCBits - bit - 1));
            }

            return reflection;
        }

        /**
         * @brief Computes the CRC lookup table for the given polynomial and saves it to the _crc_table member.
         *
         * @param polynomial the CRC polynomial to use for table generation.
         */
        void GenerateCRCTable(const PolynomialType polynomial)
        {
            static constexpr size_t kCRCBits = kCRCByteLength * 8;  // NOLINT(*-dynamic-static-initializers)

            // Stores the Most Significant Bit (MSB) mask for the CRC polynomial type.
            static constexpr PolynomialType kMSBMask =             // NOLINT(*-dynamic-static-initializers)
                static_cast<PolynomialType>(1) << (kCRCBits - 1);  // Parentheses required to avoid compiler warnings.

            // Reflected instances divide from the least significant bit upward, which is the mirror image of the
            // division below and consumes the bit-reversed form of the polynomial.
            if (_reflected)
            {
                const PolynomialType reflected_polynomial = ReflectValue(polynomial);

                for (uint16_t byte = 0; byte < kTableSize; ++byte)
                {
                    // Initializes the byte CRC value in the low end of the register, which is where reflected
                    // processing keeps the byte currently being divided.
                    auto crc = static_cast<PolynomialType>(byte);

                    for (uint8_t bit = 0; bit < kBitsPerByte; ++bit)
                    {
                        // Checks if the bottom bit (LSB) is set.
                        if (crc & 1)
                        {
                            // Shifts the CRC value right to bring the next bit into the bottom position, then XORs it
                            // with the reflected polynomial.
                            crc = static_cast<PolynomialType>(crc >> 1 ^ reflected_polynomial);
                        }
                        else
                        {
                            // Shifts the CRC value right to move to the next bit without modifying the current value,
                            // as division by the polynomial would not produce a remainder here.
                            crc >>= 1;
                        }
                    }

                    _crc_table[byte] = crc;
                }

                return;
            }

            for (uint16_t byte = 0; byte < kTableSize; ++byte)
            {
                auto crc = static_cast<PolynomialType>(byte);

                // Shifts the CRC value left by the appropriate number of bits based on the CRC type to align the
                // initial value to the highest byte of the CRC variable.
                if (kCRCBits > kBitsPerByte)
                {
                    crc <<= kCRCBits - kBitsPerByte;
                }

                for (uint8_t bit = 0; bit < kBitsPerByte; ++bit)
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

                _crc_table[byte] = crc;
            }
        }

        /**
         * @brief Computes the checksum value that verifying an intact packet produces.
         *
         * Verification runs the checksum calculation over the packet together with its checksum postamble. The value
         * this produces for an intact packet is determined by the polynomial and the final XOR value alone, so it is
         * resolved once at initialization and reused for every verification.
         *
         * @returns the checksum value that indicates an intact packet.
         */
        [[nodiscard]]
        PolynomialType ComputeExpectedResidue() const
        {
            PolynomialType residue = 0;

            // Feeds the final XOR value through a zeroed checksum register, mirroring the way verification consumes
            // the checksum postamble appended to the packet.
            for (uint8_t byte_index = 0; byte_index < kCRCByteLength; ++byte_index)
            {
                if (_reflected)
                {
                    residue = UpdateChecksum<true>(residue, ExtractChecksumByte<true>(_final_xor_value, byte_index));
                }
                else
                {
                    residue = UpdateChecksum<false>(residue, ExtractChecksumByte<false>(_final_xor_value, byte_index));
                }
            }

            return static_cast<PolynomialType>(residue ^ _final_xor_value);
        }

        /**
         * @brief Runs the checksum calculation over the packet stored in the input buffer.
         *
         * @tparam kReflected determines whether the packet and the checksum postamble are processed least significant
         * end first.
         * @tparam kCheck determines whether the method verifies the packet's checksum postamble or generates it.
         * @tparam kBufferSize the size of the input buffer.
         * @param buffer the buffer that stores the COBS-encoded packet for which to calculate the checksum.
         *
         * @returns the total number of bytes occupied in the buffer, including the appended CRC checksum, when
         * generating a new checksum. Returns '1' when verifying data integrity and the data is intact, and '0'
         * otherwise.
         */
        template <const bool kReflected, const bool kCheck, const size_t kBufferSize>
        uint16_t ProcessBuffer(uint8_t (&buffer)[kBufferSize])
        {
            PolynomialType crc_checksum = _initial_value;

            // Sets the start index to the position of the overhead byte. This specializes the function to work
            // exclusively with the buffers defined in this library, similar to how COBSProcessor's methods are
            // implemented.
            constexpr uint16_t kStartIndex = kBufferLayout::kOverheadByteIndex;

            // Adjusts the end index to include the CRC checksum postamble when verifying data integrity, or to
            // include just the packet itself when generating a new checksum.
            constexpr uint16_t kAdjustment = kCheck ? kCRCByteLength : 0;

            // Calculates the end index from the start index, the payload size byte, the fixed overhead and delimiter
            // bytes, and the CRC postamble length when verifying.
            const uint16_t end_index = kStartIndex + buffer[kBufferLayout::kPayloadSizeIndex] + 2 + kAdjustment;

            // Walks the processed region with a pointer rather than an index, which spares the loop the per-byte
            // address arithmetic that recovering the byte from a narrower index would otherwise require.
            const uint8_t* const packet_end = buffer + end_index;
            for (const uint8_t* current_byte = buffer + kStartIndex; current_byte < packet_end; ++current_byte)
            {
                crc_checksum = UpdateChecksum<kReflected>(crc_checksum, *current_byte);
            }

            // Applies the final XOR operation to the checksum. The exact algorithmic purpose depends on the specific
            // polynomial used.
            crc_checksum ^= _final_xor_value;

            // Appends the computed checksum to the buffer immediately after the processed packet when generating a new
            // checksum. The checksum always overwrites any already existing data at the target position.
            if constexpr (!kCheck)
            {
                for (uint8_t byte_index = 0; byte_index < kCRCByteLength; ++byte_index)
                {
                    buffer[end_index + byte_index] = ExtractChecksumByte<kReflected>(crc_checksum, byte_index);
                }

                return end_index + kCRCByteLength;
            }
            else
            {
                // Returns 1 if the CRC calculation on the packet and its checksum postamble matches the expected
                // residue, indicating the data is intact. Returns 0 otherwise, indicating data corruption.
                if (crc_checksum == _expected_residue) return 1;

                return 0;
            }
        }

        /**
         * @brief Folds the input data byte into the running checksum.
         *
         * @tparam kReflected determines whether the data byte is consumed least significant bit first.
         * @param checksum the running checksum to fold the data byte into.
         * @param data_byte the data byte to fold into the checksum.
         *
         * @returns the checksum updated with the input data byte.
         */
        template <const bool kReflected>
        [[nodiscard]]
        PolynomialType UpdateChecksum(const PolynomialType checksum, const uint8_t data_byte) const
        {
            if constexpr (kReflected)
            {
                // Reflected processing keeps the byte being divided in the low end of the register, so the lookup
                // table index comes from the low byte and the register advances by shifting right.
                const auto table_index = static_cast<uint8_t>(checksum ^ data_byte);
                return static_cast<PolynomialType>(checksum >> kBitsPerByte ^ _crc_table[table_index]);
            }
            else
            {
                // Combines the high byte of the CRC checksum with the data byte using bitwise XOR to calculate the
                // lookup table index, then advances the register by shifting left.
                const auto table_index =
                    static_cast<uint8_t>(checksum >> kBitsPerByte * (kCRCByteLength - 1) ^ data_byte);
                return static_cast<PolynomialType>(checksum << kBitsPerByte ^ _crc_table[table_index]);
            }
        }

        /**
         * @brief Extracts the checksum byte stored at the requested postamble offset.
         *
         * @tparam kReflected determines whether the postamble is ordered least significant byte first.
         * @param checksum the checksum from which to extract the byte.
         * @param index the postamble offset of the byte to extract.
         *
         * @returns the checksum byte that occupies the requested postamble offset.
         */
        template <const bool kReflected>
        [[nodiscard]]
        static uint8_t ExtractChecksumByte(const PolynomialType checksum, const uint8_t index)
        {
            // Reflected checksums occupy the postamble least significant byte first, which is the order that drives
            // the verification register to the expected residue.
            if constexpr (kReflected) return static_cast<uint8_t>(checksum >> kBitsPerByte * index & kByteMask);

            return static_cast<uint8_t>(checksum >> kBitsPerByte * (kCRCByteLength - index - 1) & kByteMask);
        }
};

#endif  // AXTLMC_CRC_PROCESSOR_H
