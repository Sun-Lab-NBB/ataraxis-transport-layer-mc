/**
 * @file
 * @brief The header file for the CRCProcessor class, which is used to verify transmitted data integrity by calculating
 * the Cyclic Redundancy Check (CRC) Checksum for the outgoing and incoming data packets.
 *
 * @section crc_description Description:
 * Cyclic Redundancy Check (CRC) is a widely used algorithm for verifying data integrity that is based on the residual
 * obtained from dividing the byte-serialized data array by a polynomial. A well-known property of the CRC algorithm is
 * that running the CRC calculation on a packet that contains the CRC checksum at the end always returns 0 as the CRC
 * checksum.
 *
 * For more information about CRC see this original paper:
 * W. W. Peterson and D. T. Brown, "Cyclic Codes for Error Detection," in Proceedings of the IRE, vol. 49, no. 1,
 * pp. 228-235, Jan. 1961, @a doi: 10.1109/JRPROC.1961.287814.
 *
 * This file contains four major methods packaged into the COBSProcessor class namespace:
 * - CalculatePacketCRCChecksum(): A CRC checksum calculator that uses a class-specific CRC lookup table generated
 * during class instantiation to speed up it's runtime.
 * - AddCRCChecksumToBuffer(): A packager method that converts an input CRC checksum into bytes and adds them to the
 * specified location of the input buffer.
 * - ReadCRCChecksumFromBuffer(): An unpackager method that extracts a CRC checksum from the specified location of the
 * input buffer and returns it as an appropriately sized unsigned integer.
 * - GenerateCRCTable(): A private method automatically called during class instantiation that calculates a constant
 * CRC-lookup table used to speed up checksum calculation at the cost of reserving a certain amount of RAM.
 * - A number of other minor utility methods used by the major method listed above.
 *
 * @attention This class is implemented as a template, and many methods adapt to the PolynomialType argument used during
 * class instantiation. See developer notes below for more information.
 *
 * @section crc_developer_notes Developer Notes:
 * This class is implemented as a template and all methods of the class scale with the CRC polynomial type used during
 * instantiation. This allows using the same class for all common polynomials used for CRC checksum calculations of
 * types uint8_t, uint16_t, and uint32_t. This is part of a boarder effort to make the library flexible enough to
 * be used on a wide variety of platforms.
 *
 * @note 64-bit+ polynomials are currently not supported, largely due to their niche use case and issues
 * with using the existing testing environment with 64-bit variables. That said, the code is designed in a way that
 * would naturally scale to work with 64-bit+ polynomials if the static instantiation guard is modified to allow this.
 * Currently, there are no plans to offer official support for this functionality though.
 *
 * Due to the complex nature of class definition, the class can only be instantiated by an instantiation-time
 * assignment:
 * @code
 * CRCProcessor<uint16_t> crc_class(0x1021, 0xFFFF, 0x0000);
 * @endcode
 *
 * It is important to use the same template type argument as used by the polynomial (first argument of the class
 * constructor), as the class automatically scales all constructor arguments to the declared polynomial type template
 * parameter.
 *
 * @section crc_dependencies Dependencies:
 * - Arduino.h for Arduino platform methods and macros and cross-compatibility with Arduino IDE (to an extent).
 * - shared_assets.h for shared library assets (mostly status byte-codes for library classes).
 */

#ifndef AXMC_CRC_PROCESSOR_H
#define AXMC_CRC_PROCESSOR_H

// Dependencies
#include <Arduino.h>
#include "axtlmc_shared_assets.h"

/**
 * @class CRCProcessor
 * @brief Generates a Circular Redundancy Check table during initialization and provides methods that use this table to
 * calculate the CRC checksum and add / read this checksum from storage buffers for transmitted data packets.
 *
 * @warning All methods of this class scale with the PolynomialType template argument of the class. Make sure that this
 * argument matches the used polynomial type during class instantiation to avoid unexpected behavior.
 *
 * @note Each instance of this class computes a constant CRC table during initialization and subsequently uses it to
 * generate CRC checksum values for input data byte-streams. The table reserves either 256, 512, or 1024 bytes of
 * memory depending on the PolynomialType template parameter. It is fairly uncommon to use more than a single
 * instance of this class for memory-constrained platforms, such as microcontrollers.
 *
 * @attention This class makes certain assumptions about the layout of the input buffers and, additionally, is very
 * sensitive to the correct constructor configuration (due to the high versatility of CRC-table generation method). Due
 * to these reasons, this class is intended to be used exclusively by the TransportLayer class, which
 * appropriately handles all assumptions and configuration requirements.
 *
 * @tparam PolynomialType The datatype of the CRC polynomial to be used by the class. Valid types are uint8_t, uint16_t,
 * and uint32_t. The class contains a compile-time guard against any other input datatype. Overall, this allows using
 * this class with the vast majority of available CRC checksum polynomials, significantly increasing class versatility.
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
        // Ensures that the class only accepts uint8, 16, or 32 as valid CRC types, as no other type can be used to
        // store a CRC polynomial at the time of writing. While 64-bit polynomials are also available, they are fairly
        // uncommon and most likely not necessary for most use cases of this library. As such, at the time, there are
        // no plans to offer 64-bit polynomial support.
        static_assert(
            axtlmc_shared_assets::is_same_v<PolynomialType, uint8_t> ||
                axtlmc_shared_assets::is_same_v<PolynomialType, uint16_t> ||
                axtlmc_shared_assets::is_same_v<PolynomialType, uint32_t>,
            "CRCProcessor class template PolynomialType argument must be either uint8_t, uint16_t, or uint32_t."
        );

    public:
        /// Stores the latest runtime status of the CRCProcessor. This variable is primarily designed to communicate the
        /// specific errors encountered during crc checksum calculation or reading / writing crc checksum to buffers. It
        /// uses byte-codes taken from the kCRCProcessorCodes enumeration (available through shared_assets
        /// namespace). Use the communicated status to precisely determine the runtime status of any class method.
        uint8_t crc_status = static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kStandby);

        /// The array that stores the CRC lookup table. The lookup table is used to speed up CRC checksum calculation by
        /// pre-computing the checksum value for each possible byte-value (from 0 through 255: 256 values total). The
        /// table is filled automatically during class instantiation and reserves 256, 512, or 1024 bytes of RAM for the
        /// entire lifetime of the class, depending on the PolynomialType template parameter.
        PolynomialType crc_table[256];

        /**
         * @brief Instantiates CRCProcessor class and generates a static CRC lookup table to be used by the class
         * instance to speed up CRC checksum calculations.
         *
         * @attention Uses the class template PolynomialType argument to determine the type of the CRC polynomial to be
         * used. Scales most expected arguments to match the declared PolynomialType template parameter.
         *
         * @param polynomial The polynomial to use for the generation of the CRC lookup table. Can be provided as an
         * appropriately sized HEX number (e.g., 0x1021). Note, currently only non-reversed polynomials are supported.
         * @param initial_value The initial value to which the CRC checksum variable is initialized during calculation.
         * This value is based on the polynomial parameter. Can be provided as an appropriately sized HEX number
         * (e.g., 0xFFFF).
         * @param final_xor_value The final XOR value to be applied to the calculated CRC checksum value. This value is
         * based on the polynomial parameter. Can be provided as an appropriately sized HEX number (e.g., 0x0000).
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
                crc_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCalculateCRCChecksumBufferTooSmall);

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
            crc_status = static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated);
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
                crc_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kAddCRCChecksumBufferTooSmall);
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
            crc_status = static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumAddedToBuffer);
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
                crc_status =
                    static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kReadCRCChecksumBufferTooSmall);
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
            crc_status = static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumReadFromBuffer);
            return static_cast<PolynomialType>(extracted_crc);
        }

    private:
        /// Stores the initial value used for the CRC checksum calculation. Specifically, this is the value that the
        /// CRC checksum variable is initialized to at the beginning of each CalculatePacketCRCChecksum() method
        /// runtime.
        const PolynomialType kInitialValue;

        /// Stores the final XOR value used for the CRC checksum calculation. Specifically, this is the value that the
        /// CRC checksum variable is XORed with prior to being returned to caller at the end of each
        /// CalculatePacketCRCChecksum() method runtime.
        const PolynomialType kFinalXORValue;

        /// Stores the length of the CRC polynomial in bytes. This is used across most methods of the class to
        /// automatically scale processing to the number of bytes used to store the CRC checksum value.
        static constexpr uint8_t kCRCByteLength = sizeof(PolynomialType);  // NOLINT(*-dynamic-static-initializers)

        /**
         * @brief Uses the polynomial specified (as an argument) during class instantiation to compute the CRC
         * checksums for each possible uint8_t (byte) value.
         *
         * @attention This method has to be called before calculating any CRC checksum via
         * CalculatePacketCRCChecksum() method. Currently, this is enforced by calling this method inside the class
         * constructor.
         *
         * The method is configured to update the crc_table with generated CRC values as they get generated, so the
         * crc_table has to be initialized correctly for this method to work properly.
         *
         * @note The generation of the CRC table vastly increases the speed of CRC checksum computation at the cost of
         * reserving a portion of RAM to store the table. This may be an issue when the library runs on
         * memory-constrained platforms, such as Arduino Uno microcontrollers. In that case, it is advised to use the
         * smallest appropriate polynomial.
         *
         * Example usage:
         * @code
         * uint16_t polynomial = 0x1021; // Assumes PolynomialType of the class was set to uint16_t!
         * GenerateCRCTable(polynomial); // Modifies class instance crc_table variable internally.
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
                        // If the top bit is not set, simply shifts the crc value left. This moves to the next bit
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

#endif  //AXMC_CRC_PROCESSOR_H
