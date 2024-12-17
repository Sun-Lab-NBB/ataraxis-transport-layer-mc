/**
 * @file
 * @brief A header-only file that provides the StreamMock class, which simulates a Serial Stream interface to test
 * TransportLayer class.
 *
 * @section smock_description Description:
 *
 * @attention This file is specifically designed to test the TransportLayer class on supported microcontrollers. It does
 * actively participate in any stages necessary to process incoming or outgoing data during production runtimes.
 *
 * This class allows testing the TransportLayer class without establishing a fully functional bidirectional
 * connection with any recipient. To do so, it implements two large (~600 bytes each) public buffers to mimic the
 * transmission and reception buffers used by the actual Serial connection interface.
 *
 * This class exposes the following public methods:
 * - read().
 * - readBytes() To read multiple bytes into the input buffer.
 * - write() [Overloaded to work for single byte-inputs and array inputs].
 * - reset().
 * - flush().
 * - available().
 * - peek().
 *
 * @note This class overloads the virtual methods of the Stream base-class and, therefore, uses the same names,
 * arguments and behavior as the base class.
 *
 * @section smock_developer_notes Developer Notes:
 * This class is used solely to enable testing the TransportLayer class and, in the future, it may be
 * excluded from non-developer builds of the library.
 *
 * The class reserves a lot of memory (~1200 bytes total) to support its buffers, which may be an issue for lower-end
 * microcontroller boards. The official test suite for microcontrollers accounts for this requirement and disables
 * affected tests for incompatible platforms. Microcontrollers with < 2000 bytes of RAM may not be compatible with the
 * test suite. The size of the buffers for this class can be adjusted via the class 'kBuffersSize' template parameter
 * during instantiation.
 *
 * @attention The class uses int16_t buffers, so all functions had to be modified to work with elements rather than
 * bytes, as each 'byte' is actually represented by a signed short datatype in this class. This information is
 * particularly relevant for writing test functions using the class that directly check buffer states against byte
 * inputs.
 *
 * @section smock_dependencies Dependencies:
 * - Arduino.h for Arduino platform functions and macros and cross-compatibility with Arduino IDE (to an extent).
 * - Stream.h for the base Stream class that is overloaded to form this Mock class.
 */

#ifndef AXTLMC_STREAM_MOCK_H
#define AXTLMC_STREAM_MOCK_H

// Dependencies:
#include <Arduino.h>
#include <Stream.h>

/**
 * @class StreamMock
 * @brief An implementation of the Stream class that publicly exposes its reception and transmission buffers.
 *
 * @note The class buffers use int16_t datatype with any value outside 0 through 255 range is considered invalid.
 * All class methods work as if they are operating byte-buffers, like the original Stream class, but
 * the implementation of integer buffers allows manually setting portions of the buffer to invalid values as necessary
 * for certain test scenarios.
 *
 * @tparam kBufferSize The size (in elements) to use for the transmission and reception buffers. Note, the buffers use
 * signed 16-bit integer types and reserve size * 2 bytes for each buffer.
 */
template <uint16_t kBufferSize = 300>
class StreamMock final : public Stream
{
    public:
        /**
         * @brief Overrides the destructor of the parent class. Does not do anything meaningful.
         */
        virtual ~StreamMock() = default;

        /// Stores the buffer size value derived from the template parameter during class instantiation. This size is
        /// used to instantiate both the transmission and reception buffers.
        static constexpr uint16_t buffer_size = kBufferSize;  // NOLINT(*-dynamic-static-initializers)

        /// Reception buffer. Note. Only values from 0 to 255 are treated as valid, although the buffer supports the
        /// whole uint16 range.
        int16_t rx_buffer[buffer_size] {};

        /// Transmission buffer. Note. Only values from 0 to 255 are treated as valid, although the buffer supports the
        /// whole uint16 range.
        int16_t tx_buffer[buffer_size] {};

        /// Tracks the last evaluated index in reception buffer. Incremented by read operations.
        size_t rx_buffer_index = 0;

        /// Tracks the last evaluated index in transmission buffer. Incremented by write operations.
        size_t tx_buffer_index = 0;

        /**
         * @brief Initializes class object instance. Sets all values inside the reception and transmission buffers to 0.
         */
        StreamMock()
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            memset(tx_buffer, 0, sizeof(tx_buffer));
        }

        /**
         * @brief Reads one value ('byte') from the reception buffer and returns it to caller. Returns -1 if no valid
         * values are available.
         *
         * @note The buffer uses int16_t type, but only values inside the uint8_t (0 through 255) range are considered
         * valid. As such, this method fully emulates how a byte-stream would behave.
         *
         * @returns The read value as a byte-range integer, or -1 if no valid values are available.
         */
        int read() override
        {
            // If read index is within the confines of the rx_buffer, reads the byte currently pointed to by the index.
            if (rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]))  // Adjusts to count elements, not bytes
            {
                // Checks if the value at the current index is within the valid uint8_t range
                if (rx_buffer[rx_buffer_index] >= 0 && rx_buffer[rx_buffer_index] <= 255)
                {
                    const int value = rx_buffer[rx_buffer_index];  // Reads the value from the buffer as an int
                    rx_buffer_index++;                             // Increments after reading the value
                    return value;                                  // Returns the read value
                }
            }

            // If the index is beyond the bounds of the buffer or invalid data is encountered, returns -1 without
            // incrementing the index to indicate there is no data to read.
            return -1;
        }

        /**
         * @brief Reads the specified number of values (bytes) from the reception buffer and writes them to the input
         * buffer.
         *
         * Specifically, attempts to read the "length" number of values from the class reception buffer into the input
         * buffer, writing from index 0 of the input buffer up to index length-1. Unlike the real readBytes method, this
         * method does not use a timeout timer and instead only runs until the data is read, an 'invalid' value is
         * encountered or until the end of the reception buffer is reached.
         *
         * @note The buffer uses int16_t type, but only values inside the uint8_t (0 through 255) range are considered
         * valid. As such, this method fully emulates how a byte-stream would behave.
         *
         * @param buffer The buffer to store the read bytes into.
         * @param length The maximum number of bytes to read.
         * @returns The number of bytes actually read and written to the input buffer. 0 means no valid data was read.
         */
        // ReSharper disable once CppHidingFunction
        size_t readBytes(uint8_t *buffer, const size_t length)
        {
            size_t bytes_read = 0;

            // Reads bytes from the reception buffer until the specified length is reached or no more valid bytes are
            // available
            while (bytes_read < length && rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]))
            {
                // Checks if the value at the current index is within the valid uint8_t range
                if (rx_buffer[rx_buffer_index] >= 0 && rx_buffer[rx_buffer_index] <= 255)
                {
                    // Converts the int16_t value to uint8_t and stores it in the provided buffer
                    buffer[bytes_read] = static_cast<uint8_t>(rx_buffer[rx_buffer_index]);
                    bytes_read++;
                    rx_buffer_index++;
                }
                else
                {
                    // If an invalid value is encountered, breaks out of the loop as there are no more valid bytes to
                    // read
                    break;
                }
            }

            // Returns the number of bytes actually read and stored in the buffer
            return bytes_read;
        }

        /**
         * @brief Writes the requested number of bytes from the input buffer array to the transmission buffer.
         *
         * Each writing cycle starts at index 0 of the transmission buffer, overwriting as many indices as necessary to
         * fully consume the input buffer.
         *
         * @note The input buffer has to be uint8_t, but the values will be converted to int16_t to be saved to the
         * transmission buffer.
         *
         * @param buffer The input buffer containing the bytes to write.
         * @param bytes_to_write The number of bytes to write from the input buffer.
         * @returns The number of bytes written to the transmission buffer.
         */
        size_t write(const uint8_t *buffer, const size_t bytes_to_write) override
        {
            // Writes requested number of bytes from the input buffer to the tx_buffer of the class. Note, the method
            // will be terminated prematurely if the writing process reaches the end of the tx_buffer without consuming
            // all bytes available from the input buffer.
            size_t i;
            for (i = 0; i < bytes_to_write && tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]); i++)
            {
                tx_buffer[tx_buffer_index++] = static_cast<int16_t>(buffer[i]);
            }
            return i;  // Returns the number of bytes written to the tx_buffer.
        }

        /**
         * @brief Writes the input byte value to the transmission buffer.
         *
         * @note Converts the value to the uint16_t type to be stored inside the transmission buffer.
         *
         * @param byte_value The byte value to write.
         * @returns Integer 1 when the method succeeds and 0 otherwise.
         */
        size_t write(const uint8_t byte_value) override
        {
            // Checks if the buffer has space based on the last evaluated element index.
            if (tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]))
            {
                tx_buffer[tx_buffer_index++] = static_cast<int16_t>(byte_value);
                return 1;  // Number of bytes written
            }
            return 0;  // Buffer full, nothing written
        }

        /**
         * @brief Returns the number of elements in the reception buffer available for reading.
         *
         * To do so, scans the buffer contents from the rx_buffer_index either to the end of the buffer or the first
         * invalid value and returns the length of the scanned data stretch. Uses elements rather than bytes due to the
         * uint16_t type of the buffer.
         *
         * @returns The number of available 'bytes' (valid values) in the rx_buffer.
         */
        int available() override
        {
            size_t count = 0;

            // Iterates over the rx_buffer elements starting from rx_buffer_index until the end of the buffer or the
            // first invalid value
            for (size_t i = rx_buffer_index; i < sizeof(rx_buffer) / sizeof(rx_buffer[0]); ++i)
            {
                // Checks if the current value is within the uint8_t range.
                if (rx_buffer[i] >= 0 && rx_buffer[i] <= 255)
                {
                    // If so, this is considered available data, so increments the count.
                    count++;
                }
                else
                {
                    // If an invalid value is encountered, brakes out of the loop as there is no more valid data to
                    // count.
                    break;
                }
            }

            // Returns the count of available data bytes
            return static_cast<int>(count);  // Cast count to int to match the return type
        }

        /**
         * @brief Reads a value from the reception buffer without advancing to the next value
         * (without consuming the data).
         *
         * @returns The peeked 'byte' value as an integer (signed integer), or -1 if there are no valid byte-values to
         * read (if there is no more data available).
         */
        int peek() override
        {
            // Checks whether the value pointed by rx_buffer_index is within the boundaries of the rx_buffer and is a
            // valid uint8_t value (between 0 and 255 inclusive).
            if (rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]) &&
                (rx_buffer[rx_buffer_index] >= 0 && rx_buffer[rx_buffer_index] <= 255))
            {
                // If so, returns the value without incrementing the index.
                return rx_buffer[rx_buffer_index];
            }
            return -1;  // If there is no valid data to peek, returns -1.
        }

        /**
         * @brief Simulates the data being sent to the PC (flushed) by resetting all transmission buffer values to -1
         * (no data) value and resetting the tx_buffer_index to 0 (to point to the first element of the buffer).
         *
         */
        void flush() override
        {
            // Resets the tx_buffer_index and the buffer itself to simulate the data being sent ('flushed') to the PC.
            for (size_t i = 0; i < sizeof(tx_buffer) / sizeof(tx_buffer[0]); ++i)
            {
                tx_buffer[i] = -1;  // Sets every value of the buffer to -1 (invalid / no data)
            }

            tx_buffer_index = 0;  // Sets the index to 0
        }

        /**
         * @brief Resets the reception and transmission buffers and their index tracker variables.
         *
         * This method is typically used during testing to reset the buffers between tests. Sets each variable inside
         * each buffer to -1 (no data) and sets the tracker indices to point to the beginning of each buffer (to 0).
         * This ensures that the buffers default to an empty state, mimicking the standard behavior for the empty
         * Serial Stream interface.
         */
        void reset()
        {
            // Initializes buffers to a "no data" value. Here -1 is chosen for simplicity, but any value outside the
            // 0 through 255 range would work the same way.
            for (size_t i = 0; i < sizeof(rx_buffer) / sizeof(rx_buffer[0]); ++i)
            {
                rx_buffer[i] = -1;
            }
            for (size_t i = 0; i < sizeof(tx_buffer) / sizeof(tx_buffer[0]); ++i)
            {
                tx_buffer[i] = -1;
            }
            // Resets the tracker indices
            rx_buffer_index = 0;
            tx_buffer_index = 0;
        }
};

#endif  //AXTLMC_STREAM_MOCK_H
