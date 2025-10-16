/**
 * @file
 * @brief This file provides the StreamMock class used to simulate a Serial Stream interface to test the TransportLayer
 * class.
 */

#ifndef AXTLMC_STREAM_MOCK_H
#define AXTLMC_STREAM_MOCK_H

// Dependencies:
#include <Arduino.h>
#include <Stream.h>

/**
 * @class StreamMock
 * @brief A Stream class implementation that publicly exposes its reception and transmission buffers.
 *
 * @note The instance buffers use int16_t datatype, but consider any value outside the uint8_t range (0 through 255)
 * as invalid.
 *
 * @tparam kBufferSize The size (in elements) to use for the transmission and reception buffers.
 */
template <uint16_t kBufferSize = 300>
class StreamMock final : public Stream
{
    public:
        /**
         * @brief Overrides the destructor of the parent class. Does not do anything meaningful.
         */
        virtual ~StreamMock() = default;

        /// Stores the size of ths instance's reception and transmission buffers, in elements.
        static constexpr uint16_t buffer_size = kBufferSize;  // NOLINT(*-dynamic-static-initializers)

        /// The reception buffer.
        int16_t rx_buffer[buffer_size] {};

        /// The transmission buffer.
        int16_t tx_buffer[buffer_size] {};

        /// Tracks the currently evaluated reception buffer index (value).
        size_t rx_buffer_index = 0;

        /// Tracks the currently evaluated transmission buffer index (value).
        size_t tx_buffer_index = 0;

        /**
         * @brief Resets the instance's transmission and reception buffers.
         */
        StreamMock()
        {
            memset(rx_buffer, 0, sizeof(rx_buffer));
            memset(tx_buffer, 0, sizeof(tx_buffer));
            rx_buffer_index = 0;
            tx_buffer_index = 0;
        }

        /**
         * @brief Reads one value ('byte') from the reception buffer.
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
         * @brief Transfers the specified number of values (bytes) from the reception buffer to the input buffer.
         *
         * @note Unlike the readBytes() Stream class method, this method does not use a timeout timer and instead runs
         * either until it processes the requested number of elements, an 'invalid' value is encountered, or there is no
         * more data to process.
         *
         * @param buffer The buffer where to transfer the read bytes.
         * @param length The number of bytes to read.
         * @returns The number of bytes read and written to the input buffer or 0 if no valid data was read.
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
         * @note Each writing cycle starts at index 0 of the transmission buffer, overwriting as many indices as
         * necessary to fully consume the input buffer.
         *
         * @param buffer The buffer containing the bytes to write.
         * @param bytes_to_write The number of bytes to write to the transmission buffer.
         * @returns The number of bytes written to the transmission buffer.
         */
        size_t write(const uint8_t *buffer, const size_t bytes_to_write) override
        {
            // Writes the requested number of bytes from the input buffer to the tx_buffer of the class. The method
            // is terminated prematurely if the writing process reaches the end of the tx_buffer without consuming
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
         * @param value The value to write.
         * @returns Integer 1 if the method writes the value to the transmission buffer and 0 otherwise.
         */
        size_t write(const uint8_t value) override
        {
            // Checks if the buffer has space based on the last evaluated element index.
            if (tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]))
            {
                tx_buffer[tx_buffer_index++] = static_cast<int16_t>(value);
                return 1;  // Number of bytes written
            }
            return 0;  // Buffer full, nothing written
        }

        /**
         * @brief Returns the number of elements in the reception buffer available for reading.
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
         * @brief Reads a value from the reception buffer without consuming the data.
         *
         * @returns The peeked 'byte' value between 0 and 255, or -1 if there are no valid byte-values to read.
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
         * @brief Simulates the data being sent to the PC (flushed) by resetting the instance's transmission buffer.
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
         * @brief Resets the instance's transmission and reception buffers.
         */
        void reset()
        {
            memset(rx_buffer, -1, sizeof(rx_buffer));
            memset(tx_buffer, -1, sizeof(tx_buffer));
            rx_buffer_index = 0;
            tx_buffer_index = 0;
        }
};

#endif  //AXTLMC_STREAM_MOCK_H
