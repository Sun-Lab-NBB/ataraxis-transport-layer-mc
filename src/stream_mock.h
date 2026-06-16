/**
 * @file
 *
 * @brief Provides the StreamMock class used to simulate a Serial Stream interface for testing the TransportLayer
 * class.
 */

#ifndef AXTLMC_STREAM_MOCK_H
#define AXTLMC_STREAM_MOCK_H

#include <Arduino.h>
#include <Stream.h>

/**
 * @brief Simulates a Serial Stream interface by publicly exposing the reception and transmission buffers, their
 * index trackers, and the buffer-size constant for testing.
 *
 * @note The instance buffers use int16_t datatype, but consider any value outside the uint8_t range (0 through 255)
 * as invalid.
 *
 * @tparam kBufferSize the size, in elements, to use for the transmission and reception buffers.
 */
template <const uint16_t kBufferSize = 300>
class StreamMock final : public Stream
{
        static_assert(kBufferSize > 0, "StreamMock buffer size must be greater than zero.");

    public:
        /// Stores the size of the instance's reception and transmission buffers, in elements.
        static constexpr uint16_t kStreamBufferSize = kBufferSize;  // NOLINT(*-dynamic-static-initializers)

        /// Stores the reception buffer data.
        int16_t rx_buffer[kStreamBufferSize] {};

        /// Stores the transmission buffer data.
        int16_t tx_buffer[kStreamBufferSize] {};

        /// Tracks the currently evaluated reception buffer index.
        size_t rx_buffer_index = 0;

        /// Tracks the currently evaluated transmission buffer index.
        size_t tx_buffer_index = 0;

        /// Initializes the instance by filling the transmission and reception buffers with valid zero bytes.
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
         * @note A successful read advances the reception index, consuming the value; a -1 return leaves it unchanged.
         *
         * @returns the read value as a byte-range integer, or -1 if no valid values are available.
         */
        int read() override
        {
            // Returns -1 if the read index exceeds the bounds of the reception buffer.
            if (rx_buffer_index >= sizeof(rx_buffer) / sizeof(rx_buffer[0]))
            {
                return -1;
            }

            // Returns -1 if the value at the current index is outside the valid uint8_t range.
            if (rx_buffer[rx_buffer_index] < 0 || rx_buffer[rx_buffer_index] > 255)
            {
                return -1;
            }

            const int16_t value = rx_buffer[rx_buffer_index];
            rx_buffer_index++;
            return value;
        }

        /**
         * @brief Transfers the specified number of values (bytes) from the reception buffer to the input buffer.
         *
         * @note Unlike the readBytes() Stream class method, this method does not use a timeout timer and instead runs
         * either until it processes the requested number of elements, an 'invalid' value is encountered, or there is no
         * more data to process. Each consumed byte advances the reception index, so the returned count may be any value
         * between 0 and length.
         *
         * @param buffer the buffer where to transfer the read bytes.
         * @param length the number of bytes to read.
         * @returns the number of bytes read and written to the input buffer or 0 if no valid data was read.
         */
        size_t readBytes(uint8_t* buffer, const size_t length)
        {
            size_t bytes_read = 0;

            // Reads bytes from the reception buffer until the specified length is reached or no more valid bytes are
            // available.
            while (bytes_read < length && rx_buffer_index < sizeof(rx_buffer) / sizeof(rx_buffer[0]))
            {
                // Breaks out of the loop if an invalid value is encountered.
                if (rx_buffer[rx_buffer_index] < 0 || rx_buffer[rx_buffer_index] > 255)
                {
                    break;
                }

                buffer[bytes_read] = static_cast<uint8_t>(rx_buffer[rx_buffer_index]);
                bytes_read++;
                rx_buffer_index++;
            }

            return bytes_read;
        }

        /**
         * @brief Writes the requested number of bytes from the input buffer array to the transmission buffer.
         *
         * @note Writing begins at the current transmission index and advances it, so consecutive calls append.
         * If the transmission buffer fills before all input bytes are written, writing stops early and the return
         * value reflects only the bytes actually written.
         *
         * @param buffer the buffer containing the bytes to write.
         * @param bytes_to_write the number of bytes to write to the transmission buffer.
         * @returns the number of bytes written to the transmission buffer.
         */
        size_t write(const uint8_t* buffer, const size_t bytes_to_write) override
        {
            // Writes bytes from the input buffer to the tx_buffer. Terminates prematurely if the writing process
            // reaches the end of the tx_buffer without consuming all input bytes.
            size_t index;
            for (index = 0; index < bytes_to_write && tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]);
                 index++)
            {
                tx_buffer[tx_buffer_index++] = static_cast<int16_t>(buffer[index]);
            }
            return index;
        }

        /**
         * @brief Writes the input byte value to the transmission buffer.
         *
         * @param value the value to write.
         * @returns 1 if the value was written to the transmission buffer, 0 otherwise.
         */
        size_t write(const uint8_t value) override
        {
            if (tx_buffer_index < sizeof(tx_buffer) / sizeof(tx_buffer[0]))
            {
                tx_buffer[tx_buffer_index++] = static_cast<int16_t>(value);
                return 1;
            }
            return 0;
        }

        /**
         * @brief Returns the number of elements in the reception buffer available for reading.
         *
         * @returns the number of contiguous valid byte-range elements starting at the current reception index.
         */
        int available() override
        {
            size_t count = 0;

            // Iterates over rx_buffer elements starting from rx_buffer_index until the end of the buffer or the first
            // invalid value.
            for (size_t index = rx_buffer_index; index < sizeof(rx_buffer) / sizeof(rx_buffer[0]); ++index)
            {
                if (rx_buffer[index] < 0 || rx_buffer[index] > 255)
                {
                    break;
                }
                count++;
            }

            return static_cast<int>(count);
        }

        /**
         * @brief Reads a value from the reception buffer without consuming the data.
         *
         * @returns the peeked 'byte' value between 0 and 255, or -1 if there are no valid byte-values to read.
         */
        int peek() override
        {
            // Returns -1 if the read index exceeds the bounds of the reception buffer.
            if (rx_buffer_index >= sizeof(rx_buffer) / sizeof(rx_buffer[0]))
            {
                return -1;
            }

            // Returns -1 if the value at the current index is outside the valid uint8_t range.
            if (rx_buffer[rx_buffer_index] < 0 || rx_buffer[rx_buffer_index] > 255)
            {
                return -1;
            }

            return rx_buffer[rx_buffer_index];
        }

        /// Simulates the data being sent to the PC (flushed) by resetting the instance's transmission buffer.
        void flush() override
        {
            for (size_t index = 0; index < sizeof(tx_buffer) / sizeof(tx_buffer[0]); ++index)
            {
                tx_buffer[index] = -1;
            }
            tx_buffer_index = 0;
        }

        /// Resets the transmission and reception buffers by filling them with the -1 invalid-value sentinel.
        void reset()
        {
            memset(rx_buffer, -1, sizeof(rx_buffer));
            memset(tx_buffer, -1, sizeof(tx_buffer));
            rx_buffer_index = 0;
            tx_buffer_index = 0;
        }

        /// Defaults the destructor.
        virtual ~StreamMock() = default;
};

#endif  //AXTLMC_STREAM_MOCK_H
