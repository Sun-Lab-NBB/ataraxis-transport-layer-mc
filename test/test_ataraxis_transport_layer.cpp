/**
 * @file
 *
 * @brief Verifies the behavior of all classes and methods provided by the TransportLayer library.
 *
 * @note Due to reconnection issues with Teensy boards, all tests are centralized in a single file rather than split
 * across separate test suites.
 */

#include <Arduino.h>
#include <unity.h>  // C testing framework, not the Unity game engine
#include "axtlmc_shared_assets.h"
#include "cobs_processor.h"
#include "crc_processor.h"
#include "stream_mock.h"
#include "transport_layer.h"

using namespace axtlmc_shared_assets;

// Restricts the 32-bit CRC tests to the Arduino Due and Teensy boards. Each 32-bit instance builds a 1024-byte lookup
// table, and the table generation tests hold a reference table of the same size beside it, which is more memory than
// the AVR boards can comfortably spare.
#if defined(ARDUINO_ARCH_SAM) || defined(CORE_TEENSY)
#define RUN_WIDE_CRC_TESTS
#endif

// Runs the 16-bit CRC tests on every supported board. A 16-bit instance builds a 512-byte lookup table, and the table
// generation tests hold a 512-byte reference beside it, which fits the 8192 bytes of SRAM the smallest supported board
// (ATmega2560) provides. AVR is also the only supported architecture whose 'int' is 16 bits wide, which makes it the
// likeliest place for a width-dependent defect in the table generation or the checksum register to surface, so it is
// the one board these tests must not skip.
#define RUN_CRC16_TESTS

/// Called automatically before each test function. Currently unused.
void setUp()
{}

/// Called automatically after each test function. Currently unused.
void tearDown()
{}

/// Verifies COBSProcessor EncodePayload() and DecodePayload() methods.
void test_cobs_processor_encode_decode()
{
    // Prepares test assets
    uint8_t payload_buffer[258];
    memset(payload_buffer, 22, sizeof(payload_buffer));

    // Creates a test payload using the format: start [0], payload_size [1], overhead [2], payload [3 to 12] (10 total),
    // delimiter [13]
    const uint8_t initial_packet[14] = {129, 10, 0, 1, 0, 3, 0, 0, 0, 7, 0, 9, 10, 22};
    memcpy(payload_buffer, initial_packet, sizeof(initial_packet));

    // Expected packet after encoding, used to test the encoding result
    const uint8_t encoded_packet[14] = {129, 10, 2, 1, 2, 3, 1, 1, 2, 7, 3, 9, 10, 0};

    // Expected state of the packet after decoding. The payload is reverted to the original state, the overhead is
    // reset to 0, and the delimiter byte is unchanged. Used to test the decoding result.
    const uint8_t decoded_packet[14] = {129, 10, 0, 1, 0, 3, 0, 0, 0, 7, 0, 9, 10, 0};

    constexpr uint8_t kPayloadSize = 10;  // Tested payload size for the payload generated above
    constexpr uint8_t kPacketSize  = 12;  // Tested packet size for the decoder test

    // Verifies the unencoded packet matches pre-test expectations
    TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_packet, payload_buffer, sizeof(initial_packet));

    // Encodes test payload
    const uint16_t encoded_size = COBSProcessor::EncodePayload(payload_buffer);

    // Verifies that encoding returned expected payload size (10) + overhead + delimiter (== 12, packet size)
    TEST_ASSERT_EQUAL_UINT16(kPacketSize, encoded_size);

    // Verifies that the encoded payload matches the expected encoding outcome
    TEST_ASSERT_EQUAL_UINT8_ARRAY(encoded_packet, payload_buffer, sizeof(encoded_packet));

    // Decodes test payload
    const uint16_t decoded_size = COBSProcessor::DecodePayload(payload_buffer);

    // Checks that size correctly equals kPacketSize - 2 (10, kPayloadSize).
    TEST_ASSERT_EQUAL_UINT16(kPayloadSize, decoded_size);

    // Verifies that decoding reverses the payload back to the original state. Note, decoding resets the overhead
    // byte to 0 and leaves the appended delimiter byte unchanged (hence the use of a separate tester array)
    TEST_ASSERT_EQUAL_UINT8_ARRAY(decoded_packet, payload_buffer, sizeof(decoded_packet));

    // Verifies that the non-packet-related portion of the buffer was not affected by the encoding/decoding cycles
    for (uint16_t i = sizeof(encoded_packet); i < static_cast<uint16_t>(sizeof(payload_buffer)); i++)
    {
        // Uses a custom message system similar to Unity Array check to provide the failed index number
        char message[50];  // Buffer for the failure message
        snprintf(message, sizeof(message), "Check failed at index: %d", i);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(22, payload_buffer[i], message);
    }
}

/// Verifies error handling for COBSProcessor DecodePayload() and boundary-size handling for EncodePayload().
void test_cobs_processor_errors()
{
    // Generates the test buffer and sets every value inside to 22
    uint8_t payload_buffer[258];
    memset(payload_buffer, 22, sizeof(payload_buffer));
    payload_buffer[2] = 0;  // Zeroes the overhead placeholder, which EncodePayload() overwrites with the COBS overhead

    // Verifies that payloads with minimal size are encoded correctly
    payload_buffer[1] = static_cast<uint8_t>(kBufferLayout::kMinimumPayloadSize);
    uint16_t result   = COBSProcessor::EncodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMinimumPacketSize), result);

    // Verifies packets with minimal size are decoded correctly. Uses the packet encoded above.
    result = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMinimumPayloadSize), result);

    // Verifies that payloads with maximal size are encoded correctly
    payload_buffer[1] = static_cast<uint8_t>(kBufferLayout::kMaximumPayloadSize);
    result            = COBSProcessor::EncodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMaximumPacketSize), result);

    // Verifies that packets with maximal size are decoded correctly. Uses the packet encoded above.
    result = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMaximumPayloadSize), result);

    // Tests decoder payload (in)validation error codes, issued whenever the payload does not conform to the format
    // expected from COBS encoding. During runtime, the decoder assumes that the packages were properly encoded using
    // the COBSProcessor class and, therefore, any deviation from the expected format is likely due to the payload or
    // packet being corrupted during transmission.

    // Resets the shared buffer to the default state before running the test to exclude any confounding factors from the
    // tests above
    memset(payload_buffer, 22, sizeof(payload_buffer));
    payload_buffer[2] = 0;  // Re-zeroes the overhead placeholder after the buffer reset above

    // Introduces 'jump' variables to be encoded by the call below (since 0 is the delimiter value to be encoded)
    payload_buffer[5]  = 0;
    payload_buffer[10] = 0;

    // Encodes the payload of size 15, inserting a delimiter (0) byte at index 18, generating a packet of size 17
    payload_buffer[1]           = 15;
    const uint16_t encoded_size = COBSProcessor::EncodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(17, encoded_size);

    // Mirrors the buffer state each aborted decoding cycle below is expected to leave behind. Comparing against a
    // snapshot is what demonstrates that the decoder stops writing where it stops reading, which the returned error
    // code on its own cannot show.
    uint8_t expected_buffer[sizeof(payload_buffer)];

    // Decodes the packet of size 13 (17-4), which is a valid size. The process should abort before the delimiter at
    // index 16 is reached with the appropriate error code. Tests both the error code and that the decoder that uses a
    // while loop exits the loop as expected instead of overwriting the 'out-of-limits' buffer memory.
    payload_buffer[1] = 13;

    // Captures the pre-decode state and applies the three writes the aborted cycle is entitled to make. The decoder
    // walks the encoded chain from the overhead byte to index 5 and then to index 10, and the jump taken from index 10
    // lands past the truncated packet's delimiter index (16), which ends the cycle. Every remaining byte, and the
    // whole region past index 16 in particular, has to survive the aborted cycle untouched.
    memcpy(expected_buffer, payload_buffer, sizeof(payload_buffer));
    expected_buffer[2]  = 0;  // The decoder zeroes the overhead byte before it walks the encoded chain
    expected_buffer[5]  = 0;  // The decoder restores the encoded delimiter it walks through at index 5
    expected_buffer[10] = 0;  // The decoder restores the encoded delimiter it walks through at index 10

    result = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(0, result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_buffer, payload_buffer, sizeof(expected_buffer));

    // Restores the delimiter value at index 10, which the aborted cycle above already decoded back to 0. Keeping the
    // assignment makes the state the next decoding cycle runs on explicit rather than inherited.
    payload_buffer[10] = 0;

    // Resets the overhead back to the correct value, since the decoder overwrites it to 0 on each call, even if the
    // call produces one of the 'malformed packet' errors
    payload_buffer[2] = 3;
    payload_buffer[1] = 15;  // Also restores the payload_size to the proper size

    // Captures the pre-decode state again. The overhead byte sends the decoder straight to index 5, which now holds a
    // decoded delimiter instead of a distance, so zeroing the overhead byte is the only write the cycle performs
    // before it reports the premature delimiter.
    memcpy(expected_buffer, payload_buffer, sizeof(payload_buffer));
    expected_buffer[2] = 0;

    // Tests delimiter found too early error code
    result = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(0, result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_buffer, payload_buffer, sizeof(expected_buffer));
}

/// Verifies CRCProcessor GenerateCRCTable() for 8-bit polynomials against the table from https://crccalc.com/.
void test_crc_processor_generate_table_crc8()
{
    // CRC-8 Table (Polynomial 0x07)
    // Storing both the reference and the generated table requires 512 bytes of controller memory, which is available
    // on most existing boards, including the Arduino Uno.
    constexpr uint8_t test_crc_table[256] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77,
        0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9,
        0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B,
        0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
        0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88,
        0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16,
        0x03, 0x04, 0x0D, 0x0A, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74,
        0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
        0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E,
        0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10,
        0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55,
        0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
        0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91,
        0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF,
        0xFA, 0xFD, 0xF4, 0xF3,
    };

    // Instantiates a class object to be tested. The class constructor triggers the table generation function and fills
    // the class-specific instance of _crc_table with calculated CRC values.
    const CRCProcessor<uint8_t> crc_processor(
        0x07,  // polynomial
        0x00,  // initial_value
        0x00   // final_xor_value
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX8_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor GenerateCRCTable() for reflected 8-bit polynomials against the table from
/// https://crccalc.com/.
void test_crc_processor_generate_table_crc8_reflected()
{
    // CRC-8/MAXIM-DOW Table (Polynomial 0x31, reflected)
    constexpr uint8_t test_crc_table[256] = {
        0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41, 0x9D, 0xC3,
        0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC, 0x23, 0x7D, 0x9F, 0xC1,
        0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62, 0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81,
        0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF, 0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
        0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07, 0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47,
        0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A, 0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45,
        0xC6, 0x98, 0x7A, 0x24, 0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05,
        0xE7, 0xB9, 0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
        0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50, 0xAF, 0xF1,
        0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE, 0x32, 0x6C, 0x8E, 0xD0,
        0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73, 0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5,
        0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B, 0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
        0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16, 0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75,
        0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8, 0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54,
        0xD7, 0x89, 0x6B, 0x35,
    };

    // Instantiates a class object to be tested. The reflected flag makes the constructor bit-reverse the polynomial
    // before generating the table, so the polynomial is given in the same standard form as for non-reflected variants.
    const CRCProcessor<uint8_t> crc_processor(
        0x31,  // polynomial
        0x00,  // initial_value
        0x00,  // final_xor_value
        true   // reflected
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX8_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor GenerateCRCTable() for 16-bit polynomials against the table from https://crccalc.com/.
void test_crc_processor_generate_table_crc16()
{
    // CRC-16/CCITT-FALSE Table (Polynomial 0x1021)
    // Storing both the reference and the generated table requires 1024 bytes of controller memory, which is a stretch
    // for controllers like the Arduino Uno and comfortable on more modern systems.
    constexpr uint16_t test_crc_table[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD,
        0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A,
        0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B,
        0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861,
        0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,
        0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87,
        0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A,
        0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3,
        0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290,
        0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E,
        0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F,
        0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C,
        0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83,
        0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74,
        0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    };

    // Instantiates a class object to be tested. The class constructor triggers the table generation function and fills
    // the class-specific instance of _crc_table with calculated CRC values.
    const CRCProcessor<uint16_t> crc_processor(
        0x1021,  // polynomial
        0xFFFF,  // initial_value
        0x0000   // final_xor_value
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX16_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor GenerateCRCTable() for reflected 16-bit polynomials against the table from
/// https://crccalc.com/.
void test_crc_processor_generate_table_crc16_reflected()
{
    // CRC-16/ARC Table (Polynomial 0x8005, reflected). This table is shared by every reflected 0x8005 variant,
    // including the CRC-16/USB configuration exercised by the checksum tests below.
    // Storing both the reference and the generated table requires 1024 bytes of controller memory, which is a stretch
    // for controllers like the Arduino Uno and comfortable on more modern systems.
    constexpr uint16_t test_crc_table[256] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1,
        0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40,
        0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1,
        0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1,
        0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40,
        0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1,
        0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0,
        0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740,
        0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0,
        0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1,
        0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140,
        0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0,
        0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0,
        0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
        0x4100, 0x81C1, 0x8081, 0x4040,
    };

    // Instantiates a class object to be tested, which also generates the CRC lookup table.
    const CRCProcessor<uint16_t> crc_processor(
        0x8005,  // polynomial
        0xFFFF,  // initial_value
        0x0000,  // final_xor_value
        true     // reflected
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX16_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor GenerateCRCTable() for 32-bit polynomials against the table from https://crccalc.com/.
void test_crc_processor_generate_table_crc32()
{
    // CRC-32/XFER Table (Polynomial 0x000000AF)
    // Storing both the reference and the generated table requires 2048 bytes of controller memory, which is a stretch
    // for controllers like the Arduino Uno and comfortable on more modern systems.
    constexpr uint32_t test_crc_table[256] = {
        0x00000000, 0x000000AF, 0x0000015E, 0x000001F1, 0x000002BC, 0x00000213, 0x000003E2, 0x0000034D, 0x00000578,
        0x000005D7, 0x00000426, 0x00000489, 0x000007C4, 0x0000076B, 0x0000069A, 0x00000635, 0x00000AF0, 0x00000A5F,
        0x00000BAE, 0x00000B01, 0x0000084C, 0x000008E3, 0x00000912, 0x000009BD, 0x00000F88, 0x00000F27, 0x00000ED6,
        0x00000E79, 0x00000D34, 0x00000D9B, 0x00000C6A, 0x00000CC5, 0x000015E0, 0x0000154F, 0x000014BE, 0x00001411,
        0x0000175C, 0x000017F3, 0x00001602, 0x000016AD, 0x00001098, 0x00001037, 0x000011C6, 0x00001169, 0x00001224,
        0x0000128B, 0x0000137A, 0x000013D5, 0x00001F10, 0x00001FBF, 0x00001E4E, 0x00001EE1, 0x00001DAC, 0x00001D03,
        0x00001CF2, 0x00001C5D, 0x00001A68, 0x00001AC7, 0x00001B36, 0x00001B99, 0x000018D4, 0x0000187B, 0x0000198A,
        0x00001925, 0x00002BC0, 0x00002B6F, 0x00002A9E, 0x00002A31, 0x0000297C, 0x000029D3, 0x00002822, 0x0000288D,
        0x00002EB8, 0x00002E17, 0x00002FE6, 0x00002F49, 0x00002C04, 0x00002CAB, 0x00002D5A, 0x00002DF5, 0x00002130,
        0x0000219F, 0x0000206E, 0x000020C1, 0x0000238C, 0x00002323, 0x000022D2, 0x0000227D, 0x00002448, 0x000024E7,
        0x00002516, 0x000025B9, 0x000026F4, 0x0000265B, 0x000027AA, 0x00002705, 0x00003E20, 0x00003E8F, 0x00003F7E,
        0x00003FD1, 0x00003C9C, 0x00003C33, 0x00003DC2, 0x00003D6D, 0x00003B58, 0x00003BF7, 0x00003A06, 0x00003AA9,
        0x000039E4, 0x0000394B, 0x000038BA, 0x00003815, 0x000034D0, 0x0000347F, 0x0000358E, 0x00003521, 0x0000366C,
        0x000036C3, 0x00003732, 0x0000379D, 0x000031A8, 0x00003107, 0x000030F6, 0x00003059, 0x00003314, 0x000033BB,
        0x0000324A, 0x000032E5, 0x00005780, 0x0000572F, 0x000056DE, 0x00005671, 0x0000553C, 0x00005593, 0x00005462,
        0x000054CD, 0x000052F8, 0x00005257, 0x000053A6, 0x00005309, 0x00005044, 0x000050EB, 0x0000511A, 0x000051B5,
        0x00005D70, 0x00005DDF, 0x00005C2E, 0x00005C81, 0x00005FCC, 0x00005F63, 0x00005E92, 0x00005E3D, 0x00005808,
        0x000058A7, 0x00005956, 0x000059F9, 0x00005AB4, 0x00005A1B, 0x00005BEA, 0x00005B45, 0x00004260, 0x000042CF,
        0x0000433E, 0x00004391, 0x000040DC, 0x00004073, 0x00004182, 0x0000412D, 0x00004718, 0x000047B7, 0x00004646,
        0x000046E9, 0x000045A4, 0x0000450B, 0x000044FA, 0x00004455, 0x00004890, 0x0000483F, 0x000049CE, 0x00004961,
        0x00004A2C, 0x00004A83, 0x00004B72, 0x00004BDD, 0x00004DE8, 0x00004D47, 0x00004CB6, 0x00004C19, 0x00004F54,
        0x00004FFB, 0x00004E0A, 0x00004EA5, 0x00007C40, 0x00007CEF, 0x00007D1E, 0x00007DB1, 0x00007EFC, 0x00007E53,
        0x00007FA2, 0x00007F0D, 0x00007938, 0x00007997, 0x00007866, 0x000078C9, 0x00007B84, 0x00007B2B, 0x00007ADA,
        0x00007A75, 0x000076B0, 0x0000761F, 0x000077EE, 0x00007741, 0x0000740C, 0x000074A3, 0x00007552, 0x000075FD,
        0x000073C8, 0x00007367, 0x00007296, 0x00007239, 0x00007174, 0x000071DB, 0x0000702A, 0x00007085, 0x000069A0,
        0x0000690F, 0x000068FE, 0x00006851, 0x00006B1C, 0x00006BB3, 0x00006A42, 0x00006AED, 0x00006CD8, 0x00006C77,
        0x00006D86, 0x00006D29, 0x00006E64, 0x00006ECB, 0x00006F3A, 0x00006F95, 0x00006350, 0x000063FF, 0x0000620E,
        0x000062A1, 0x000061EC, 0x00006143, 0x000060B2, 0x0000601D, 0x00006628, 0x00006687, 0x00006776, 0x000067D9,
        0x00006494, 0x0000643B, 0x000065CA, 0x00006565,
    };

    // Instantiates a class object to be tested. The class constructor triggers the table generation function and fills
    // the class-specific instance of _crc_table with calculated CRC values.
    const CRCProcessor<uint32_t> crc_processor(
        0x000000AF,  // polynomial
        0x00000000,  // initial_value
        0x00000000   // final_xor_value
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX32_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor GenerateCRCTable() for reflected 32-bit polynomials against the table from
/// https://crccalc.com/.
void test_crc_processor_generate_table_crc32_reflected()
{
    // CRC-32/ISO-HDLC Table (Polynomial 0x04C11DB7, reflected)
    // Storing both the reference and the generated table requires 2048 bytes of controller memory, which is a stretch
    // for controllers like the Arduino Uno and comfortable on more modern systems.
    constexpr uint32_t test_crc_table[256] = {
        0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3, 0x0EDB8832,
        0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
        0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0, 0xFD62F97A,
        0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
        0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3,
        0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
        0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB,
        0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
        0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4,
        0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
        0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE, 0xA3BC0074,
        0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
        0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525,
        0x206F85B3, 0xB966D409, 0xCE61E49F, 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
        0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615,
        0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
        0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7, 0xFED41B76,
        0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
        0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B, 0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6,
        0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
        0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7,
        0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
        0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7,
        0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
        0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45, 0xA00AE278,
        0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
        0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C, 0xCABAC28A, 0x53B39330,
        0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
        0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
    };

    // Instantiates a class object to be tested, which also generates the CRC lookup table.
    const CRCProcessor<uint32_t> crc_processor(
        0x04C11DB7,  // polynomial
        0xFFFFFFFF,  // initial_value
        0xFFFFFFFF,  // final_xor_value
        true         // reflected
    );

    // Verifies that the internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX32_ARRAY(test_crc_table, crc_processor.get_crc_table(), 256);
}

/// Verifies CRCProcessor CalculateChecksum() method for 16-bit 0x1021 polynomial.
void test_crc_processor_calculate_checksum()
{
    // Generates the test buffer of size 10 with an example packet of size 6 and two placeholder values for the CRC
    // checksum. The preamble contains the start byte placeholder and the payload size (4), which is used to infer the
    // packet size (6).
    uint8_t test_packet[10] = {0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x15, 0x00, 0x00};

    // Instantiates the class object to be tested, which also generates the CRC lookup table.
    CRCProcessor<uint16_t> crc_processor(
        0x1021,  // polynomial
        0xFFFF,  // initial_value
        0x0000   // final_xor_value
    );

    // Runs the checksum generation function on the test packet
    const uint16_t result = crc_processor.CalculateChecksum<false>(test_packet);

    // Verifies that the CRC checksum generator appends the expected checksum value
    TEST_ASSERT_EQUAL_UINT8(245, test_packet[8]);  // High byte
    TEST_ASSERT_EQUAL_UINT8(78, test_packet[9]);   // Low byte

    // Verifies that the returned data + CRC postamble size matches the expected value
    TEST_ASSERT_EQUAL_UINT16(10, result);

    // Runs the checksum verification function on the packet and the appended CRC checksum postamble.
    // Ensures that the CRC checker works as expected. This relies on the known property of CRC checksums: if a CRC
    // computation runs on the data with appended CRC checksum, the resultant value is always 0. The function interprets
    // this as a '1' result.
    TEST_ASSERT_EQUAL_UINT16(1, crc_processor.CalculateChecksum<true>(test_packet));

    // Invalidates the checksum and verifies that the checker function now returns 0 to indicate data corruption.
    test_packet[9] = 11;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.CalculateChecksum<true>(test_packet));
}

/// Verifies CRCProcessor CalculateChecksum() for a polynomial configured with a non-zero final XOR value.
void test_crc_processor_nonzero_final_xor()
{
    // Generates the test buffer of size 10 with an example packet of size 6 and two placeholder values for the CRC
    // checksum, matching the layout used by the checksum test above.
    uint8_t test_packet[10] = {0x00, 0x04, 0x01, 0x02, 0x03, 0x04, 0x05, 0x15, 0x00, 0x00};

    // Instantiates the object to be tested using a non-reflected 0x8005 polynomial and a non-zero final XOR value.
    CRCProcessor<uint16_t> crc_processor(
        0x8005,  // polynomial
        0xFFFF,  // initial_value
        0xFFFF   // final_xor_value
    );

    // Runs the checksum generation function on the test packet
    const uint16_t result = crc_processor.CalculateChecksum<false>(test_packet);
    TEST_ASSERT_EQUAL_UINT16(10, result);

    // Verifies that the intact packet passes the integrity check
    TEST_ASSERT_EQUAL_UINT16(1, crc_processor.CalculateChecksum<true>(test_packet));

    // Invalidates the checksum postamble and verifies that the checker reports data corruption
    test_packet[9] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.CalculateChecksum<true>(test_packet));

    // Restores the checksum, corrupts the packet payload instead, and verifies that this is also detected
    test_packet[9] ^= 0x01;
    test_packet[4] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.CalculateChecksum<true>(test_packet));
}

/// Verifies that an instance configured with the given parameters reproduces the published CRC catalogue check value
/// and rejects both a corrupted payload and a corrupted checksum postamble.
template <typename PolynomialType>
void VerifyCatalogueCheckValue(
    const PolynomialType polynomial,
    const PolynomialType initial_value,
    const PolynomialType final_xor_value,
    const bool reflected,
    const uint8_t (&expected_postamble)[sizeof(PolynomialType)]
)
{
    // CalculateChecksum covers the overhead byte, the payload, and the delimiter byte. A payload size byte of 7
    // therefore makes the covered range exactly the nine characters of "123456789", which is the input every published
    // CRC catalogue uses to state its check value. This makes the generated postamble directly comparable to that
    // value, and it additionally pins down the byte order the configuration writes the postamble in.
    constexpr uint8_t kCheckPayloadSize = 7;
    constexpr uint8_t kPacketBytes      = 11;

    uint8_t test_packet[kPacketBytes + sizeof(PolynomialType)] =
        {0x00, kCheckPayloadSize, '1', '2', '3', '4', '5', '6', '7', '8', '9'};

    CRCProcessor<PolynomialType> crc_processor(polynomial, initial_value, final_xor_value, reflected);

    // Verifies that the returned data + CRC postamble size matches the expected value
    TEST_ASSERT_EQUAL_UINT16(sizeof(test_packet), crc_processor.template CalculateChecksum<false>(test_packet));

    // Verifies that the generated checksum postamble matches the catalogue check value
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_postamble, test_packet + kPacketBytes, sizeof(PolynomialType));

    // Verifies that the intact packet passes the integrity check
    TEST_ASSERT_EQUAL_UINT16(1, crc_processor.template CalculateChecksum<true>(test_packet));

    // Corrupts a payload byte and verifies that the checker reports data corruption
    test_packet[4] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.template CalculateChecksum<true>(test_packet));

    // Restores the payload, corrupts the checksum postamble instead, and verifies that this is also detected
    test_packet[4] ^= 0x01;
    test_packet[sizeof(test_packet) - 1] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.template CalculateChecksum<true>(test_packet));
}

/// Verifies CRCProcessor CalculateChecksum() for the non-reflected CRC-8/SMBUS configuration.
void test_crc_processor_checksum_crc8()
{
    constexpr uint8_t expected_postamble[1] = {0xF4};

    VerifyCatalogueCheckValue<uint8_t>(
        0x07,   // polynomial
        0x00,   // initial_value
        0x00,   // final_xor_value
        false,  // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the reflected CRC-8/MAXIM-DOW configuration.
void test_crc_processor_checksum_crc8_reflected()
{
    constexpr uint8_t expected_postamble[1] = {0xA1};

    VerifyCatalogueCheckValue<uint8_t>(
        0x31,  // polynomial
        0x00,  // initial_value
        0x00,  // final_xor_value
        true,  // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the non-reflected CRC-16/IBM-3740 configuration.
void test_crc_processor_checksum_crc16()
{
    constexpr uint8_t expected_postamble[2] = {0x29, 0xB1};

    VerifyCatalogueCheckValue<uint16_t>(
        0x1021,  // polynomial
        0xFFFF,  // initial_value
        0x0000,  // final_xor_value
        false,   // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the reflected CRC-16/USB configuration, which applies a non-zero
/// final XOR value.
void test_crc_processor_checksum_crc16_reflected()
{
    // The postamble is stored least significant byte first, which reverses the 0xB4C8 catalogue check value.
    constexpr uint8_t expected_postamble[2] = {0xC8, 0xB4};

    VerifyCatalogueCheckValue<uint16_t>(
        0x8005,  // polynomial
        0xFFFF,  // initial_value
        0xFFFF,  // final_xor_value
        true,    // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the non-reflected CRC-32/BZIP2 configuration, which applies a
/// non-zero final XOR value.
void test_crc_processor_checksum_crc32()
{
    constexpr uint8_t expected_postamble[4] = {0xFC, 0x89, 0x19, 0x18};

    VerifyCatalogueCheckValue<uint32_t>(
        0x04C11DB7,  // polynomial
        0xFFFFFFFF,  // initial_value
        0xFFFFFFFF,  // final_xor_value
        false,       // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the reflected CRC-32/ISO-HDLC configuration, which applies a non-zero
/// final XOR value.
void test_crc_processor_checksum_crc32_reflected()
{
    // The postamble is stored least significant byte first, which reverses the 0xCBF43926 catalogue check value.
    constexpr uint8_t expected_postamble[4] = {0x26, 0x39, 0xF4, 0xCB};

    VerifyCatalogueCheckValue<uint32_t>(
        0x04C11DB7,  // polynomial
        0xFFFFFFFF,  // initial_value
        0xFFFFFFFF,  // final_xor_value
        true,        // reflected
        expected_postamble
    );
}

/// Verifies that StreamMock class methods function correctly.
void test_stream_mock()
{
    // Instantiates the StreamMock class object to be tested. StreamMock mimics the base Stream class, but exposes
    // rx/tx buffer for direct manipulation
    StreamMock<> stream;

    // Extracts stream buffer size to a local variable
    constexpr uint16_t kStreamBufferSize = StreamMock<>::kStreamBufferSize;

    // Initializes a buffer to store the test data. Has to initialize an input buffer using uint8_t and an output
    // buffer (for test stream buffers) using int16_t. This is an unfortunate consequence of how the mock class is
    // implemented to support the behavior of the prototype stream class.
    const uint8_t test_array_in[10]  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    const int16_t test_array_out[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    // Verifies that the buffers are initialized to expected values (0)
    for (uint16_t i = 0; i < kStreamBufferSize; i++)
    {
        TEST_ASSERT_EQUAL_INT16(0, stream.rx_buffer[i]);
        TEST_ASSERT_EQUAL_INT16(0, stream.tx_buffer[i]);
    }

    // Tests available() method. It is expected to return the size of the buffer as the number of available bytes since
    // the buffers are initialized to 0, which is a valid byte-value for this class.
    const int32_t available_bytes = stream.available();  // Uses int32 for type-safety as the method returns int
    TEST_ASSERT_EQUAL_INT32(kStreamBufferSize, available_bytes);

    // Tests write() method with array input, which transfers the data from the input array to the stream's tx buffer
    const auto data_written = static_cast<int16_t>(stream.write(test_array_in, sizeof(test_array_in)));

    // Verifies that the writing operation was successful
    TEST_ASSERT_EQUAL_INT16_ARRAY(test_array_out, stream.tx_buffer, data_written);  // Checks the tx_buffer state
    TEST_ASSERT_EQUAL_size_t(data_written, stream.tx_buffer_index);

    // Tests write() method using a single-byte input (verifies byte-wise buffer filling)
    const auto byte_written = static_cast<int16_t>(stream.write(101));

    // Verifies that the addition was successful
    TEST_ASSERT_EQUAL_size_t(data_written + byte_written, stream.tx_buffer_index);
    TEST_ASSERT_EQUAL_INT16(101, stream.tx_buffer[stream.tx_buffer_index - 1]);

    // Tests reset() method, which sets both buffers to -1 and sets the rx/tx buffer indices to 0
    stream.Reset();

    // Verifies that the buffers have been reset to -1
    for (uint16_t i = 0; i < kStreamBufferSize; i++)
    {
        TEST_ASSERT_EQUAL_INT16(-1, stream.rx_buffer[i]);
        TEST_ASSERT_EQUAL_INT16(-1, stream.tx_buffer[i]);
    }

    // Also verifies that the rx_index and tx_index were reset to 0
    TEST_ASSERT_EQUAL_size_t(0, stream.rx_buffer_index);
    TEST_ASSERT_EQUAL_size_t(0, stream.tx_buffer_index);

    // Explicitly overwrites both buffers with test data
    for (uint16_t i = 0; i < static_cast<uint16_t>(sizeof(test_array_in)); i++)
    {
        stream.rx_buffer[i] = test_array_out[i];
        stream.tx_buffer[i] = test_array_out[i];
    }

    // Tests flush() function, which, for the mock class, functions as a tx_buffer-specific reset
    stream.flush();

    // Verifies that the tx buffer has been reset to -1
    for (const int16_t element : stream.tx_buffer)
    {
        TEST_ASSERT_EQUAL_INT16(-1, element);
    }

    // Verifies that the flush() method did not modify the rx buffer
    TEST_ASSERT_EQUAL_INT16_ARRAY(test_array_out, stream.rx_buffer, sizeof(test_array_in));

    // Tests peek() method, which should return the value that the current rx_buffer index is pointing at
    auto peeked_value = static_cast<uint16_t>(stream.peek());

    // Verifies that the peeked value matches the expected value written from the test_array (Should use index 0)
    TEST_ASSERT_EQUAL_INT16(test_array_out[stream.rx_buffer_index], peeked_value);

    // Also verifies that the operation does not consume the value by running it again, expecting the same value as
    // before as a response
    const auto peeked_value_2 = static_cast<uint16_t>(stream.peek());
    TEST_ASSERT_EQUAL_INT16(peeked_value, peeked_value_2);

    // Tests read() method, which is used to read a byte value from the rx buffer and 'consume' it by advancing the
    // rx_buffer_index
    auto read_value = static_cast<uint16_t>(stream.read());

    // Verifies that the consumed value is equal to the expected value peeked above
    TEST_ASSERT_EQUAL_INT16(peeked_value, read_value);

    // Consumes the remaining valid data to reach the invalid portion of the rx buffer and verifies that the read
    // data matches expected values
    for (uint8_t i = stream.rx_buffer_index; i < static_cast<uint8_t>(sizeof(test_array_in)); i++)
    {
        read_value = static_cast<uint16_t>(stream.read());
        TEST_ASSERT_EQUAL_UINT8(test_array_in[i], read_value);
    }

    // Attempts to consume an invalid value (-1) from the rx_buffer. Attempting to consume an invalid value should
    // return -1
    read_value = static_cast<uint16_t>(stream.read());

    // Verifies that the method returns -1 when attempting to read invalid data
    TEST_ASSERT_EQUAL_INT16(-1, read_value);

    // Also verifies that peek() method returns -1 when peeking invalid data
    peeked_value = static_cast<uint16_t>(stream.peek());
    TEST_ASSERT_EQUAL_INT16(-1, peeked_value);

    // Resets the rx_buffer and re-writes the test data to the buffer to test multibyte read method
    stream.Reset();
    for (uint16_t i = 0; i < static_cast<uint16_t>(sizeof(test_array_in)); i++)
    {
        stream.rx_buffer[i] = test_array_out[i];
    }

    // Initializes the test buffer
    uint8_t test_buffer[sizeof(test_array_in)] = {};

    // Reads the data from the stream buffer into the test buffer and verifies the read data matches expectation
    size_t read_bytes_number = stream.readBytes(test_buffer, sizeof(test_buffer));
    TEST_ASSERT_EQUAL_size_t(sizeof(test_buffer), read_bytes_number);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_array_in, test_buffer, sizeof(test_array_in));

    // Verifies that attempting to read the buffer filled with invalid values fails as expected
    read_bytes_number = stream.readBytes(test_buffer, sizeof(test_buffer));
    TEST_ASSERT_EQUAL_size_t(0, read_bytes_number);
}

/// Verifies WriteData() and ReadData() methods of the TransportLayer class using structures, arrays, and values.
void test_transport_layer_buffer_manipulation()
{
    // Instantiates the mock serial class and the tested TransportLayer class
    StreamMock<56> mock_port;

    // Uses different rx and tx buffer sizes
    TransportLayer<uint16_t, 56, 45> protocol(
        mock_port,
        0x1021,  // crc_polynomial
        0xFFFF,  // crc_initial_value
        0x0000   // crc_final_xor_value
    );

    // Statically extracts the buffer sizes using accessor methods.
    static constexpr uint16_t kTransmissionBufferSize =
        TransportLayer<uint16_t, 56, 45>::get_transmission_buffer_size();
    static constexpr uint16_t kReceptionBufferSize = TransportLayer<uint16_t, 56, 45>::get_reception_buffer_size();

    // Verifies the performance of payload and buffer size accessor (get) methods.
    TEST_ASSERT_EQUAL_UINT8(56, protocol.get_maximum_transmitted_payload_size());
    TEST_ASSERT_EQUAL_UINT16(62, kTransmissionBufferSize);  // Payload +  COBS (2) + Preamble (2) + Postamble (2)
    TEST_ASSERT_EQUAL_UINT8(45, protocol.get_maximum_received_payload_size());
    TEST_ASSERT_EQUAL_UINT16(51, kReceptionBufferSize);  // Payload +  COBS (2) + Preamble (2) + Postamble (2)

    // Initializes the test and expected buffers to 0. Uses two buffers due to using different sizes for reception and
    // transmission buffers. Test buffers expose the contents of the TransportLayer class internal buffers, and
    // expected buffers verify the state of the buffer contents extracted via test buffers.
    uint8_t expected_tx_buffer[kTransmissionBufferSize] = {};
    uint8_t expected_rx_buffer[kReceptionBufferSize]    = {};
    uint8_t test_tx_buffer[kTransmissionBufferSize]     = {};
    uint8_t test_rx_buffer[kReceptionBufferSize]        = {};

    // Sets all variables in expected buffers to 0 (It is expected that class buffers initialize to 0). Sets all
    // variables in the test classes to 11, so that they would be set to unexpected values should the test fail in some
    // way.
    memset(test_tx_buffer, 11, kTransmissionBufferSize);
    memset(expected_tx_buffer, 0, kTransmissionBufferSize);
    expected_tx_buffer[0] = 129;  // Accounts for the start_byte that is statically assigned at buffer instantiation.
    memset(test_rx_buffer, 11, kReceptionBufferSize);
    memset(expected_rx_buffer, 0, kReceptionBufferSize);

    // Verifies class status and buffer variables initialization (all should initialize to predicted values):

    // Transmission Buffer
    protocol.CopyTransmissionData(test_tx_buffer);  // Reads _transmission_buffer contents into the test buffer
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_tx_buffer, kTransmissionBufferSize);

    // Reception Buffer
    protocol.CopyReceptionData(test_rx_buffer);  // Reads _reception_buffer contents into the test buffer
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_rx_buffer, test_rx_buffer, kReceptionBufferSize);

    // Transfer Status
    constexpr auto kExpectedCode = static_cast<uint8_t>(kTransportStatusCodes::kStandby);
    TEST_ASSERT_EQUAL_UINT8(kExpectedCode, protocol.get_runtime_status());

    // Payload size trackers. Generally, this is a redundant check since payload size is now part of the overall buffer
    // structure, but it verifies the functioning of accessor methods.
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_transmission_buffer());
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_reception_buffer());

    // Instantiates test objects to be written to and read from the buffers

    /// Stores the test data used to verify struct serialization and deserialization.
    constexpr struct TestStruct
    {
            uint8_t byte_value          = 122;     ///< Stores the unsigned 8-bit test value.
            uint16_t short_value        = 45631;   ///< Stores the unsigned 16-bit test value.
            uint32_t long_value         = 321123;  ///< Stores the unsigned 32-bit test value.
            int8_t signed_8_bit_value   = -55;     ///< Stores the signed 8-bit test value.
            int16_t signed_16_bit_value = -8213;   ///< Stores the signed 16-bit test value.
    } PACKED_STRUCT test_structure;

    const uint8_t test_array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 101, 255};
    constexpr int32_t kTestValue = -62312;

    // Writes test objects into the _transmission_buffer
    bool status = protocol.WriteData(test_structure);
    TEST_ASSERT_TRUE(status);
    status = protocol.WriteData(test_array);
    TEST_ASSERT_TRUE(status);
    status = protocol.WriteData(kTestValue);
    TEST_ASSERT_TRUE(status);

    // Verifies that the buffer status matches the expected status (bytes successfully written)
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kObjectWrittenToBuffer),
        protocol.get_runtime_status()
    );

    // Verifies that the byte tracker matches the value expected given the byte size of all written objects
    // Combines the sizes (in bytes) of all test objects to come up with the overall payload size
    constexpr uint16_t kExpectedBytes = sizeof(test_structure) + sizeof(test_array) + sizeof(kTestValue);
    TEST_ASSERT_EQUAL_UINT16(kExpectedBytes, protocol.get_bytes_in_transmission_buffer());

    // Checks that the _transmission_buffer itself is set to the expected values. For this, overwrites the initial
    // portion of the expected_tx_buffer with the expected values of the _transmission_buffer after data has been
    // written to it. Note, all values here have been manually converted to bytes, so this partially relies on the
    // tested platform endianness.
    expected_tx_buffer[0]  = 129;
    expected_tx_buffer[1]  = 24;
    expected_tx_buffer[2]  = 0;
    expected_tx_buffer[3]  = 122;
    expected_tx_buffer[4]  = 63;
    expected_tx_buffer[5]  = 178;
    expected_tx_buffer[6]  = 99;
    expected_tx_buffer[7]  = 230;
    expected_tx_buffer[8]  = 4;
    expected_tx_buffer[9]  = 0;
    expected_tx_buffer[10] = 201;
    expected_tx_buffer[11] = 235;
    expected_tx_buffer[12] = 223;
    expected_tx_buffer[13] = 1;
    expected_tx_buffer[14] = 2;
    expected_tx_buffer[15] = 3;
    expected_tx_buffer[16] = 4;
    expected_tx_buffer[17] = 5;
    expected_tx_buffer[18] = 6;
    expected_tx_buffer[19] = 7;
    expected_tx_buffer[20] = 8;
    expected_tx_buffer[21] = 101;
    expected_tx_buffer[22] = 255;
    expected_tx_buffer[23] = 152;
    expected_tx_buffer[24] = 12;
    expected_tx_buffer[25] = 255;
    expected_tx_buffer[26] = 255;
    protocol.CopyTransmissionData(test_tx_buffer);  // Copies the _transmission_buffer contents to the test_buffer

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_tx_buffer, kTransmissionBufferSize);

    // Initializes new test objects, sets all to 0, which is different from the originally used test object values

    /// Stores zeroed test data used to verify deserialization from the reception buffer.
    struct TestStruct2
    {
            uint8_t byte_value          = 0;  ///< Stores the unsigned 8-bit test value.
            uint16_t short_value        = 0;  ///< Stores the unsigned 16-bit test value.
            uint32_t long_value         = 0;  ///< Stores the unsigned 32-bit test value.
            int8_t signed_8_bit_value   = 0;  ///< Stores the signed 8-bit test value.
            int16_t signed_16_bit_value = 0;  ///< Stores the signed 16-bit test value.
    } PACKED_STRUCT test_structure_new;

    uint8_t test_array_new[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int32_t test_value_new     = 0;

    // Copies the contents of the _transmission_buffer to the _reception_buffer to test reception buffer manipulation
    // (reading)
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_TRUE(copied);

    // Reads the data from the _reception_buffer into the newly instantiated test objects, resetting them to the
    // original test object values
    status = protocol.ReadData(test_structure_new);
    TEST_ASSERT_TRUE(status);
    status = protocol.ReadData(test_array_new);
    TEST_ASSERT_TRUE(status);
    status = protocol.ReadData(test_value_new);
    TEST_ASSERT_TRUE(status);

    // Verifies that the buffer status matches the expected status (bytes successfully read)
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kObjectReadFromBuffer),
        protocol.get_runtime_status()
    );

    // Verifies that the objects read from the buffer are the same as the original objects:
    // Structure (tests field-wise)
    TEST_ASSERT_EQUAL_UINT8(test_structure.byte_value, test_structure_new.byte_value);
    TEST_ASSERT_EQUAL_UINT16(test_structure.short_value, test_structure_new.short_value);
    TEST_ASSERT_EQUAL_UINT32(test_structure.long_value, test_structure_new.long_value);
    TEST_ASSERT_EQUAL_INT8(test_structure.signed_8_bit_value, test_structure_new.signed_8_bit_value);
    TEST_ASSERT_EQUAL_INT16(test_structure.signed_16_bit_value, test_structure_new.signed_16_bit_value);

    // Array
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_array, test_array_new, 10);

    // Value
    TEST_ASSERT_EQUAL_INT32(kTestValue, test_value_new);

    // Verifies that the reception buffer (which is basically set to the _transmission_buffer state now) was not
    // altered by the read method runtime
    memcpy(expected_rx_buffer, expected_tx_buffer, kReceptionBufferSize);  // Copies expected tx values to rx buffer
    protocol.CopyReceptionData(test_rx_buffer);  // Sets test_rx_buffer to the actual state of the rx buffer
    expected_tx_buffer[0] = 0;  // RX buffer is not set to the start byte value, so this expectation has to be corrected
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_rx_buffer, kReceptionBufferSize);
}

/// Verifies error handling by WriteData() and ReadData() methods of the TransportLayer class.
void test_transport_layer_buffer_manipulation_errors()
{
    // Initializes the tested class
    StreamMock<55> mock_port;
    // Uses same rx and tx payload sizes
    TransportLayer<uint16_t, 55, 55> protocol(
        mock_port,
        0x1021,  // crc_polynomial
        0xFFFF,  // crc_initial_value
        0x0000   // crc_final_xor_value
    );

    // Initializes the test variables
    uint8_t test_array[55] = {};

    // Verifies that writing a variable with the same size as the maximum payload size works as expected
    protocol.WriteData(test_array);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kObjectWrittenToBuffer),
        protocol.get_runtime_status()
    );

    // Verifies that attempting to write the variable to an index beyond the payload range results in an error
    bool status = protocol.WriteData(test_array);
    TEST_ASSERT_FALSE(status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError),
        protocol.get_runtime_status()
    );

    // Copies the contents of the _transmission_buffer to the _reception_buffer to test reception buffer manipulation
    // (reading)
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_TRUE(copied);

    // Verifies that reading from the end of the payload functions as expected
    protocol.ReadData(test_array);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kObjectReadFromBuffer),
        protocol.get_runtime_status()
    );

    // Verifies that attempting to read from an index beyond the payload range results in an error
    status = protocol.ReadData(test_array);
    TEST_ASSERT_FALSE(status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError),
        protocol.get_runtime_status()
    );
}

/// Verifies SendData() and ReceiveData() methods of the TransportLayer class and all supporting sub-methods.
void test_transport_layer_data_transmission()
{
    // Initializes the tested class
    StreamMock<254> mock_port;  // Minimal required size

    // Uses identical rx and tx payload sizes and tests maximal supported sizes for both buffers. Also uses a CRC-16
    // to test multibyte CRC handling.
    TransportLayer<uint16_t> protocol(
        mock_port,
        0x1021,  // crc_polynomial
        0xFFFF,  // crc_initial_value
        0x0000   // crc_final_xor_value
    );

    // Instantiates a separate CRC encoder instance used to verify processing results.
    // CRC settings must match those used by the TransportLayer instance.
    auto crc_class = CRCProcessor<uint16_t>(
        0x1021,  // polynomial
        0xFFFF,  // initial_value
        0x0000   // final_xor_value
    );

    // Generates the test array to be packaged and 'sent'
    const uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

    // Writes the package into the _transmission_buffer
    protocol.WriteData(test_array);

    // Sends the payload to the Stream buffer.
    const bool send_status = protocol.SendData();

    // Verifies that the data has been successfully sent to the Stream buffer
    TEST_ASSERT_TRUE(send_status);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Manually verifies the contents of the StreamMock class tx_buffer to confirm that the data has been
    // processed correctly:

    // Instantiates an array to simulate the _transmission_buffer after the data has been added to it.
    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PAYLOAD[10], DELIMITER, CRC[2]
    uint8_t buffer_array[16] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer. Note, assumes COBSProcessor methods have been tested before running this
    // test. Specifically, targets the 10-value payload starting from index 3. Uses the same delimiter byte value as
    // does the serial protocol class
    COBSProcessor::EncodePayload(buffer_array);

    // Calculates the CRC for the COBS-encoded buffer. Also assumes that the CRCProcessor methods have been tested
    // before running this test. The CRC calculation includes the overhead byte, the encoded payload and the inserted
    // delimiter byte. Note, the returned checksum depends on the used polynomial type.
    crc_class.CalculateChecksum<false>(buffer_array);

    // Verifies that the packet inside the StreamMock tx_buffer is the same as the packet created via the manual steps
    // above.
    for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(buffer_array)); i++)
    {
        TEST_ASSERT_EQUAL_UINT8(buffer_array[i], static_cast<uint8_t>(mock_port.tx_buffer[i]));
    }

    // Copies the fully encoded package into the rx_buffer to simulate packet reception and test ReceiveData() method.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(buffer_array) * sizeof(mock_port.rx_buffer[0]));

    // Ensures that the overhead byte copied to the rx_buffer is not zero (that the packet is COBS-encoded). This check
    // has to be true for the decoding to work as expected and not throw a 'packet already decoded' error.
    TEST_ASSERT_NOT_EQUAL_UINT16(mock_port.rx_buffer[2], 0);

    // Receives the data stored in the StreamMock reception buffer. If all steps of this process succeed, the method
    // returns 'true'.
    const bool receive_status = protocol.ReceiveData();

    // Verifies that the data has been successfully received from the StreamMock rx buffer
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_TRUE(receive_status);

    // Verifies that the internal class _reception_buffer tracker was set to the expected payload size
    TEST_ASSERT_EQUAL_UINT16(10, protocol.get_bytes_in_reception_buffer());

    // Verifies that the reverse-processed payload is the same as the original payload array. Reverse processing is
    // less involved than the forward conversion, since it needs no CRC value and no simulated COBS encoding. The test
    // assumes those methods have been fully tested beforehand.
    uint8_t decoded_array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Placeholder-initialized
    protocol.ReadData(decoded_array);                            // Reads the data from _reception_buffer

    // Verifies that the decoded payload fully matches the test payload array contents
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_array, decoded_array, sizeof(test_array));

    // Verifies that the minor Available() method works as expected. Available() returns 'true' if data to parse is
    // available and 'false' otherwise. Since the StreamMock class initializes its buffers with zeroes, which is a valid
    // data value, this method should return 'true' even after fully consuming the test payload.
    bool data_available = protocol.Available();
    TEST_ASSERT_TRUE(data_available);

    // Verifies that ResetReceptionBuffer() works as expected. ResetReceptionBuffer() resets the overhead,
    // payload_size, and consumed-payload-bytes variables of the buffer. Since the overhead is already reset by the
    // decoder method, only the payload_size action is evaluated below.
    protocol.ResetReceptionBuffer();
    TEST_ASSERT_EQUAL_UINT16(0, protocol.get_bytes_in_reception_buffer());

    // Also verifies ResetTransmissionBuffer() method, which works the same as the ResetReceptionBuffer() method, but
    // specifically targets the _transmission_buffer
    protocol.ResetTransmissionBuffer();
    TEST_ASSERT_EQUAL_UINT16(0, protocol.get_bytes_in_transmission_buffer());

    // Fully resets the mock rx_buffer with -1, which is used as a stand-in for no available data. This is to test the
    // 'false' return portion of the Available() method.
    memset(mock_port.rx_buffer, -1, 254 * sizeof(mock_port.rx_buffer[0]));  // Converts from elements to bytes

    // Verifies that available() correctly returns 'false' if no data is actually available to be read from the
    // Stream class rx_buffer
    data_available = protocol.Available();
    TEST_ASSERT_FALSE(data_available);
}

/// Verifies ReceiveData() error handling for the TransportLayer class. SendData() error handling is covered by
/// test_transport_layer_empty_payload_error() and test_transport_layer_partial_send_error().
void test_transport_layer_data_transmission_errors()
{
    // Initializes the tested class
    StreamMock<50> mock_port;  // Initializes to the minimal required size
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Stages and sends a well-formed payload, which is the transmission path this test relies on. The transmission
    // error codes are covered by the dedicated tests below, so this sequence moves straight on to testing reception.
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Verifies that the data has been 'sent' successfully
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Verifies that the algorithm correctly handles missing start byte error. By default, the algorithm is configured
    // to treat these 'errors' as 'no bytes available for reading' status, which is a non-error status
    mock_port.rx_buffer[0] = 0;  // Removes the start byte
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse),
        protocol.get_runtime_status()
    );
    mock_port.rx_buffer[0]    = 129;  // Restores the start byte
    mock_port.rx_buffer_index = 0;    // Resets readout index back to 0

    // Verifies that when not enough bytes are available to parse, the algorithm correctly aborts parsing with the
    // kNoBytesToParse code. The abort triggers when Available() does not find at least kMinimumPacketSize buffered
    // bytes.
    mock_port.rx_buffer[1] = -1;  // Essentially aborts reception at the payload_size byte value.
    const bool result      = protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_FALSE(result);
    mock_port.rx_buffer[1] = mock_port.tx_buffer[1];

    // Verifies that the algorithm correctly handles a CRC checksum error (indicates corrupted packets).
    mock_port.rx_buffer[14] = 123;  // Fake CRC byte, overwrites the crc byte value found at the end of the packet
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed),
        protocol.get_runtime_status()
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Verifies that the algorithm correctly handles missing payload_size byte errors. For the test to work,
    // the buffer has to be modified to contain valid bytes before the start byte so that the available() method
    // triggers packet reception and parsing.
    // Starts by prepending 'filler' data to the buffer before the start_byte
    const uint16_t prepended_data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // All values are non-start-byte
    memcpy(mock_port.rx_buffer, prepended_data, sizeof(prepended_data));

    // Then, adds the main data to the buffer right after the prepended data. For this, uses 'reinterpret_cast' to shift
    // the target buffer pointer to the correct location.
    memcpy(
        reinterpret_cast<uint8_t*>(mock_port.rx_buffer) + sizeof(prepended_data),
        mock_port.tx_buffer,
        kPacketElements * sizeof(mock_port.rx_buffer[0])
    );

    // Note that from now on all indices are statically shifted by 10 to account for the prepended data
    mock_port.rx_buffer[11] = -1;  // Essentially aborts reception at the payload_size byte value.
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPayloadSizeByteNotFound),
        protocol.get_runtime_status()
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Verifies that the algorithm correctly handles invalid payload_size byte errors. Tests payload_size byte
    // being too small (0) and too large (61). Note, these sizes depend on the template kMaximumReceivedPayloadSize
    // parameter and the fixed kMinimumPayloadSize constant.

    // Payload too small
    mock_port.rx_buffer[11] = 0;
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize),
        protocol.get_runtime_status()
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Payload too large
    mock_port.rx_buffer[11] = 61;
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize),
        protocol.get_runtime_status()
    );
    mock_port.rx_buffer_index = 0;   // Resets readout index back to 0
    mock_port.rx_buffer[11]   = 10;  // Restores the payload_size byte value

    // Fills the remainder of the rx_buffer with valid non-delimiter byte-values, so the parser keeps consuming bytes
    // until it reaches the invalid value inserted below.
    for (uint16_t i = 15; i < StreamMock<50>::kStreamBufferSize; i++)
    {
        mock_port.rx_buffer[i] = 11;
    }

    // Verifies that the algorithm correctly handles encountering no valid bytes for a long time as a stale packet
    // error. For that, inserts an invalid value in the middle of the packet, which will be interpreted as not receiving
    // data until the timeout guard kicks in to break the stale runtime.
    mock_port.rx_buffer[17] = -1;  // Sets byte 8 to an 'invalid' value to simulate not receiving valid bytes at index 7
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketTimeoutError),
        protocol.get_runtime_status()
    );
}

/// Verifies that ReceiveData() reports kDelimiterNotFoundError for missing delimiters.
void test_transport_layer_delimiter_not_found_error()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));
    mock_port.rx_buffer[13] = 1;  // Changes delimiter byte to non-zero

    // Simulates receiving data
    protocol.ReceiveData();

    // Verifies that the delimiter was not found
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kDelimiterNotFoundError),
        protocol.get_runtime_status()
    );
}

/// Verifies that ReceiveData() reports kDelimiterFoundTooEarlyError for premature delimiters.
void test_transport_layer_delimiter_found_too_early_error()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));
    mock_port.rx_buffer[7] = 0;  // Add delimiter value too early

    // Simulates receiving data
    protocol.ReceiveData();

    // Verifies that the delimiter was found too early
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kDelimiterFoundTooEarlyError),
        protocol.get_runtime_status()
    );
}

/// Verifies that ReceiveData() reports kPostambleTimeoutError when the postamble is not received.
void test_transport_layer_postamble_timeout_error()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Stages and sends a well-formed payload, which is the transmission path this test relies on. The transmission
    // error codes are covered by the dedicated tests below, so this sequence moves straight on to testing reception.
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Invalidates the single postamble byte, so the parser reaches the delimiter and then stalls waiting for the
    // checksum that never arrives.
    mock_port.rx_buffer[14] = -1;

    // Simulates receiving data
    protocol.ReceiveData();

    // Simulates timeout for postamble not being received
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPostambleTimeoutError),
        protocol.get_runtime_status()
    );
}

/// Verifies that ReceiveData() consumes exactly kPostambleSize bytes of the CRC checksum postamble.
void test_transport_layer_postamble_size_boundary()
{
    // Initializes the tested class. The uint8_t polynomial makes the postamble exactly one byte long, so a parser
    // that consumes a fixed multi-byte postamble reads past the end of the packet.
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Builds a well-formed packet inside the mock class tx buffer
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Invalidates the element immediately following the postamble. Reception has to stop at the postamble, so a
    // parser that consumes an extra byte stalls here and reports kPostambleTimeoutError instead.
    mock_port.rx_buffer[kPacketElements] = -1;

    // Verifies that the packet is received without reading past its postamble
    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_TRUE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );

    // Verifies that the received payload matches the transmitted payload
    uint8_t decoded_payload[10] = {};
    const bool read_status      = protocol.ReadData(decoded_payload);
    TEST_ASSERT_TRUE(read_status);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_payload, decoded_payload, sizeof(test_payload));
}

/// Verifies that SendData() reports kEmptyPayloadError when the transmission buffer holds no payload.
void test_transport_layer_empty_payload_error()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Sends without staging a payload first
    bool send_status = protocol.SendData();

    // Verifies that the transmission was rejected
    TEST_ASSERT_FALSE(send_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kEmptyPayloadError),
        protocol.get_runtime_status()
    );

    // Verifies that no bytes reached the communication interface
    TEST_ASSERT_EQUAL_size_t(0, mock_port.tx_buffer_index);

    // Verifies that a staged payload is transmitted normally
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};
    protocol.WriteData(test_payload);
    send_status = protocol.SendData();
    TEST_ASSERT_TRUE(send_status);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Verifies that sending again is rejected, as SendData() resets the transmission buffer after each transmission
    send_status = protocol.SendData();
    TEST_ASSERT_FALSE(send_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kEmptyPayloadError),
        protocol.get_runtime_status()
    );
}

/// Verifies that SendData() reports kPacketPartiallySent when the interface accepts only a part of the packet.
void test_transport_layer_partial_send_error()
{
    // Sizes the mock transmission buffer below the packet size, so that its write() method terminates early
    StreamMock<10> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload, which forms a 15-byte packet once encoded
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    protocol.WriteData(test_payload);
    const bool send_status = protocol.SendData();

    // Verifies that the truncated transmission was reported
    TEST_ASSERT_FALSE(send_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketPartiallySent),
        protocol.get_runtime_status()
    );

    // Verifies that the interface accepted only as many bytes as its buffer holds
    TEST_ASSERT_EQUAL_size_t(StreamMock<10>::kStreamBufferSize, mock_port.tx_buffer_index);

    // Verifies that the transmission buffer is reset, as a truncated packet cannot be retransmitted from it
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_transmission_buffer());
}


/// Stores the maximum packet size, in bytes, that a StallingStream instance is able to hold.
static constexpr uint16_t kStallingStreamCapacity = 64;

/**
 * @brief Releases the bytes of a preloaded packet one at a time, withholding each byte for a fixed interval before
 * reporting it as available for reading.
 *
 * StreamMock derives its available() count from static buffer contents, so any stall it produces lasts until the test
 * ends. This double instead ends every stall on its own, which is the pattern a real communication interface produces
 * when it delivers a packet in bursts and is the only way to reach the reception loops' resumption path.
 *
 * @note The instance implements the read() and available() methods that the TransportLayer class uses to receive
 * packets. The remaining Stream methods are implemented to satisfy the interface and are not exercised by the
 * reception path.
 */
class StallingStream final : public Stream
{
    public:
        /**
         * @brief Initializes the instance with the packet to release and the schedule to release it on.
         *
         * @param packet the packet bytes to copy into the instance and release to the reader.
         * @param packet_size the number of bytes the packet occupies, capped at kStallingStreamCapacity.
         * @param immediate_bytes the number of leading packet bytes the instance reports as available at once.
         * @param stall_microseconds the interval each remaining byte is withheld for before it is released.
         */
        StallingStream(
            const uint8_t* packet,
            const uint16_t packet_size,
            const uint16_t immediate_bytes,
            const uint32_t stall_microseconds
        ) :
            _packet {},
            _packet_size(packet_size < kStallingStreamCapacity ? packet_size : kStallingStreamCapacity),
            _released(immediate_bytes < _packet_size ? immediate_bytes : _packet_size),
            _stall_microseconds(stall_microseconds),
            _last_release_time(micros())
        {
            memcpy(_packet, packet, _packet_size);
        }

        /**
         * @brief Returns the number of released bytes that the reader has not yet consumed.
         *
         * Releases one additional byte whenever the configured interval has elapsed since the previous release, which
         * spaces the withheld bytes evenly without ever leaving the reader waiting indefinitely.
         *
         * @returns the number of bytes available for reading.
         */
        int available() override
        {
            const uint32_t current_time = micros();

            // Advances the release schedule by a single byte per elapsed interval, which is what separates two
            // consecutive bytes by the configured stall rather than delivering the remainder of the packet at once.
            if (_released < _packet_size && current_time - _last_release_time >= _stall_microseconds)
            {
                _released++;
                _last_release_time = current_time;
            }

            return static_cast<int>(_released - _index);
        }

        /**
         * @brief Consumes and returns the next released byte.
         *
         * @returns the consumed byte, or -1 if the reader has already consumed every released byte.
         */
        int read() override
        {
            if (_index >= _released) return -1;

            return _packet[_index++];
        }

        /**
         * @brief Returns the next released byte without consuming it.
         *
         * @returns the peeked byte, or -1 if the reader has already consumed every released byte.
         */
        int peek() override
        {
            if (_index >= _released) return -1;

            return _packet[_index];
        }

        /**
         * @brief Discards the input byte, as the instance only drives the reception path.
         *
         * @returns 1, reporting that the byte was accepted.
         */
        size_t write(const uint8_t) override
        {
            return 1;
        }

        /**
         * @brief Discards the input buffer, as the instance only drives the reception path.
         *
         * @param bytes_to_write the number of bytes the caller offered.
         * @returns the number of bytes the caller offered, reporting that all of them were accepted.
         */
        size_t write(const uint8_t*, const size_t bytes_to_write) override
        {
            return bytes_to_write;
        }

        /// Intentionally empty, as the instance stores no outgoing data to flush.
        void flush() override
        {}

        /// Defaults the destructor. Uses the 'virtual' form rather than 'override', because the Arduino Print base
        /// class declares no virtual destructor on any supported architecture, so 'override' fails to compile.
        virtual ~StallingStream() = default;

    private:
        /// Stores the packet bytes the instance releases to the reader.
        uint8_t _packet[kStallingStreamCapacity];

        /// Stores the number of bytes the released packet occupies.
        const uint16_t _packet_size;

        /// Tracks the number of packet bytes the instance has released for reading.
        uint16_t _released;

        /// Tracks the number of released bytes the reader has consumed.
        uint16_t _index = 0;

        /// Stores the interval, in microseconds, each withheld byte is held back for before it is released.
        const uint32_t _stall_microseconds;

        /// Stores the timestamp, in microseconds, of the most recent byte release.
        uint32_t _last_release_time;
};

/// Pins the numeric value and the storage width of every kTransportStatusCodes member.
void test_shared_assets_status_code_values()
{
    // The PC-side companion library decodes a reported transport status from the numeric value the microcontroller
    // places on the wire, so these numbers are part of the protocol contract rather than an internal detail. Every
    // other status assertion in this suite compares an accessor against the enum member itself, which follows any
    // renumbering silently, so each value is pinned against an explicit literal here.

    // The status travels as a single byte, so widening the underlying type would desynchronize the two libraries
    TEST_ASSERT_EQUAL_size_t(1, sizeof(kTransportStatusCodes));

    // Buffer and packet lifecycle codes
    TEST_ASSERT_EQUAL_UINT8(11, static_cast<uint8_t>(kTransportStatusCodes::kStandby));
    TEST_ASSERT_EQUAL_UINT8(12, static_cast<uint8_t>(kTransportStatusCodes::kDecodingFailed));
    TEST_ASSERT_EQUAL_UINT8(13, static_cast<uint8_t>(kTransportStatusCodes::kPacketSent));
    TEST_ASSERT_EQUAL_UINT8(14, static_cast<uint8_t>(kTransportStatusCodes::kPayloadSizeByteNotFound));
    TEST_ASSERT_EQUAL_UINT8(15, static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize));
    TEST_ASSERT_EQUAL_UINT8(16, static_cast<uint8_t>(kTransportStatusCodes::kPacketTimeoutError));
    TEST_ASSERT_EQUAL_UINT8(17, static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse));
    TEST_ASSERT_EQUAL_UINT8(18, static_cast<uint8_t>(kTransportStatusCodes::kPacketParsed));
    TEST_ASSERT_EQUAL_UINT8(19, static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed));
    TEST_ASSERT_EQUAL_UINT8(20, static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived));

    // Object serialization codes
    TEST_ASSERT_EQUAL_UINT8(21, static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError));
    TEST_ASSERT_EQUAL_UINT8(22, static_cast<uint8_t>(kTransportStatusCodes::kObjectWrittenToBuffer));
    TEST_ASSERT_EQUAL_UINT8(23, static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError));
    TEST_ASSERT_EQUAL_UINT8(24, static_cast<uint8_t>(kTransportStatusCodes::kObjectReadFromBuffer));

    // Packet framing and transmission codes
    TEST_ASSERT_EQUAL_UINT8(25, static_cast<uint8_t>(kTransportStatusCodes::kDelimiterNotFoundError));
    TEST_ASSERT_EQUAL_UINT8(26, static_cast<uint8_t>(kTransportStatusCodes::kDelimiterFoundTooEarlyError));
    TEST_ASSERT_EQUAL_UINT8(27, static_cast<uint8_t>(kTransportStatusCodes::kPostambleTimeoutError));
    TEST_ASSERT_EQUAL_UINT8(28, static_cast<uint8_t>(kTransportStatusCodes::kEmptyPayloadError));
    TEST_ASSERT_EQUAL_UINT8(29, static_cast<uint8_t>(kTransportStatusCodes::kPacketPartiallySent));
}

/// Pins every kBufferLayout constant to the absolute value the packet format requires.
void test_shared_assets_buffer_layout_constants()
{
    // These constants define the packet the PC-side companion library builds and parses, and that library hardcodes
    // the same numbers rather than importing them. Every other use in this suite and in the library sources places a
    // kBufferLayout constant on both sides of the comparison, which pins the relationship between the constants but
    // follows any change to their absolute values, so each value is pinned against an explicit literal here.

    // Payload size bounds. The lower bound forbids empty payloads and the upper bound is the COBS hard limit, beyond
    // which the overhead byte can no longer index the final delimiter.
    TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(kBufferLayout::kMinimumPayloadSize));
    TEST_ASSERT_EQUAL_UINT8(254, static_cast<uint8_t>(kBufferLayout::kMaximumPayloadSize));

    // COBS-encoded packet size bounds, which are the payload bounds plus the overhead and delimiter bytes. The upper
    // bound exceeds the byte range, which is why the constant is declared wider than its siblings.
    TEST_ASSERT_EQUAL_UINT8(3, static_cast<uint8_t>(kBufferLayout::kMinimumPacketSize));
    TEST_ASSERT_EQUAL_UINT16(256, static_cast<uint16_t>(kBufferLayout::kMaximumPacketSize));

    // Reserved byte values. Both sides of the connection have to agree on these exactly, as the receiver frames every
    // packet by scanning for the start byte and terminates the payload at the delimiter.
    TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(kBufferLayout::kDelimiterByte));
    TEST_ASSERT_EQUAL_UINT8(129, static_cast<uint8_t>(kBufferLayout::kStartByte));

    // Buffer layout indices, which fix the field order inside every transmitted and received packet
    TEST_ASSERT_EQUAL_UINT8(0, static_cast<uint8_t>(kBufferLayout::kStartByteIndex));
    TEST_ASSERT_EQUAL_UINT8(1, static_cast<uint8_t>(kBufferLayout::kPayloadSizeIndex));
    TEST_ASSERT_EQUAL_UINT8(2, static_cast<uint8_t>(kBufferLayout::kOverheadByteIndex));
    TEST_ASSERT_EQUAL_UINT8(3, static_cast<uint8_t>(kBufferLayout::kPayloadStartIndex));
}

/// Verifies COBSProcessor EncodePayload() overhead-byte arithmetic at the largest distance the scheme supports.
void test_cobs_processor_maximum_overhead_distance()
{
    // Sizes the buffer to hold the largest packet the protocol supports plus the two preamble bytes, which places the
    // delimiter byte appended past the payload at the very last index of the buffer.
    uint8_t payload_buffer[kBufferLayout::kMaximumPacketSize + 2];

    // Fills the buffer with a non-delimiter value. A maximum-size payload that holds no delimiter bytes at all is the
    // only layout that leaves the encoder's tracker at the appended delimiter byte, which is what makes the overhead
    // byte store the largest distance the class static_assert permits. The boundary-size checks in the error test
    // cannot reach this state, as they encode into a buffer that still holds a delimiter byte at index 4 left over
    // from an earlier decoding cycle, which caps their overhead byte at 2.
    constexpr uint8_t kFillerValue = 22;
    memset(payload_buffer, kFillerValue, sizeof(payload_buffer));

    payload_buffer[kBufferLayout::kPayloadSizeIndex]  = kBufferLayout::kMaximumPayloadSize;
    payload_buffer[kBufferLayout::kOverheadByteIndex] = 0;  // Zeroes the placeholder the encoder overwrites

    const uint16_t encoded_size = COBSProcessor::EncodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMaximumPacketSize), encoded_size);

    // Verifies that the overhead byte stores the maximum payload size plus one, the upper bound the class
    // static_assert declares. The returned packet size is computed from the payload size alone and therefore does not
    // depend on the distance arithmetic, making this the only assertion that pins that arithmetic at its upper bound.
    TEST_ASSERT_EQUAL_UINT8(255, payload_buffer[kBufferLayout::kOverheadByteIndex]);

    // Verifies that the delimiter byte was appended immediately past the payload rather than inside or beyond it
    constexpr uint16_t kDelimiterIndex = kBufferLayout::kMaximumPayloadSize + kBufferLayout::kPayloadStartIndex;
    TEST_ASSERT_EQUAL_UINT8(kBufferLayout::kDelimiterByte, payload_buffer[kDelimiterIndex]);

    // Verifies that the encoder left the payload untouched, as a payload holding no delimiter bytes gives the encoder
    // nothing to overwrite
    for (uint16_t i = kBufferLayout::kPayloadStartIndex; i < kDelimiterIndex; i++)
    {
        // Uses a custom message system similar to Unity Array check to provide the failed index number
        char message[50];  // Buffer for the failure message
        snprintf(message, sizeof(message), "Check failed at index: %d", i);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(kFillerValue, payload_buffer[i], message);
    }

    // Verifies that the decoder follows the maximum distance back to the appended delimiter byte, which confirms the
    // encoded overhead value is usable rather than merely numerically correct
    const uint16_t decoded_size = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kBufferLayout::kMaximumPayloadSize), decoded_size);
    TEST_ASSERT_EQUAL_UINT8(0, payload_buffer[kBufferLayout::kOverheadByteIndex]);
}

/// Verifies COBSProcessor EncodePayload() overhead-byte arithmetic when a delimiter sits at the first payload byte.
void test_cobs_processor_delimiter_at_first_payload_byte()
{
    // Prepares test assets
    uint8_t payload_buffer[16];
    memset(payload_buffer, 22, sizeof(payload_buffer));

    // Creates a test payload using the format: start [0], payload_size [1], overhead [2], payload [3 to 7] (5 total),
    // delimiter [8]. The delimiter placed at index 3 sits at the first payload byte, which is the lower bound of the
    // encoder loop and the shortest distance the overhead byte can hold.
    const uint8_t initial_packet[9] = {129, 5, 0, 0, 4, 5, 6, 7, 22};
    memcpy(payload_buffer, initial_packet, sizeof(initial_packet));

    // Expected packet after encoding. The overhead byte holds 1, the distance from index 2 to index 3, and the encoded
    // delimiter holds 5, the distance from index 3 to the delimiter appended at index 8.
    const uint8_t encoded_packet[9] = {129, 5, 1, 5, 4, 5, 6, 7, 0};

    // Expected state of the packet after decoding. The payload is reverted to the original state, the overhead is
    // reset to 0, and the delimiter byte is unchanged.
    const uint8_t decoded_packet[9] = {129, 5, 0, 0, 4, 5, 6, 7, 0};

    // Encodes test payload
    const uint16_t encoded_size = COBSProcessor::EncodePayload(payload_buffer);

    // Verifies that encoding returned expected payload size (5) + overhead + delimiter (== 7, packet size)
    TEST_ASSERT_EQUAL_UINT16(7, encoded_size);

    // Verifies the overhead byte on its own, since it is the value that regresses if the encoder loop stops before
    // reaching its lower-bound index and therefore never encodes the delimiter staged at that index
    TEST_ASSERT_EQUAL_UINT8(1, payload_buffer[kBufferLayout::kOverheadByteIndex]);

    // Verifies that the encoded payload matches the expected encoding outcome
    TEST_ASSERT_EQUAL_UINT8_ARRAY(encoded_packet, payload_buffer, sizeof(encoded_packet));

    // Decodes test payload. A successful round trip confirms that the distance stored at the first payload byte
    // actually chains to the appended delimiter, rather than merely holding a plausible-looking value.
    const uint16_t decoded_size = COBSProcessor::DecodePayload(payload_buffer);
    TEST_ASSERT_EQUAL_UINT16(5, decoded_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(decoded_packet, payload_buffer, sizeof(decoded_packet));

    // Verifies that the non-packet-related portion of the buffer was not affected by the encoding/decoding cycles
    for (uint16_t i = sizeof(encoded_packet); i < static_cast<uint16_t>(sizeof(payload_buffer)); i++)
    {
        // Uses a custom message system similar to Unity Array check to provide the failed index number
        char message[50];  // Buffer for the failure message
        snprintf(message, sizeof(message), "Check failed at index: %d", i);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(22, payload_buffer[i], message);
    }
}

/// Verifies that CRCProcessor bit-reverses the initial value of a reflected instance, using an initial value that is
/// not its own bit-reversal.
void test_crc_processor_reflected_initial_value_is_bit_reversed()
{
    // Every reflected configuration exercised elsewhere in this suite passes an initial value that is its own
    // bit-reversal (0x00, 0xFFFF, 0xFFFFFFFF), which leaves the reflection of the initial value unobservable. A zero
    // final XOR value also makes the expected residue zero, so generating and then verifying a packet succeeds no
    // matter which form of the initial value seeds the register. Only an absolute check value distinguishes the two.

    // Reflecting the 0x01 initial value seeds the checksum register with 0x80, which drives the reflected 0x31 table
    // to 0x0C over the nine characters the helper covers. Consuming the initial value verbatim would yield 0x05.
    constexpr uint8_t expected_postamble[1] = {0x0C};

    VerifyCatalogueCheckValue<uint8_t>(
        0x31,  // polynomial
        0x01,  // initial_value
        0x00,  // final_xor_value
        true,  // reflected
        expected_postamble
    );

    // Pins the mirror image of the configuration above, where the initial value is the bit-reversal of 0x01 and
    // therefore seeds the register with 0x01. An instance that skips the reflection swaps the two check values, so
    // pinning both directions leaves it no expectation it can satisfy.
    constexpr uint8_t mirrored_postamble[1] = {0x05};

    VerifyCatalogueCheckValue<uint8_t>(
        0x31,  // polynomial
        0x80,  // initial_value
        0x00,  // final_xor_value
        true,  // reflected
        mirrored_postamble
    );
}

/// Verifies that CRCProcessor applies the final XOR value of a reflected instance without bit-reversing it, using a
/// final XOR value that is not its own bit-reversal.
void test_crc_processor_reflected_final_xor_is_not_reflected()
{
    // The final XOR value is applied to the register after reflected processing has already placed it in reflected
    // form, so it enters the checksum exactly as the catalogue states it, unlike the initial value. Since the expected
    // residue is derived from the same stored final XOR value, a bit-reversed final XOR value stays self-consistent
    // across generation and verification, and only an absolute check value exposes it.

    // The reflected 0x31 configuration with a zero final XOR value produces 0xA1, which
    // test_crc_processor_checksum_crc8_reflected pins. Applying the final XOR value verbatim gives 0xA1 ^ 0x0F, while
    // bit-reversing it to 0xF0 would give 0xA1 ^ 0xF0 == 0x51.
    constexpr uint8_t expected_postamble[1] = {0xAE};

    VerifyCatalogueCheckValue<uint8_t>(
        0x31,  // polynomial
        0x00,  // initial_value
        0x0F,  // final_xor_value
        true,  // reflected
        expected_postamble
    );

    // Pins the mirror image of the configuration above, whose final XOR value is the bit-reversal of 0x0F. The two
    // check values are each other's outcome under a bit-reversing final XOR value, so neither can be met by accident.
    constexpr uint8_t mirrored_postamble[1] = {0x51};

    VerifyCatalogueCheckValue<uint8_t>(
        0x31,  // polynomial
        0x00,  // initial_value
        0xF0,  // final_xor_value
        true,  // reflected
        mirrored_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the non-reflected CRC-8/I-432-1 configuration, which applies a
/// non-zero final XOR value at the 8-bit width.
void test_crc_processor_checksum_crc8_nonzero_final_xor()
{
    // Every other non-reflected 8-bit instance in this suite passes a zero final XOR value, which collapses the
    // expected residue to zero and leaves the non-reflected residue computation unexercised at this width.
    // CRC-8/I-432-1 shares the polynomial and the initial value of the CRC-8/SMBUS configuration and differs only in
    // its 0x55 final XOR value, so its check value is the SMBUS check value XORed with that final XOR value:
    // 0xF4 ^ 0x55 == 0xA1. Verifying the generated packet drives the register to the non-zero residue 0xF9, which is
    // reached only if the residue is derived from the final XOR value rather than assumed to be zero.
    constexpr uint8_t expected_postamble[1] = {0xA1};

    VerifyCatalogueCheckValue<uint8_t>(
        0x07,   // polynomial
        0x00,   // initial_value
        0x55,   // final_xor_value
        false,  // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for the non-reflected CRC-8/MIFARE-MAD configuration, which applies a
/// non-zero initial value at the 8-bit width.
void test_crc_processor_checksum_crc8_nonzero_initial_value()
{
    // Every other non-reflected 8-bit instance in this suite seeds the checksum register with 0x00, which makes a
    // register that ignores the initial value indistinguishable from one that honors it. CRC-8/MIFARE-MAD seeds the
    // register with 0xC7 and produces the check value 0x99, whereas a zeroed register produces 0x37, the check value
    // of the otherwise identical CRC-8/GSM-A configuration. Verification cannot expose this on its own, as the same
    // register seeds both the generation and the verification pass, so the check value is pinned instead.
    constexpr uint8_t expected_postamble[1] = {0x99};

    VerifyCatalogueCheckValue<uint8_t>(
        0x1D,   // polynomial
        0xC7,   // initial_value
        0x00,   // final_xor_value
        false,  // reflected
        expected_postamble
    );
}

/// Verifies CRCProcessor CalculateChecksum() for a packet that carries the largest payload the buffer layout permits.
void test_crc_processor_maximum_payload_size()
{
    // The largest payload any other checksum test drives is 7 bytes, which keeps the processed range far from the end
    // of the buffer and hides any miscalculation of that range's extent. A maximum-size payload pushes the checksum
    // postamble flush against the end of the buffer, which is the layout where a range that runs one byte short stops
    // before the delimiter and leaves the postamble slot untouched. The size is read from the shared buffer layout,
    // which caps the payload at the COBS limit on every board.
    constexpr uint8_t kPayloadSize = kBufferLayout::kMaximumPayloadSize;

    // The packet spans the preamble, the overhead byte, the payload, and the delimiter byte. The 8-bit polynomial
    // makes the postamble a single byte, so it occupies the last element of the buffer.
    constexpr uint16_t kPacketBytes = kBufferLayout::kPayloadStartIndex + kPayloadSize + 1;

    uint8_t test_packet[kPacketBytes + 1];

    // Fills the buffer with the low byte of each index, so that every processed byte differs from its neighbors. A
    // uniform fill would let a processed range shifted by a byte at either end reproduce the same checksum.
    for (uint16_t i = 0; i < static_cast<uint16_t>(sizeof(test_packet)); i++)
    {
        test_packet[i] = static_cast<uint8_t>(i);
    }

    // Declares the maximum payload size, which is what drives the processed range to the end of the buffer
    test_packet[kBufferLayout::kPayloadSizeIndex] = kPayloadSize;

    // Seeds the postamble slot with a sentinel, so that a generation pass stopping short of the buffer end leaves a
    // recognizable value behind instead of a plausible checksum
    test_packet[kPacketBytes] = 0xAA;

    // Instantiates the object to be tested using the polynomial whose lookup table is verified against the published
    // table by test_crc_processor_generate_table_crc8
    CRCProcessor<uint8_t> crc_processor(
        0x07,  // polynomial
        0x00,  // initial_value
        0x00   // final_xor_value
    );

    // Verifies that the generated packet occupies the buffer exactly, which pins the end of the processed range
    const uint16_t result = crc_processor.CalculateChecksum<false>(test_packet);
    TEST_ASSERT_EQUAL_UINT16(sizeof(test_packet), result);

    // Verifies that the checksum overwrote the sentinel in the last element of the buffer and matches the value the
    // processed region produces
    TEST_ASSERT_EQUAL_UINT8(0x6F, test_packet[kPacketBytes]);

    // Verifies that the maximum-size packet passes the integrity check, which additionally covers the postamble
    TEST_ASSERT_EQUAL_UINT16(1, crc_processor.CalculateChecksum<true>(test_packet));

    // Corrupts the last payload byte and verifies that the checker reports data corruption
    test_packet[kPacketBytes - 2] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.CalculateChecksum<true>(test_packet));

    // Restores the payload, corrupts the delimiter byte instead, and verifies that the processed range reaches
    // through the end of the packet
    test_packet[kPacketBytes - 2] ^= 0x01;
    test_packet[kPacketBytes - 1] ^= 0x01;
    TEST_ASSERT_EQUAL_UINT16(0, crc_processor.CalculateChecksum<true>(test_packet));
}

/// Verifies that the single-byte StreamMock write() overload rejects writes once the transmission buffer is full.
void test_stream_mock_write_rejects_full_buffer()
{
    // Uses a deliberately tiny buffer, because the rejection branch only executes once every element of the
    // transmission buffer is occupied, which the default 300-element buffer makes impractical to reach byte by byte.
    StreamMock<4> mock_port;

    // Extracts the stream buffer size to a local variable, so that the assertions below track the template argument
    constexpr uint16_t kStreamBufferSize = StreamMock<4>::kStreamBufferSize;

    // Initializes the input and the expected-state buffers. Has to initialize the input buffer using uint8_t and the
    // expected buffer using int16_t, matching how the mock class stores its buffered elements.
    const uint8_t fill_bytes[kStreamBufferSize]      = {11, 22, 33, 44};
    const int16_t expected_buffer[kStreamBufferSize] = {11, 22, 33, 44};

    // Fills the transmission buffer to capacity. The multi-byte overload is expected to accept every input byte, as
    // the buffer starts out empty.
    TEST_ASSERT_EQUAL_size_t(kStreamBufferSize, mock_port.write(fill_bytes, sizeof(fill_bytes)));
    TEST_ASSERT_EQUAL_size_t(kStreamBufferSize, mock_port.tx_buffer_index);
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected_buffer, mock_port.tx_buffer, kStreamBufferSize);

    // Attempts to append one more byte. The mock has to refuse the write, since accepting it would advance the index
    // past the end of the transmission buffer and corrupt the memory that follows it.
    TEST_ASSERT_EQUAL_size_t(0, mock_port.write(static_cast<uint8_t>(99)));

    // Verifies that the rejected write neither advanced the index nor altered the already-buffered data
    TEST_ASSERT_EQUAL_size_t(kStreamBufferSize, mock_port.tx_buffer_index);
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected_buffer, mock_port.tx_buffer, kStreamBufferSize);

    // Verifies that the multi-byte overload also reports zero accepted bytes against a full buffer. This is a distinct
    // case from a partially accepted write, as the copy loop never executes even once.
    TEST_ASSERT_EQUAL_size_t(0, mock_port.write(fill_bytes, sizeof(fill_bytes)));
    TEST_ASSERT_EQUAL_size_t(kStreamBufferSize, mock_port.tx_buffer_index);
    TEST_ASSERT_EQUAL_INT16_ARRAY(expected_buffer, mock_port.tx_buffer, kStreamBufferSize);
}

/// Verifies that WriteData() rejects an object that overflows the payload region by exactly one byte.
void test_transport_layer_write_data_rejects_one_byte_overflow()
{
    // Initializes the tested class with an explicit 55-byte payload cap, which keeps the overflow boundary at the same
    // byte offset on every supported board, unlike the board-dependent default cap.
    StreamMock<60> mock_port;
    TransportLayer<uint8_t, 55, 55> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Fills all but one byte of the payload region. Approaching the boundary from a non-zero offset is what makes the
    // remaining space, rather than the payload cap alone, the quantity the write below is checked against.
    const uint8_t filler[54] = {};
    bool status              = protocol.WriteData(filler);
    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT8(54, protocol.get_bytes_in_transmission_buffer());

    // Attempts to write two bytes into the single byte of space that remains, which overruns the payload region by
    // exactly one byte and is therefore the smallest overflow the bound can admit.
    constexpr uint16_t kTwoByteValue = 0xBEEF;
    status                           = protocol.WriteData(kTwoByteValue);

    // Verifies that the write was rejected and the reason was recorded
    TEST_ASSERT_FALSE(status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError),
        protocol.get_runtime_status()
    );

    // Verifies that the rejected write left the staged payload untouched. A tracker that advanced past the cap would
    // make the next transmission announce a payload size the receiver rejects.
    TEST_ASSERT_EQUAL_UINT8(54, protocol.get_bytes_in_transmission_buffer());

    // Verifies that an object that exactly fills the remaining space is still accepted, which pins the boundary from
    // the accepting side and rules out an over-tight bound.
    constexpr uint8_t kOneByteValue = 0x5A;
    status                          = protocol.WriteData(kOneByteValue);
    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT8(55, protocol.get_bytes_in_transmission_buffer());

    // Verifies that a payload region with no space left accepts nothing further
    status = protocol.WriteData(kOneByteValue);
    TEST_ASSERT_FALSE(status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(55, protocol.get_bytes_in_transmission_buffer());
}

/// Verifies that WriteData() honors an explicitly specified object size instead of the object's full byte size.
void test_transport_layer_write_data_partial_object()
{
    // Instantiates the tested class. The mock is required by the constructor but is never driven, as this test stays
    // entirely inside the transmission buffer.
    StreamMock<55> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Statically extracts the transmission buffer size, as CopyTransmissionData() requires an exactly matching
    // destination buffer.
    static constexpr uint16_t kTransmissionBufferSize =
        TransportLayer<uint8_t, 50, 50>::get_transmission_buffer_size();

    // The source object is deliberately larger than the portion written below, so a write that falls back to the
    // object's full size stages five trailing bytes the caller never asked for.
    const uint8_t source_object[8] = {11, 22, 33, 44, 55, 66, 77, 88};
    constexpr size_t kWrittenBytes = 3;

    bool status = protocol.WriteData(source_object, kWrittenBytes);
    TEST_ASSERT_TRUE(status);

    // Verifies that the payload size tracker advanced by the requested number of bytes alone
    TEST_ASSERT_EQUAL_UINT8(kWrittenBytes, protocol.get_bytes_in_transmission_buffer());

    // Inspects the staged bytes before anything else is written, as the size tracker alone does not reveal how many
    // bytes the copy moved. The payload slot immediately past the partial write has to still hold the zero the buffer
    // was initialized with, because a copy sized by the object rather than by the request leaves the object's fourth
    // byte there.
    uint8_t staged_buffer[kTransmissionBufferSize] = {};
    protocol.CopyTransmissionData(staged_buffer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(source_object, &staged_buffer[kBufferLayout::kPayloadStartIndex], kWrittenBytes);
    TEST_ASSERT_EQUAL_UINT8(0, staged_buffer[kBufferLayout::kPayloadStartIndex + kWrittenBytes]);

    // Appends a marker byte, which has to land in the payload slot immediately past the partial write for the staged
    // payload to remain contiguous.
    constexpr uint8_t kMarkerValue = 0xA5;
    status                         = protocol.WriteData(kMarkerValue);
    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT8(kWrittenBytes + 1, protocol.get_bytes_in_transmission_buffer());

    // Moves the staged payload to the reception buffer, which is the only route through which the public interface
    // reads the payload back.
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_TRUE(copied);

    const uint8_t expected_payload[kWrittenBytes + 1] = {11, 22, 33, kMarkerValue};
    uint8_t staged_payload[kWrittenBytes + 1]         = {};

    status = protocol.ReadData(staged_payload);
    TEST_ASSERT_TRUE(status);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_payload, staged_payload, sizeof(expected_payload));
}

/// Verifies that CopyTxBufferPayloadToRxBuffer() refuses to copy a payload larger than the reception buffer holds.
void test_transport_layer_copy_payload_rejects_oversized_payload()
{
    // Uses asymmetric payload caps, which is the only configuration where the transmission buffer can stage a payload
    // that the reception buffer has no room for.
    StreamMock<55> mock_port;
    TransportLayer<uint8_t, 50, 20> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Statically extracts the reception buffer size, as CopyReceptionData() requires an exactly matching destination.
    static constexpr uint16_t kReceptionBufferSize = TransportLayer<uint8_t, 50, 20>::get_reception_buffer_size();

    // Stages a payload that fits the transmission buffer but exceeds the reception buffer's 20-byte payload region
    const uint8_t oversized_payload[30] = {};
    const bool write_status             = protocol.WriteData(oversized_payload);
    TEST_ASSERT_TRUE(write_status);
    TEST_ASSERT_EQUAL_UINT8(sizeof(oversized_payload), protocol.get_bytes_in_transmission_buffer());

    // Verifies that the copy was refused. Copying would move 30 payload bytes into a 20-byte payload region, writing
    // past the end of the reception buffer and over the instance members stored after it.
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_FALSE(copied);

    // Verifies that the refused copy left the reception buffer untouched, including its payload size tracker. The test
    // buffer is pre-filled with 11 so that a failed extraction cannot masquerade as an untouched buffer.
    uint8_t reception_buffer_state[kReceptionBufferSize] = {};
    uint8_t expected_buffer_state[kReceptionBufferSize]  = {};
    memset(reception_buffer_state, 11, kReceptionBufferSize);
    memset(expected_buffer_state, 0, kReceptionBufferSize);
    protocol.CopyReceptionData(reception_buffer_state);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_buffer_state, reception_buffer_state, kReceptionBufferSize);
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_reception_buffer());

    // Verifies that a payload filling the reception buffer's payload region exactly is still copied, which pins the
    // guard from its accepting side.
    protocol.ResetTransmissionBuffer();
    const uint8_t fitting_payload[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    TEST_ASSERT_TRUE(protocol.WriteData(fitting_payload));
    TEST_ASSERT_TRUE(protocol.CopyTxBufferPayloadToRxBuffer());
    TEST_ASSERT_EQUAL_UINT8(sizeof(fitting_payload), protocol.get_bytes_in_reception_buffer());

    // Verifies that the accepted copy moved the payload bytes themselves
    uint8_t received_payload[20] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(received_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(fitting_payload, received_payload, sizeof(fitting_payload));
}

/// Verifies that ReceiveData() clears the reception buffer when packet parsing fails.
void test_transport_layer_reception_buffer_reset_after_parse_failure()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Builds a well-formed packet inside the mock class tx buffer
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Overwrites the payload size byte with a value one above the instance's capacity. The parser commits that byte
    // to the reception buffer before it range-checks it, which makes this the cheapest way to leave the buffer's
    // payload size tracker pointing at data the instance never accepted. Derives the value from the accessor, so the
    // byte stays exactly one above the cap if the template arguments used above are ever changed.
    mock_port.rx_buffer[kBufferLayout::kPayloadSizeIndex] =
        static_cast<int16_t>(protocol.get_maximum_received_payload_size() + 1);

    // Verifies that the parser rejects the packet for the expected reason
    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_FALSE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize),
        protocol.get_runtime_status()
    );

    // Verifies that the rejected packet leaves no payload behind. Without the reset, the payload size tracker keeps
    // the invalid size the parser stored, which a caller that ignores the return value reads as a received payload.
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_reception_buffer());

    // Verifies that the discarded packet cannot be consumed as though it were a decoded payload
    uint8_t decoded_payload[10] = {};
    const bool read_status      = protocol.ReadData(decoded_payload);
    TEST_ASSERT_FALSE(read_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError),
        protocol.get_runtime_status()
    );
}

/// Verifies that ReceiveData() clears the reception buffer when packet validation fails.
void test_transport_layer_reception_buffer_reset_after_validation_failure()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Builds a well-formed packet inside the mock class tx buffer
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Flips every bit of the packet's single checksum byte. This leaves the packet framing intact, so the parser
    // completes and stores the payload size, and the packet is only rejected at the later integrity check.
    constexpr uint16_t kChecksumIndex   = kPacketElements - 1;
    mock_port.rx_buffer[kChecksumIndex] = static_cast<int16_t>(mock_port.tx_buffer[kChecksumIndex] ^ 0xFF);

    // Verifies that the integrity check rejects the corrupted packet
    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_FALSE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed),
        protocol.get_runtime_status()
    );

    // Verifies that the corrupted packet leaves no payload behind. Without the reset, the payload size tracker keeps
    // pointing at the raw COBS-encoded bytes the parser stored, which never passed the integrity check.
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_reception_buffer());

    // Verifies that the discarded packet cannot be consumed as though it were a decoded payload
    uint8_t decoded_payload[10] = {};
    const bool read_status      = protocol.ReadData(decoded_payload);
    TEST_ASSERT_FALSE(read_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError),
        protocol.get_runtime_status()
    );
}

/// Verifies that receiving a second packet rewinds the reception buffer's read cursor, making the new payload
/// readable from its first byte.
void test_transport_layer_sequential_reception_rewinds_read_cursor()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Initializes two distinct test payloads, so that reading the second packet cannot accidentally reproduce the
    // first payload's contents. Both hold delimiter values, which exercises the COBS jump chain in each direction.
    const uint8_t first_payload[10]  = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};
    const uint8_t second_payload[10] = {11, 12, 0, 14, 15, 16, 0, 18, 19, 20};

    // Both packets occupy 15 elements: the preamble (2), the COBS-encoded payload (12), and the postamble (1).
    constexpr uint16_t kPacketElements = 15;

    // Stages, sends, and receives the first packet
    TEST_ASSERT_TRUE(protocol.WriteData(first_payload));
    TEST_ASSERT_TRUE(protocol.SendData());
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));
    TEST_ASSERT_TRUE(protocol.ReceiveData());

    // Consumes the entire first payload, which is what drives the read cursor away from the start of the payload
    // region and makes the rewind observable on the next reception.
    uint8_t first_decoded[10] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(first_decoded));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(first_payload, first_decoded, sizeof(first_payload));

    // Rewinds the mock class buffers so that the second packet is written to and read from their first elements,
    // which is the state the interface presents when the next packet arrives.
    mock_port.flush();
    mock_port.rx_buffer_index = 0;

    // Stages, sends, and receives the second packet on the same instance
    TEST_ASSERT_TRUE(protocol.WriteData(second_payload));
    TEST_ASSERT_TRUE(protocol.SendData());
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));
    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(sizeof(second_payload), protocol.get_bytes_in_reception_buffer());

    // Verifies that the second payload is consumed from its first byte. Unless reception rewinds the read cursor,
    // the bytes consumed from the first payload are still counted against the second one, which leaves no readable
    // bytes behind and fails the read outright.
    uint8_t second_decoded[10] = {};
    const bool read_status     = protocol.ReadData(second_decoded);
    TEST_ASSERT_TRUE(read_status);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(second_payload, second_decoded, sizeof(second_payload));
}

/// Verifies that ReceiveData() reports kDecodingFailed when a packet clears both the framing and the integrity
/// checks but carries a broken COBS jump chain.
void test_transport_layer_decoding_failed_error()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Instantiates a separate CRC processor configured identically to the one the tested instance owns. Reaching the
    // decoding stage requires restoring the packet's checksum after the corruption applied below.
    CRCProcessor<uint8_t> crc_processor(
        0x07,  // polynomial
        0x00,  // initial_value
        0x00   // final_xor_value
    );

    // Uses a payload that holds no delimiter values, so the encoder builds a single jump leading from the overhead
    // byte straight to the delimiter appended past the payload. Breaking that one jump breaks the whole chain.
    const uint8_t test_payload[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    // Builds a well-formed packet inside the mock class tx buffer
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Mirrors the packet in a byte buffer, as the CRC helper operates on the same uint8_t layout the class buffers
    // use. The packet occupies 15 elements: the preamble (2), the encoded payload (12), and the postamble (1).
    constexpr uint16_t kPacketElements     = 15;
    constexpr uint16_t kDelimiterIndex     = kPacketElements - 2;
    uint8_t packet_buffer[kPacketElements] = {};
    for (uint16_t index = 0; index < kPacketElements; ++index)
    {
        packet_buffer[index] = static_cast<uint8_t>(mock_port.tx_buffer[index]);
    }

    // Confirms the encoded layout the corruption below depends on: the delimiter closes the packet and the overhead
    // byte stores its distance to that delimiter.
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kBufferLayout::kDelimiterByte), packet_buffer[kDelimiterIndex]);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kDelimiterIndex - kBufferLayout::kOverheadByteIndex),
        packet_buffer[kBufferLayout::kOverheadByteIndex]
    );

    // Extends the overhead byte's jump by one, which makes the decoder step past the delimiter and run out of packet
    // instead of landing on it. The corruption is invisible to the parser: it introduces no delimiter value ahead of
    // the packet's end and leaves the payload size byte untouched, so the packet still parses cleanly.
    ++packet_buffer[kBufferLayout::kOverheadByteIndex];

    // Recomputes the checksum over the corrupted packet, so that reception clears the integrity check and reaches
    // the decoding stage that this test targets.
    TEST_ASSERT_EQUAL_UINT16(kPacketElements, crc_processor.CalculateChecksum<false>(packet_buffer));

    // Publishes the corrupted packet to the mock class rx buffer to simulate data reception
    for (uint16_t index = 0; index < kPacketElements; ++index)
    {
        mock_port.rx_buffer[index] = static_cast<int16_t>(packet_buffer[index]);
    }

    // Verifies that the packet is rejected at the decoding stage
    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_FALSE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kDecodingFailed),
        protocol.get_runtime_status()
    );

    // Verifies that the undecodable packet leaves no payload behind, as its bytes remain COBS-encoded
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_bytes_in_reception_buffer());
}

/// Verifies that SendData() hands the communication interface exactly the constructed packet and nothing more.
void test_transport_layer_send_data_bounds_transmitted_byte_count()
{
    // Sizes the mock transmission buffer well above the packet, so that any byte written past the packet lands in the
    // buffer instead of being dropped by the mock's own end-of-buffer guard.
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Fills the mock buffers with the invalid-value sentinel, which makes every element the transmission touches
    // distinguishable from the elements it leaves alone.
    mock_port.Reset();

    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Derives the packet length from the layout constants rather than a literal: the preamble and overhead bytes
    // (kPayloadStartIndex), the payload, the appended delimiter byte, and the single-byte checksum postamble.
    constexpr uint16_t kExpectedPacketSize =
        kBufferLayout::kPayloadStartIndex + sizeof(test_payload) + 1 + sizeof(uint8_t);

    protocol.WriteData(test_payload);
    const bool send_status = protocol.SendData();

    TEST_ASSERT_TRUE(send_status);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Verifies that the interface received the packet and nothing else. Sending trailing buffer bytes would leave the
    // receiver treating them as the opening bytes of the next packet.
    TEST_ASSERT_EQUAL_size_t(kExpectedPacketSize, mock_port.tx_buffer_index);

    // Verifies that the element immediately past the packet still holds the sentinel written by the reset above
    TEST_ASSERT_EQUAL_INT16(-1, mock_port.tx_buffer[kExpectedPacketSize]);
}

/// Verifies that a payload of the minimum supported size survives a full transmission and reception cycle.
void test_transport_layer_minimum_payload_round_trip()
{
    // Initializes the tested class
    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // A single byte is the smallest payload the protocol accepts, so it is the value that both the transmission-side
    // and the reception-side size guards evaluate at their accepting boundary.
    constexpr uint8_t kTestValue = 0xA7;

    const bool write_status = protocol.WriteData(kTestValue);
    TEST_ASSERT_TRUE(write_status);
    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_bytes_in_transmission_buffer());

    // Verifies that the transmission side treats the minimum-size payload as valid instead of rejecting it as empty
    const bool send_status = protocol.SendData();
    TEST_ASSERT_TRUE(send_status);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // The packet occupies 6 elements: the preamble (2), the COBS-encoded payload (3), and the checksum postamble (1).
    constexpr uint16_t kPacketElements = 6;
    TEST_ASSERT_EQUAL_size_t(kPacketElements, mock_port.tx_buffer_index);

    // Loops the packet back into the mock reception buffer to simulate receiving it
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Verifies that the reception side accepts the minimum-size payload size byte instead of rejecting it as invalid
    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_TRUE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_bytes_in_reception_buffer());

    // Verifies that the single payload byte survived the round trip unchanged
    uint8_t received_value = 0;
    const bool read_status = protocol.ReadData(received_value);
    TEST_ASSERT_TRUE(read_status);
    TEST_ASSERT_EQUAL_UINT8(kTestValue, received_value);
}

/// Verifies that ReceiveData() accepts a payload size equal to the reception cap and rejects the next size up.
void test_transport_layer_received_payload_size_boundary()
{
    // Reads the cap from the accessor rather than restating the template argument, so the boundary the test drives is
    // the one the parser compares the incoming payload size byte against.
    constexpr uint8_t kReceivedCap = TransportLayer<uint8_t, 50, 50>::get_maximum_received_payload_size();

    // An at-cap payload produces a packet of the preamble (2), the COBS overhead byte, the payload, the delimiter
    // byte, and the single-byte checksum postamble. Sizing the mock to exactly that leaves the parser no slack to
    // read past the packet it is given.
    constexpr uint16_t kPacketElements = kReceivedCap + 5;

    StreamMock<kPacketElements> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Fills the payload with non-delimiter values, so the encoded packet carries its only delimiter at the end and the
    // parser has to consume every payload byte before it stops.
    uint8_t at_cap_payload[kReceivedCap] = {};
    for (uint8_t index = 0; index < kReceivedCap; ++index)
    {
        at_cap_payload[index] = static_cast<uint8_t>(index + 1);
    }

    // Transmits the at-cap payload and loops the resulting packet back into the reception path
    TEST_ASSERT_TRUE(protocol.WriteData(at_cap_payload));
    TEST_ASSERT_TRUE(protocol.SendData());
    TEST_ASSERT_EQUAL_size_t(kPacketElements, mock_port.tx_buffer_index);
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Verifies that a payload size equal to the cap is accepted. The packet it produces fills the reception buffer to
    // its final byte, so a buffer sized one byte short of an at-cap packet is caught here as well.
    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(kReceivedCap, protocol.get_bytes_in_reception_buffer());

    // Verifies that the at-cap payload survived the round trip intact
    uint8_t decoded_payload[kReceivedCap] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(decoded_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(at_cap_payload, decoded_payload, kReceivedCap);

    // Rewinds the interface read cursor, so the same packet is parsed a second time
    mock_port.rx_buffer_index = 0;

    // Raises the payload size byte by one. Nothing else about the packet changes, so the size comparison alone decides
    // the outcome of the second reception.
    mock_port.rx_buffer[kBufferLayout::kPayloadSizeIndex] = static_cast<int16_t>(kReceivedCap + 1);

    TEST_ASSERT_FALSE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kInvalidPayloadSize),
        protocol.get_runtime_status()
    );
}

/// Verifies that Available() reports a readable packet at exactly the minimum packet size and withholds it one byte
/// below that size.
void test_transport_layer_available_threshold_boundary()
{
    // Mirrors the threshold the class derives from the buffer layout: the smallest payload, the two preamble bytes,
    // the COBS overhead and delimiter bytes, and the postamble of the uint8_t checksum.
    constexpr uint16_t kMinimumPacketSize =
        kBufferLayout::kMinimumPayloadSize + kBufferLayout::kOverheadByteIndex + 2 + sizeof(uint8_t);

    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // The mock counts the valid values that precede the first invalid one, so marking the element that follows a
    // threshold-sized run makes the interface report exactly a packet's worth of readable bytes.
    mock_port.rx_buffer[kMinimumPacketSize] = -1;

    int32_t readable_bytes = mock_port.available();
    TEST_ASSERT_EQUAL_INT32(kMinimumPacketSize, readable_bytes);

    // A minimum-size packet occupies exactly this many bytes, so a threshold that withholds the report here strands
    // every such packet in the interface buffer for good.
    TEST_ASSERT_TRUE(protocol.Available());

    // Shortens the readable run by a single byte, which is the largest run that cannot hold a packet.
    mock_port.rx_buffer[kMinimumPacketSize - 1] = -1;

    readable_bytes = mock_port.available();
    TEST_ASSERT_EQUAL_INT32(kMinimumPacketSize - 1, readable_bytes);
    TEST_ASSERT_FALSE(protocol.Available());
}

/// Verifies that ReceiveData() preserves an unread payload when the communication interface holds fewer bytes than a
/// packet requires.
void test_transport_layer_short_stream_preserves_reception_buffer()
{
    // Mirrors the threshold the class derives from the buffer layout: the smallest payload, the two preamble bytes,
    // the COBS overhead and delimiter bytes, and the postamble of the uint8_t checksum.
    constexpr uint16_t kMinimumPacketSize =
        kBufferLayout::kMinimumPayloadSize + kBufferLayout::kOverheadByteIndex + 2 + sizeof(uint8_t);

    StreamMock<50> mock_port;
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Stages a payload and receives it back without consuming it, which is the state a caller that polls the
    // interface before reading the previous packet leaves the instance in.
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};
    TEST_ASSERT_TRUE(protocol.WriteData(test_payload));
    TEST_ASSERT_TRUE(protocol.SendData());

    // Copies the packet into the mock class rx buffer to simulate data reception. The packet occupies 15 elements:
    // the preamble (2), the COBS-encoded payload (12), and the CRC checksum postamble (1).
    constexpr uint16_t kPacketElements = 15;
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(sizeof(test_payload), protocol.get_bytes_in_reception_buffer());

    // Leaves the interface holding one byte fewer than a packet requires, which is the largest stream that still
    // aborts reception before any parsing begins. Filling the run with start bytes ensures that a reception which
    // reaches the parser consumes them instead of returning immediately.
    mock_port.Reset();
    for (uint16_t index = 0; index < kMinimumPacketSize - 1; ++index)
    {
        mock_port.rx_buffer[index] = kBufferLayout::kStartByte;
    }

    // Verifies that the aborted poll reports the non-error status rather than a parsing failure
    TEST_ASSERT_FALSE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kNoBytesToParse),
        protocol.get_runtime_status()
    );

    // Verifies that the payload received earlier is still staged in full
    TEST_ASSERT_EQUAL_UINT8(sizeof(test_payload), protocol.get_bytes_in_reception_buffer());

    // Verifies that the preserved payload still reads back byte for byte, which additionally confirms that the read
    // cursor was left where the earlier reception placed it.
    uint8_t decoded_payload[10] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(decoded_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_payload, decoded_payload, sizeof(test_payload));
}

/// Verifies that ReceiveData() completes a packet whose bytes arrive after stalls that each end before the reception
/// timeout, even when those stalls add up to more than that timeout.
void test_transport_layer_reception_resumes_after_short_stall()
{
    // Builds a well-formed packet by hand, which drives the reception path without a transmission having to fill a
    // mock buffer first. The layout is: START, PAYLOAD_SIZE, OVERHEAD, PAYLOAD[10], DELIMITER, CRC[1].
    uint8_t packet[15] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0};
    COBSProcessor::EncodePayload(packet);

    CRCProcessor<uint8_t> crc_processor(
        0x07,  // polynomial
        0x00,  // initial_value
        0x00   // final_xor_value
    );
    crc_processor.CalculateChecksum<false>(packet);

    // Releases exactly the number of leading bytes the class requires before it begins parsing, then spaces every
    // remaining byte by an interval that stays well below the reception timeout while the sum of those intervals
    // exceeds it. A reception that restarts its stall window on each received byte therefore completes, and one that
    // measures the whole packet against a single window abandons the packet partway through.
    constexpr uint16_t kImmediateBytes    = 6;
    constexpr uint32_t kStallMicroseconds = 2500;

    StallingStream stalling_port(packet, static_cast<uint16_t>(sizeof(packet)), kImmediateBytes, kStallMicroseconds);
    TransportLayer<uint8_t, 50, 50> protocol(
        stalling_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );

    // Verifies that the resumed reception reassembled the payload rather than a truncated or shifted copy of it
    const uint8_t expected_payload[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};
    uint8_t decoded_payload[10]        = {};
    TEST_ASSERT_EQUAL_UINT8(sizeof(expected_payload), protocol.get_bytes_in_reception_buffer());
    TEST_ASSERT_TRUE(protocol.ReadData(decoded_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_payload, decoded_payload, sizeof(expected_payload));

#ifdef RUN_WIDE_CRC_TESTS
    // Repeats the exercise with a two-byte checksum postamble, which is the only configuration in which the postamble
    // loop reads more than one byte and therefore the only one in which its own stall window is observable. The
    // interval is widened so that the two postamble bytes alone outlast the reception timeout when their arrivals are
    // measured against a single window.
    constexpr uint16_t kWideImmediateBytes    = 7;
    constexpr uint32_t kWideStallMicroseconds = 6000;

    uint8_t wide_packet[16] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0, 0};
    COBSProcessor::EncodePayload(wide_packet);

    CRCProcessor<uint16_t> wide_crc_processor(
        0x1021,  // polynomial
        0xFFFF,  // initial_value
        0x0000   // final_xor_value
    );
    wide_crc_processor.CalculateChecksum<false>(wide_packet);

    StallingStream wide_stalling_port(
        wide_packet,
        static_cast<uint16_t>(sizeof(wide_packet)),
        kWideImmediateBytes,
        kWideStallMicroseconds
    );
    TransportLayer<uint16_t, 50, 50> wide_protocol(
        wide_stalling_port,
        0x1021,  // crc_polynomial
        0xFFFF,  // crc_initial_value
        0x0000   // crc_final_xor_value
    );

    TEST_ASSERT_TRUE(wide_protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        wide_protocol.get_runtime_status()
    );

    uint8_t wide_decoded_payload[10] = {};
    TEST_ASSERT_TRUE(wide_protocol.ReadData(wide_decoded_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_payload, wide_decoded_payload, sizeof(expected_payload));
#endif
}

/// Verifies that the defaulted payload capacity template parameters resolve to the value the board's serial buffer
/// size implies.
void test_transport_layer_default_template_parameters()
{
    // The instance transfers no data here, so the mock only has to satisfy the constructor's Stream reference.
    StreamMock<8> mock_port;

    // Instantiates the class with both payload capacity parameters defaulted, which is the configuration the library
    // examples rely on and the only one that resolves the capacities from the board's serial buffer size.
    TransportLayer<uint8_t> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // Recomputes the capacity with the expression the class template declares, so the expectation follows the board
    // (254 on Teensy, 248 on SAM, 56 on AVR) instead of restating a hard-coded number. Every other capacity assertion
    // in this suite merely restates an explicit template argument, which leaves the defaults unverified.
    static constexpr uint8_t kExpectedPayloadSize =
        min(kSerialBufferSize - kMaximumPacketMetadataSize, kBufferLayout::kMaximumPayloadSize);

    TEST_ASSERT_EQUAL_UINT8(kExpectedPayloadSize, protocol.get_maximum_transmitted_payload_size());
    TEST_ASSERT_EQUAL_UINT8(kExpectedPayloadSize, protocol.get_maximum_received_payload_size());

    // Verifies that the buffers sized from the defaults follow the packet layout: payload + preamble (2) + COBS (2) +
    // postamble (1).
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(kExpectedPayloadSize) + 5,
        TransportLayer<uint8_t>::get_transmission_buffer_size()
    );
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(kExpectedPayloadSize) + 5,
        TransportLayer<uint8_t>::get_reception_buffer_size()
    );

    // The widest metadata a packet can carry is the four framing bytes (start byte, payload size, overhead byte, and
    // delimiter) plus a four-byte checksum postamble. Deriving the bound from the packet anatomy rather than from
    // kMaximumPacketMetadataSize keeps the two assertions below independent of the constant they check.
    constexpr uint16_t kWidestPacketMetadata = 8;

    // Verifies that a full-capacity payload still fits the board's serial buffer when it carries the widest postamble
    // any supported polynomial produces, which is the reservation the capping expression exists to make.
    TEST_ASSERT_TRUE(kExpectedPayloadSize + kWidestPacketMetadata <= kSerialBufferSize);

    // Verifies that the default is no smaller than it has to be: one more payload byte would either overflow the
    // board's serial buffer or exceed the COBS ceiling.
    TEST_ASSERT_TRUE(
        kExpectedPayloadSize == kBufferLayout::kMaximumPayloadSize ||
        kExpectedPayloadSize + 1 + kWidestPacketMetadata > kSerialBufferSize
    );
}

/// Verifies that TransportLayer operates at the smallest payload capacity its template parameters accept.
void test_transport_layer_minimum_payload_capacity()
{
    // Sizes the mock above the six-element packet, which leaves room to mark the element that follows the packet as
    // unavailable.
    StreamMock<8> mock_port;

    // Collapses both payload bounds onto 1, so the single legal payload size sits on the lower and the upper bound of
    // the parser's payload size check at the same time and the write path runs with one byte of available space.
    TransportLayer<uint8_t, 1, 1> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_maximum_transmitted_payload_size());
    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_maximum_received_payload_size());

    // Statically extracts the buffer sizes, as the template argument commas would otherwise split the assertion
    // macro's argument list.
    static constexpr uint16_t kTransmissionBufferSize = TransportLayer<uint8_t, 1, 1>::get_transmission_buffer_size();
    static constexpr uint16_t kReceptionBufferSize    = TransportLayer<uint8_t, 1, 1>::get_reception_buffer_size();

    // Verifies the buffer arithmetic at its minimum: payload (1) + preamble (2) + COBS (2) + postamble (1).
    TEST_ASSERT_EQUAL_UINT16(6, kTransmissionBufferSize);
    TEST_ASSERT_EQUAL_UINT16(6, kReceptionBufferSize);

    // Uses the delimiter value as the payload, so the encoder has to build a delimiter chain even at this size and
    // the decoder has to restore that byte for the comparison below to hold.
    constexpr uint8_t kPayloadByte = 0;
    TEST_ASSERT_TRUE(protocol.WriteData(kPayloadByte));
    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_bytes_in_transmission_buffer());

    // Verifies that the one byte of capacity is now exhausted, which exercises the write path at zero space left.
    TEST_ASSERT_FALSE(protocol.WriteData(kPayloadByte));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError),
        protocol.get_runtime_status()
    );

    TEST_ASSERT_TRUE(protocol.SendData());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // The smallest packet this library can produce occupies exactly six elements.
    constexpr uint16_t kPacketElements = 6;
    TEST_ASSERT_EQUAL_size_t(kPacketElements, mock_port.tx_buffer_index);

    // Copies the packet into the mock reception buffer to simulate packet reception.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketElements * sizeof(mock_port.rx_buffer[0]));

    // Invalidates the element that follows the packet. This leaves exactly as many buffered bytes as the reception
    // threshold demands, so the packet is both the smallest the parser accepts and the smallest that clears that
    // threshold.
    mock_port.rx_buffer[kPacketElements] = -1;

    // Verifies that the parser accepts a payload size byte that sits on both of its bounds at once.
    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(1, protocol.get_bytes_in_reception_buffer());

    // Initializes the destination to a value the payload cannot produce, so a read that transfers nothing is visible.
    uint8_t received_byte = 111;
    TEST_ASSERT_TRUE(protocol.ReadData(received_byte));
    TEST_ASSERT_EQUAL_UINT8(kPayloadByte, received_byte);

    // Verifies that the single payload byte is consumed, which exercises the read path at zero remaining bytes.
    TEST_ASSERT_FALSE(protocol.ReadData(received_byte));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kReadObjectBufferError),
        protocol.get_runtime_status()
    );
}

/// Verifies that TransportLayer sends and receives a payload that fills the board's transmitted payload capacity.
void test_transport_layer_maximum_payload_round_trip()
{
    // Resolves the board's default payload capacity at compile time. The capacity differs per board (254 on Teensy,
    // 248 on SAM, 56 on AVR), so every size below is derived from the accessors rather than hard-coded.
    static constexpr uint8_t kPayloadSize = TransportLayer<uint8_t>::get_maximum_transmitted_payload_size();

    // A packet occupies the preamble (2), the COBS-encoded payload (payload + 2), and the checksum postamble (1),
    // which is exactly the size of the instance's transmission buffer.
    static constexpr uint16_t kPacketSize = TransportLayer<uint8_t>::get_transmission_buffer_size();

    StreamMock<kPacketSize> mock_port;
    TransportLayer<uint8_t> protocol(
        mock_port,
        0x07,  // crc_polynomial
        0x00,  // crc_initial_value
        0x00   // crc_final_xor_value
    );

    // The round trip requires both capacities to agree, as the received payload has to fit the reception buffer.
    TEST_ASSERT_EQUAL_UINT8(kPayloadSize, protocol.get_maximum_received_payload_size());

    // Fills the payload in a loop, which keeps the AVR stack free of a large brace initializer. The payload holds no
    // delimiter values, so the overhead byte ends up measuring the distance to the delimiter appended past the
    // payload, which is payload_size + 1. On the boards whose capacity reaches the COBS ceiling that distance is 255,
    // the largest value a single overhead byte can express and the boundary the COBSProcessor static assertion
    // protects.
    uint8_t test_payload[kPayloadSize];
    for (uint16_t i = 0; i < kPayloadSize; i++)
    {
        test_payload[i] = static_cast<uint8_t>(i + 1);
    }

    // Verifies that a payload matching the capacity exactly is accepted.
    TEST_ASSERT_TRUE(protocol.WriteData(test_payload));
    TEST_ASSERT_EQUAL_UINT8(kPayloadSize, protocol.get_bytes_in_transmission_buffer());

    // Verifies that the payload region is now exhausted, which confirms the payload above filled it to the last byte.
    constexpr uint8_t kExtraByte = 7;
    TEST_ASSERT_FALSE(protocol.WriteData(kExtraByte));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kWriteObjectBufferError),
        protocol.get_runtime_status()
    );

    TEST_ASSERT_TRUE(protocol.SendData());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Verifies that the interface accepted the entire packet and that the packet advertises the full payload size.
    TEST_ASSERT_EQUAL_size_t(kPacketSize, mock_port.tx_buffer_index);
    TEST_ASSERT_EQUAL_UINT8(
        kPayloadSize,
        static_cast<uint8_t>(mock_port.tx_buffer[kBufferLayout::kPayloadSizeIndex])
    );

    // Verifies the overhead byte at its largest value. A delimiter-free payload leaves the encoder's tracker on the
    // appended delimiter, so the overhead has to span the whole payload plus that delimiter.
    TEST_ASSERT_EQUAL_UINT16(
        static_cast<uint16_t>(kPayloadSize) + 1,
        static_cast<uint16_t>(mock_port.tx_buffer[kBufferLayout::kOverheadByteIndex])
    );

    // Copies the packet into the mock reception buffer to simulate packet reception.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, kPacketSize * sizeof(mock_port.rx_buffer[0]));

    // Verifies that the parser accepts a payload size byte sitting exactly on its upper bound.
    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(kPayloadSize, protocol.get_bytes_in_reception_buffer());

    // Verifies that the full-capacity payload survives the round trip byte for byte.
    uint8_t received_payload[kPayloadSize] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(received_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_payload, received_payload, kPayloadSize);
}

/// Verifies that TransportLayer forwards the reflection flag to its CRC instance and round-trips a packet whose
/// checksum is computed least significant bit first.
void test_transport_layer_reflected_crc_round_trip()
{
    // Sizes the mock above the 15-element packet the 10-byte payload produces.
    StreamMock<24> mock_port;

    // Enables reflection through the fifth constructor argument, which every other instantiation in this suite omits.
    // The non-zero final XOR value additionally forces verification through the reflected residue branch, as a zero
    // XOR value produces the same residue in both modes. An 8-bit polynomial keeps the lookup table small enough to
    // run this test on every supported board.
    TransportLayer<uint8_t, 50, 50> protocol(
        mock_port,
        0x31,  // crc_polynomial
        0x00,  // crc_initial_value
        0xFF,  // crc_final_xor_value
        true   // crc_reflected
    );

    const uint8_t test_payload[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

    protocol.WriteData(test_payload);
    TEST_ASSERT_TRUE(protocol.SendData());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Rebuilds the expected packet using a CRC instance configured in the same reflected mode. The layout is START,
    // PAYLOAD_SIZE, OVERHEAD, PAYLOAD[10], DELIMITER, CRC[1].
    uint8_t reflected_packet[15] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0};
    COBSProcessor::EncodePayload(reflected_packet);
    auto reflected_crc = CRCProcessor<uint8_t>(
        0x31,  // polynomial
        0x00,  // initial_value
        0xFF,  // final_xor_value
        true   // reflected
    );
    TEST_ASSERT_EQUAL_UINT16(15, reflected_crc.CalculateChecksum<false>(reflected_packet));

    // Builds the same packet with reflection disabled. The two configurations share every other parameter, so the
    // checksum they produce differs only because of the reflection setting.
    uint8_t plain_packet[15] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0};
    COBSProcessor::EncodePayload(plain_packet);
    auto plain_crc = CRCProcessor<uint8_t>(
        0x31,  // polynomial
        0x00,  // initial_value
        0xFF,  // final_xor_value
        false  // reflected
    );
    plain_crc.CalculateChecksum<false>(plain_packet);

    // Pins both checksums to the values their configurations produce for this packet. Comparing the two against each
    // other alone would not detect a defect inside CRCProcessor, as such a defect moves the instance's checksum and
    // the independently computed one together. The two values are pure byte arithmetic, so they hold on every board.
    TEST_ASSERT_EQUAL_HEX8(0x98, reflected_packet[14]);
    TEST_ASSERT_EQUAL_HEX8(0x9E, plain_packet[14]);

    // Confirms that the two configurations disagree on this payload, which is what makes the packet comparison below
    // sensitive to the reflection flag reaching the CRC instance.
    TEST_ASSERT_NOT_EQUAL_UINT16(plain_packet[14], reflected_packet[14]);

    // Verifies that the transmitted packet carries the reflected checksum rather than the non-reflected one.
    for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(reflected_packet)); i++)
    {
        TEST_ASSERT_EQUAL_UINT8(reflected_packet[i], static_cast<uint8_t>(mock_port.tx_buffer[i]));
    }

    // Copies the packet into the mock reception buffer to simulate packet reception.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(reflected_packet) * sizeof(mock_port.rx_buffer[0]));

    // Verifies that validation accepts the packet, which requires the reflected residue to match the reflected
    // checksum computed over the packet and its postamble.
    TEST_ASSERT_TRUE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(10, protocol.get_bytes_in_reception_buffer());

    uint8_t decoded_payload[10] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(decoded_payload));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_payload, decoded_payload, sizeof(test_payload));

    // Verifies that a corrupted packet is still rejected, so the reflected verification path is not passing every
    // packet indiscriminately.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(reflected_packet) * sizeof(mock_port.rx_buffer[0]));
    mock_port.rx_buffer_index = 0;
    mock_port.rx_buffer[14] ^= 0x01;  // Corrupts the checksum postamble byte
    TEST_ASSERT_FALSE(protocol.ReceiveData());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kCRCCheckFailed),
        protocol.get_runtime_status()
    );
}

#ifdef RUN_WIDE_CRC_TESTS
/// Verifies that TransportLayer sends, parses, and validates a packet when a 32-bit polynomial widens the CRC
/// checksum postamble to four bytes.
void test_transport_layer_crc32_round_trip()
{
    // Sizes the mock above the 18-element packet, which leaves room to mark the element that follows the postamble
    // as unavailable.
    StreamMock<64> mock_port;

    // Uses a 32-bit polynomial, which is the only configuration that stretches the checksum postamble to four bytes
    // and therefore the only one that exercises every size the class derives from the postamble length.
    TransportLayer<uint32_t, 50, 50> protocol(
        mock_port,
        0x04C11DB7,  // crc_polynomial
        0xFFFFFFFF,  // crc_initial_value
        0xFFFFFFFF   // crc_final_xor_value
    );

    // Statically extracts the buffer sizes, as the template argument commas would otherwise split the assertion
    // macro's argument list.
    static constexpr uint16_t kTransmissionBufferSize =
        TransportLayer<uint32_t, 50, 50>::get_transmission_buffer_size();
    static constexpr uint16_t kReceptionBufferSize = TransportLayer<uint32_t, 50, 50>::get_reception_buffer_size();

    // Verifies that both staging buffers reserve four bytes for the postamble: payload (50) + preamble (2) +
    // COBS (2) + postamble (4).
    TEST_ASSERT_EQUAL_UINT16(58, kTransmissionBufferSize);
    TEST_ASSERT_EQUAL_UINT16(58, kReceptionBufferSize);

    // Instantiates a separate CRC encoder used to verify the processing results. Its settings must match those used
    // by the TransportLayer instance.
    auto crc_class = CRCProcessor<uint32_t>(
        0x04C11DB7,  // polynomial
        0xFFFFFFFF,  // initial_value
        0xFFFFFFFF   // final_xor_value
    );

    // Uses a payload with embedded delimiter values, so the packet also carries a COBS-encoded delimiter chain.
    const uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

    protocol.WriteData(test_array);
    const bool send_status = protocol.SendData();
    TEST_ASSERT_TRUE(send_status);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(kTransportStatusCodes::kPacketSent), protocol.get_runtime_status());

    // Rebuilds the expected packet by hand. The layout is START, PAYLOAD_SIZE, OVERHEAD, PAYLOAD[10], DELIMITER,
    // CRC[4], so the four checksum bytes place the last packet byte at index 17.
    uint8_t buffer_array[18] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0, 0, 0, 0};
    COBSProcessor::EncodePayload(buffer_array);

    // Verifies that the four-byte postamble extends the packet to 18 bytes rather than the 16 a two-byte checksum
    // would produce.
    TEST_ASSERT_EQUAL_UINT16(18, crc_class.CalculateChecksum<false>(buffer_array));

    // Verifies that the transmitted packet matches the manually constructed one, including all four checksum bytes.
    for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(buffer_array)); i++)
    {
        TEST_ASSERT_EQUAL_UINT8(buffer_array[i], static_cast<uint8_t>(mock_port.tx_buffer[i]));
    }
    TEST_ASSERT_EQUAL_size_t(sizeof(buffer_array), mock_port.tx_buffer_index);

    // Copies the packet into the mock reception buffer to simulate packet reception.
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(buffer_array) * sizeof(mock_port.rx_buffer[0]));

    // Invalidates the element that follows the postamble. Reception has to stop after the fourth checksum byte, so a
    // parser that consumes a fifth byte stalls here and reports kPostambleTimeoutError instead.
    mock_port.rx_buffer[sizeof(buffer_array)] = -1;

    const bool receive_status = protocol.ReceiveData();
    TEST_ASSERT_TRUE(receive_status);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kTransportStatusCodes::kPacketReceived),
        protocol.get_runtime_status()
    );
    TEST_ASSERT_EQUAL_UINT8(10, protocol.get_bytes_in_reception_buffer());

    // Verifies that the payload survives the round trip, which requires the CRC check to have passed over the packet
    // and all four of its checksum bytes.
    uint8_t decoded_array[10] = {};
    TEST_ASSERT_TRUE(protocol.ReadData(decoded_array));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_array, decoded_array, sizeof(test_array));

    // Verifies the reception threshold, which the four-byte postamble raises to nine bytes. Resets the mock first, as
    // the reception above consumed part of its buffer.
    mock_port.Reset();
    for (uint16_t i = 0; i < 8; i++)
    {
        mock_port.rx_buffer[i] = 0;
    }
    TEST_ASSERT_FALSE(protocol.Available());

    // Supplies the ninth byte, which completes the smallest packet a four-byte postamble allows: payload (1) +
    // preamble (2) + COBS (2) + postamble (4).
    mock_port.rx_buffer[8] = 0;
    TEST_ASSERT_TRUE(protocol.Available());
}
#endif

/// Specifies the test functions to be executed and controls their runtime.
int RunUnityTests()
{
    UNITY_BEGIN();

    // COBS Processor
    RUN_TEST(test_cobs_processor_encode_decode);
    RUN_TEST(test_cobs_processor_errors);

    // CRC Processor
    RUN_TEST(test_crc_processor_generate_table_crc8);
    RUN_TEST(test_crc_processor_generate_table_crc8_reflected);
    RUN_TEST(test_crc_processor_checksum_crc8);
    RUN_TEST(test_crc_processor_checksum_crc8_reflected);

#ifdef RUN_CRC16_TESTS
    RUN_TEST(test_crc_processor_generate_table_crc16);
    RUN_TEST(test_crc_processor_generate_table_crc16_reflected);
    RUN_TEST(test_crc_processor_calculate_checksum);
    RUN_TEST(test_crc_processor_nonzero_final_xor);
    RUN_TEST(test_crc_processor_checksum_crc16);
    RUN_TEST(test_crc_processor_checksum_crc16_reflected);
#endif

#ifdef RUN_WIDE_CRC_TESTS
    RUN_TEST(test_crc_processor_generate_table_crc32);
    RUN_TEST(test_crc_processor_generate_table_crc32_reflected);
    RUN_TEST(test_crc_processor_checksum_crc32);
    RUN_TEST(test_crc_processor_checksum_crc32_reflected);
#endif

    // Stream Mock
    RUN_TEST(test_stream_mock);

    // TransportLayer Write / Read Data
    RUN_TEST(test_transport_layer_buffer_manipulation);
    RUN_TEST(test_transport_layer_buffer_manipulation_errors);

    // TransportLayer Send / Receive Data
    RUN_TEST(test_transport_layer_data_transmission);
    RUN_TEST(test_transport_layer_data_transmission_errors);
    RUN_TEST(test_transport_layer_delimiter_not_found_error);
    RUN_TEST(test_transport_layer_postamble_timeout_error);
    RUN_TEST(test_transport_layer_postamble_size_boundary);
    RUN_TEST(test_transport_layer_delimiter_found_too_early_error);
    RUN_TEST(test_transport_layer_empty_payload_error);
    RUN_TEST(test_transport_layer_partial_send_error);


    // Shared Assets
    RUN_TEST(test_shared_assets_status_code_values);
    RUN_TEST(test_shared_assets_buffer_layout_constants);

    // COBS Processor boundaries
    RUN_TEST(test_cobs_processor_maximum_overhead_distance);
    RUN_TEST(test_cobs_processor_delimiter_at_first_payload_byte);

    // CRC Processor parameters
    RUN_TEST(test_crc_processor_reflected_initial_value_is_bit_reversed);
    RUN_TEST(test_crc_processor_reflected_final_xor_is_not_reflected);
    RUN_TEST(test_crc_processor_checksum_crc8_nonzero_final_xor);
    RUN_TEST(test_crc_processor_checksum_crc8_nonzero_initial_value);
    RUN_TEST(test_crc_processor_maximum_payload_size);

    // Stream Mock hardening
    RUN_TEST(test_stream_mock_write_rejects_full_buffer);

    // TransportLayer buffer bounds
    RUN_TEST(test_transport_layer_write_data_rejects_one_byte_overflow);
    RUN_TEST(test_transport_layer_write_data_partial_object);
    RUN_TEST(test_transport_layer_copy_payload_rejects_oversized_payload);

    // TransportLayer post-failure reception state
    RUN_TEST(test_transport_layer_reception_buffer_reset_after_parse_failure);
    RUN_TEST(test_transport_layer_reception_buffer_reset_after_validation_failure);
    RUN_TEST(test_transport_layer_sequential_reception_rewinds_read_cursor);
    RUN_TEST(test_transport_layer_decoding_failed_error);

    // TransportLayer transmission and reception boundaries
    RUN_TEST(test_transport_layer_send_data_bounds_transmitted_byte_count);
    RUN_TEST(test_transport_layer_minimum_payload_round_trip);
    RUN_TEST(test_transport_layer_received_payload_size_boundary);
    RUN_TEST(test_transport_layer_available_threshold_boundary);
    RUN_TEST(test_transport_layer_short_stream_preserves_reception_buffer);
    RUN_TEST(test_transport_layer_reception_resumes_after_short_stall);

    // TransportLayer configuration matrix
    RUN_TEST(test_transport_layer_default_template_parameters);
    RUN_TEST(test_transport_layer_minimum_payload_capacity);
    RUN_TEST(test_transport_layer_maximum_payload_round_trip);
    RUN_TEST(test_transport_layer_reflected_crc_round_trip);

#ifdef RUN_WIDE_CRC_TESTS
    RUN_TEST(test_transport_layer_crc32_round_trip);
#endif

    return UNITY_END();
}

// Defines the baud rates for different boards.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
static constexpr uint32_t kSerialBaudRate = 5250000;

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__) ||  \
    defined(__AVR_ATmega32U4__)
static constexpr uint32_t kSerialBaudRate = 1000000;

// For all other boards the default 9600 rate is used.
#else
static constexpr uint32_t kSerialBaudRate = 9600;
#endif

/// Runs all tests inside setup() as required by the Arduino framework for one-shot testing.
void setup()
{
    // Starts the serial connection.
    Serial.begin(kSerialBaudRate);

    // Waits ~2 seconds for the Unity test runner to establish a connection with the board Serial interface. For
    // this is less important, since it uses a USB interface which does not reset the board on connection.
    delay(2000);

    // Runs the required tests
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

/// Intentionally empty. All tests run in setup() as one-shot operations.
void loop()
{}
