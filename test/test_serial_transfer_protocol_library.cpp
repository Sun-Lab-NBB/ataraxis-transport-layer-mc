// Due to certain issues with reconnecting to teensy boards for running separate test suits, this test suit acts as a
// single centralized hub for running all available tests for all supported classes and methods of the
// SerializedTransferProtocol library. Declare all required tests using separate functions (as needed) and then add the
// tests to be evaluated to the RunUnityTests function at the bottom of this file. Comment unused tests out if needed.

// Dependencies
#include <Arduino.h>  // For Arduino functions
#include <unity.h>    // This is the C testing framework, no connection to the Unity game engine
#include "axtlmc_shared_assets.h"
#include "cobs_processor.h"   // COBSProcessor class
#include "crc_processor.h"    // CRCProcessor class
#include "stream_mock.h"      // StreamMock class required for SerializedTransferProtocol class testing
#include "transport_layer.h"  // SerializedTransferProtocol class

// This function is called automatically before each test function. Currently not used.
void setUp()
{}

// This function is called automatically after each test function. Currently not used.
void tearDown()
{}

// Tests COBSProcessor EncodePayload() and DecodePayload() methods.
void TestCOBSProcessor()
{
    // Prepares test assets
    uint8_t payload_buffer[258];                         // Initializes test buffer
    memset(payload_buffer, 22, sizeof(payload_buffer));  // Sets all values to 22
    COBSProcessor<> cobs_processor;                      // Instantiates the class object to be tested

    // Creates a test payload using the format: start [0], payload_size [1], overhead [2], payload [3 to 12] (10 total),
    // delimiter [13]
    const uint8_t initial_packet[14] = {129, 10, 0, 1, 0, 3, 0, 0, 0, 7, 0, 9, 10, 22};
    memcpy(payload_buffer, initial_packet, sizeof(initial_packet));  // Copies the payload into the buffer

    // Expected packet after encoding, used to test the encoding result
    const uint8_t encoded_packet[14] = {129, 10, 2, 1, 2, 3, 1, 1, 2, 7, 3, 9, 10, 0};

    // Expected state of the packet after decoding. They payload is reverted to original
    // state, the overhead is reset to 0, but delimiter byte is not changed. Used to test the decoding result.
    const uint8_t decoded_packet[14] = {129, 10, 0, 1, 0, 3, 0, 0, 0, 7, 0, 9, 10, 0};

    constexpr uint8_t payload_size         = 10;    // Tested payload size, for payload generated above
    constexpr uint8_t packet_size          = 12;    // Tested packet size, for the decoder test
    constexpr uint8_t delimiter_byte_value = 0x00;  // Tested delimiter byte value, uses the preferred default of 0

    // Verifies the unencoded packet matches pre-test expectations
    TEST_ASSERT_EQUAL_UINT8_ARRAY(initial_packet, payload_buffer, sizeof(initial_packet));

    // Verifies that the cobs_status is initialized to the expected standby value
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kStandby),
        cobs_processor.cobs_status
    );

    // Encodes test payload
    const uint16_t encoded_size = cobs_processor.EncodePayload(payload_buffer, delimiter_byte_value);

    // Verifies the encoding runtime status
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadEncoded),
        cobs_processor.cobs_status
    );

    // Verifies that encoding returned expected payload size (10) + overhead + delimiter (== 12, packet size)
    TEST_ASSERT_EQUAL_UINT16(packet_size, encoded_size);

    // Verifies that the encoded payload matches the expected encoding outcome
    TEST_ASSERT_EQUAL_UINT8_ARRAY(encoded_packet, payload_buffer, sizeof(encoded_packet));

    // Decodes test payload
    const uint16_t decoded_size = cobs_processor.DecodePayload(payload_buffer, delimiter_byte_value);

    // Verifies the decoding runtime status
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadDecoded),
        cobs_processor.cobs_status
    );

    // Checks that size correctly equals to packet_size - 2 (10, payload_size).
    TEST_ASSERT_EQUAL_UINT16(payload_size, decoded_size);

    // Verifies that decoding reverses the payload back to the original state. Note, this excludes the overhead and
    // the delimiter, as the decoding operation does not alter these values (hence the use of a separate tester array)
    TEST_ASSERT_EQUAL_UINT8_ARRAY(decoded_packet, payload_buffer, sizeof(decoded_packet));

    // Verifies that the non-packet-related portion of the buffer was not affected by the encoding/decoding cycles
    for (uint16_t i = sizeof(encoded_packet); i < static_cast<uint16_t>(sizeof(payload_buffer)); i++)
    {
        // Uses a custom message system similar to Unity Array check to provide the filed index number
        char message[50];  // Buffer for the failure message
        snprintf(message, sizeof(message), "Check failed at index: %d", i);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(22, payload_buffer[i], message);
    }
}

// Tests error handling for EncodePayload() and DecodePayload() COBSProcessor methods.
void TestCOBSProcessorErrors()
{
    // Instantiates the class object to be tested
    COBSProcessor<> cobs_processor;

    // Generates test buffer and sets every value inside to 22
    uint8_t payload_buffer[258];
    memset(payload_buffer, 22, sizeof(payload_buffer));
    payload_buffer[2] = 0;  // Resets the overhead placeholder to 0, otherwise the encoding attempt below will fail

    // Verifies minimum encoding and decoding payload / packet size ranges. Uses standard global buffer of size 256
    // with all values set to 22. Takes ranges from the kCOBSProcessorCodes enumerator class to benefit from the fact
    // all hard-coded settings are centralized and can be modified from one place without separately tweaking source and
    // test code.

    // Verifies that payloads with minimal size are encoded correctly
    payload_buffer[1] = static_cast<uint8_t>(kCOBSProcessorLimits::kMinPayloadSize);
    uint16_t result   = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadEncoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCOBSProcessorLimits::kMinPacketSize), result);

    // Verifies packets with minimal size are decoded correctly. Uses the packet encoded above.
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadDecoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCOBSProcessorLimits::kMinPayloadSize), result);

    // Verifies that payloads with maximal size are encoded correctly
    payload_buffer[1] = static_cast<uint8_t>(kCOBSProcessorLimits::kMaxPayloadSize);
    result            = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadEncoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCOBSProcessorLimits::kMaxPacketSize), result);

    // Verifies that packets with maximal size are decoded correctly. Uses the packet encoded above.
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadDecoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(static_cast<uint16_t>(kCOBSProcessorLimits::kMaxPayloadSize), result);

    // Verifies that unsupported (too high / too low) ranges give expected error codes that can be decoded using the
    // enumerator class. To do so, shifts the payload/packet size 1 value above or below the limit and tests for the
    // correct returned error code.

    // Tests too small payload size encoder error
    payload_buffer[1] = static_cast<uint8_t>(kCOBSProcessorLimits::kMinPayloadSize) - 1;
    result            = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderTooSmallPayloadSize),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests too small packet size decoder error. Uses the same payload size as above (packet size is derived from
    // payload size).
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderTooSmallPacketSize),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests too large payload size encoder error
    payload_buffer[1] = static_cast<uint8_t>(kCOBSProcessorLimits::kMaxPayloadSize) + 1;
    result            = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderTooLargePayloadSize),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests too large packet size decoder error. Uses the same payload size as above.
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderTooLargePacketSize),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests decoder payload (in)validation error codes, issued whenever the payload does not conform to the format
    // expected from COBS encoding. During runtime, the decoder assumes that the packages were properly encoded using
    // the COBSProcessor class and, therefore, any deviation from the expected format is likely due to the payload or
    // packet being corrupted during transmission.

    // Resets the shared buffer to default state before running the test to exclude any confounding factors from the
    // tests above
    memset(payload_buffer, 22, sizeof(payload_buffer));
    payload_buffer[2] = 0;  // Sets the overhead placeholder to 0 which is required for encoding to work

    // Introduces 'jump' variables to be encoded by the call below (since 0 is the delimiter value to be encoded)
    payload_buffer[5]  = 0;
    payload_buffer[10] = 0;

    // Encodes the payload of size 15, inserting a delimiter (0) byte at index 16, generating a packet of size 17
    payload_buffer[1]     = 15;
    uint16_t encoded_size = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT16(17, encoded_size);

    // Decodes the packet of size 13 (17-4), which is a valid size. The process should abort before the delimiter at
    // index 16 is reached with the appropriate error code. Tests both the error code and that the decoder that uses a
    // while loop exits the loop as expected instead of overwriting the 'out-of-limits' buffer memory.
    payload_buffer[1] = 13;
    result            = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderUnableToFindDelimiter),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Overwrites encoded jump variable at index 10 with the actual delimiter value. This should trigger the decoder
    // loop to break early, and issue an error code, as it encountered the delimiter before it expected it based on the
    // input packet size
    payload_buffer[10] = 0;

    // Resets the overhead back to the correct value, since the decoder overwrites it to 0 on each call, even if the
    // call produces one of the 'malformed packet' errors
    payload_buffer[2] = 3;

    payload_buffer[1] = 15;  // Also restores the payload_size to the proper size

    // Tests delimiter found too early error code
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderDelimiterFoundTooEarly),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests that calling a decoder on a packet with overhead byte set to 0 produces the expected error code
    // In this particular case, the error would correctly prevent calling decoder on the same data twice.
    // Also ensure the error takes precedence over the kDecoderDelimiterFoundTooEarly error.
    result = cobs_processor.DecodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPacketAlreadyDecoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Tests that calling an encoder on a buffer with overhead placeholder not set to 0 produces an error
    payload_buffer[2] = 3;  // Resets the overhead byte to a non-0 value

    // Tests correct kPayloadAlreadyEncoded error
    result = cobs_processor.EncodePayload(payload_buffer, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kPayloadAlreadyEncoded),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Initializes a small test buffer to test buffer-size related errors
    uint8_t test_buffer[5] = {129, 20, 0, 1, 0};

    // Attempts to encode a payload with size 20 using a buffer with size 5. This is not allowed and should trigger an
    // error
    result = cobs_processor.EncodePayload(test_buffer, 11);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kEncoderPacketLargerThanBuffer),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Same as above, but tests the error for the decoder function
    result = cobs_processor.DecodePayload(test_buffer, 11);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCOBSProcessorCodes::kDecoderPacketLargerThanBuffer),
        cobs_processor.cobs_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);
}

// Tests 8-bit CRC GenerateCRCTable() method of CRCProcessor class.
// Verifies that the table generated programmatically using input polynomial parameters matches static reference values.
// For that, uses an external source to generate the test table. Specifically, https://crccalc.com/ was used here as it
// offers pregenerated lookup tables used by the calculator itself.
void TestCRCProcessorGenerateTable_CRC8()
{
    // CRC-8 Table (Polynomial 0x07)
    // Make sure your controller has enough memory for the tested and generated tables. Here, the controller needs to
    // have 256 bytes of memory to store both tables, which should be compatible with most existing boards, including
    // Arduino Uno.
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
    // the class-specific public instance of crc_table with calculated CRC values.
    const CRCProcessor<uint8_t> crc_processor(0x07, 0x00, 0x00);

    // Verifies that internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX8_ARRAY(test_crc_table, crc_processor.crc_table, 256);
}

// Tests 16-bit CRC GenerateCRCTable() method of CRCProcessor class.
// Verifies that the table generated programmatically using input polynomial parameters matches static reference values.
// For that, uses an external source to generate the test table. Specifically, https://crccalc.com/ was used here as it
// offers pregenerated lookup tables used by the calculator itself.
void TestCRCProcessorGenerateTable_CRC16()
{
    // CRC-16/CCITT-FALSE Table (Polynomial 0x1021)
    // Make sure your controller has enough memory for the tested and generated tables. Here, the controller needs to
    // have 1024 bytes of memory to store both tables, which will be a stretch for controllers like Arduino Uno (but
    // not more modern and advanced systems).
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
    // the class-specific public instance of crc_table with calculated CRC values.
    const CRCProcessor<uint16_t> crc_processor(0x1021, 0xFFFF, 0x0000);

    // Verifies that internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX16_ARRAY(test_crc_table, crc_processor.crc_table, 256);
}

// Tests 32-bit CRC GenerateCRCTable() method of CRCProcessor class.
// Verifies that the table generated programmatically using input polynomial parameters matches static reference values.
// For that, uses an external source to generate the test table. Specifically, https://crccalc.com/ was used here as it
// offers pregenerated lookup tables used by the calculator itself.
void TestCRCProcessorGenerateTable_CRC32()
{
    // CRC-32/XFER Table (Polynomial 0x000000AF)
    // Make sure your controller has enough memory for the tested and generated tables. Here, the controller needs to
    // have 2048 bytes of memory to store both tables, which will be a stretch for controllers like Arduino Uno (but
    // not more modern and advanced systems).
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
    // the class-specific public instance of crc_table with calculated CRC values.
    const CRCProcessor<uint32_t> crc_processor(0x000000AF, 0x00000000, 0x00000000);

    // Verifies that internally created CRC table matches the external table
    TEST_ASSERT_EQUAL_HEX32_ARRAY(test_crc_table, crc_processor.crc_table, 256);
}

// Tests CRCProcessor class CalculatePacketCRCChecksum(), AddCRCChecksumToBuffer() and ReadCRCChecksumFromBuffer()
// methods. Relies on the TestCRCProcessorGenerateTable functions to verify lookup table generation for all supported
// CRCs before running this test. All tests here are calibrated for 16-bit 0x1021 polynomial and will not work for
// any other polynomial.
void TestCRCProcessor()
{
    // Generates the test buffer of size 8 with an example packet of size 6 and two placeholder values
    uint8_t test_packet[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x15, 0x00, 0x00};

    // Instantiates the class object to be tested, which also generates a crc_table.
    CRCProcessor<uint16_t> crc_processor(0x1021, 0xFFFF, 0x0000);

    // Verifies that the crc_status initializes to the expected value
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kStandby),
        crc_processor.crc_status
    );
    // Runs the checksum generation function on the test packet
    uint16_t result = crc_processor.CalculatePacketCRCChecksum(test_packet, 0, 6);

    // Verifies that the CRC checksum generator returns the expected number and status
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_HEX16(0xF54E, result);

    // Stuffs the CRC checksum into the test buffer
    const uint16_t buffer_size = crc_processor.AddCRCChecksumToBuffer(test_packet, 6, result);

    // Verifies that the addition function works as expected and returns the correct used size of the buffer and status
    // code
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumAddedToBuffer),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_UINT16(8, buffer_size);

    // Runs the checksum on the packet and the two CRC bytes appended to it
    result = crc_processor.CalculatePacketCRCChecksum(test_packet, 0, 8);

    // Ensures that including CRC checksum in the input buffer correctly returns 0. This is a standard property of
    // CRC checksums often used in-place of direct checksum comparison when CRC-verified payload is checked upon
    // reception for errors.
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumCalculated),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Extracts the CRC checksum from the buffer
    const uint16_t extracted_checksum = crc_processor.ReadCRCChecksumFromBuffer(test_packet, 6);

    // Verifies that the checksum is correctly extracted from buffer using the expected value check and status check
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCRCChecksumReadFromBuffer),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_HEX16(0xF54E, extracted_checksum);
}

// Tests error handling for CalculatePacketCRCChecksum(), AddCRCChecksumToBuffer() and ReadCRCChecksumFromBuffer()
// of CRCProcessor class.
void TestCRCProcessorErrors()
{
    // Generates a small test buffer
    uint8_t test_buffer[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    // Instantiates the class object to be tested, which also generates a crc_table.
    CRCProcessor<uint16_t> crc_processor(0x1021, 0xFFFF, 0x0000);

    // Attempts to generate a CRC for the buffer above using an incorrect input packet_size of 11. Since this is smaller
    // than the buffer size of 5, the function should return 0 (default error return) and set the crc_status to an error
    // code (this is the critical part tested here).
    uint16_t checksum = crc_processor.CalculatePacketCRCChecksum(test_buffer, 0, 11);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kCalculateCRCChecksumBufferTooSmall),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, checksum);

    // Generates the checksum of the test buffer using correct input parameters
    checksum = crc_processor.CalculatePacketCRCChecksum(test_buffer, 0, 5);

    // Verifies that the AddCRCChecksumToBuffer function raises the correct error if the input buffer size is too small
    // to accommodate enough bytes to store the crc checksum starting at the start_index. Here, start
    // index of 4 is inside the buffer, but 2 bytes are needed for crc 16 checksum and index 5 is not available, leading
    // to an error.
    uint16_t result = crc_processor.AddCRCChecksumToBuffer(test_buffer, 4, checksum);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kAddCRCChecksumBufferTooSmall),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);

    // Same as above, but for the GetCRCChecksumFromBuffer function (same idea, index 5 is needed, but is not available
    // to read the CRC from it).
    result = crc_processor.ReadCRCChecksumFromBuffer(test_buffer, 4);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kCRCProcessorCodes::kReadCRCChecksumBufferTooSmall),
        crc_processor.crc_status
    );
    TEST_ASSERT_EQUAL_UINT16(0, result);
}

// Tests that the StreamMock class methods function as expected. This is a fairly minor, but necessary test to carry out
// before testing major SerializedTransferProtocol methods.
void TestStreamMock()
{
    // Instantiates the StreamMock class object to be tested. StreamMock mimics the base Stream class, but exposes
    // rx/tx buffer for direct manipulation
    StreamMock<> stream;

    // Extracts stream buffer size to a local variable
    uint16_t constexpr stream_buffer_size = StreamMock<>::buffer_size;

    // Initializes a buffer to store the test data. Has to initialize an input buffer using uint8_t and an output
    // buffer (for test stream buffers) using int16_t. This is an unfortunate consequence of how the mock class is
    // implemented to support the behavior of the prototype stream class.
    const uint8_t test_array_in[10]  = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    const int16_t test_array_out[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    // Verifies that the buffers are initialized to expected values (0)
    for (uint16_t i = 0; i < stream_buffer_size; i++)
    {
        TEST_ASSERT_EQUAL_INT16(0, stream.rx_buffer[i]);
        TEST_ASSERT_EQUAL_INT16(0, stream.tx_buffer[i]);
    }

    // Tests available() method. It is expected to return the size of the buffer as the number of available bytes since
    // the buffers are initialized to 0, which is a valid byte-value for this class.
    const int32_t available_bytes = stream.available();  // Have to use int32 for type-safety as method returns int
    TEST_ASSERT_EQUAL_INT32(stream_buffer_size, available_bytes);

    // Tests write() method with array input, which transfers the data from the test array to the stream tx buffer
    const auto data_written = static_cast<int16_t>(stream.write(test_array_in, sizeof(test_array_in)));

    // Verifies that the writing operation was successful
    TEST_ASSERT_EQUAL_INT16_ARRAY(test_array_out, stream.tx_buffer, data_written);  // Checks the tx_buffer state
    TEST_ASSERT_EQUAL_size_t(data_written, stream.tx_buffer_index);

    // Tests write() method using a single-byte input (verifies byte-wise buffer filling)
    const auto byte_written = static_cast<int16_t>(stream.write(101));

    // Verifies that the addition was successful
    TEST_ASSERT_EQUAL_size_t(data_written + byte_written, stream.tx_buffer_index);
    TEST_ASSERT_EQUAL_INT16(101, stream.tx_buffer[stream.tx_buffer_index - 1]);

    // Tests reset() method, which sets booth buffers to -1 and sets the rx/tx buffer indices to 0
    stream.reset();

    // Verifies that the buffers have been reset to -1
    for (uint16_t i = 0; i < stream_buffer_size; i++)
    {
        TEST_ASSERT_EQUAL_INT16(-1, stream.rx_buffer[i]);
        TEST_ASSERT_EQUAL_INT16(-1, stream.tx_buffer[i]);
    }

    // Also verifies that the tx_index was reset to 0
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
    for (uint16_t i = 0; i < stream_buffer_size; i++)  // NOLINT(*-loop-convert)
    {
        TEST_ASSERT_EQUAL_INT16(-1, stream.tx_buffer[i]);
    }

    // Verifies that the flush() method did not modify the rx buffer
    TEST_ASSERT_EQUAL_INT16_ARRAY(test_array_out, stream.rx_buffer, sizeof(test_array_in));

    // Tests peek() method, which should return the value that the current rx_buffer index is pointing at
    auto peeked_value = static_cast<uint16_t>(stream.peek());

    // Verifies that the peeked value matches expected value written from the test_array (Should use index 0)
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
    // return - 1
    read_value = static_cast<uint16_t>(stream.read());

    // Verifies that the method returns -1 when attempting to read invalid data
    TEST_ASSERT_EQUAL_INT16(-1, read_value);

    // Also verifies that peek() method returns -1 when peeking invalid data
    peeked_value = static_cast<uint16_t>(stream.peek());
    TEST_ASSERT_EQUAL_INT16(-1, peeked_value);

    // Resets the rx_buffer and re-writes the test data to the buffer to test multibyte read method
    stream.reset();
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

// Tests WriteData() and ReadData() methods of SerializedTransferProtocol class. The test is performed as a cycle to
// allow reusing test assets. Tests writing and reading a structure, an array and a concrete value. Also, this is the
// only method that verifies that the class variables initialize to the expected constant values and that tests using
// different transmission and reception buffer sizes.
void TestSerializedTransferProtocolBufferManipulation()
{
    // Instantiates the mock serial class and the tested SerializedTransferProtocol class
    StreamMock<> mock_port;

    // Note, uses different maximum payload size for the Rx and Tx buffers
    TransportLayer<uint16_t, 254, 80> protocol(mock_port, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000, false);

    // Statically extracts the buffer sizes using accessor methods.
    static constexpr uint16_t tx_buffer_size = TransportLayer<uint16_t, '\xfe', 'P'>::get_tx_buffer_size();
    static constexpr uint16_t rx_buffer_size = TransportLayer<uint16_t, '\xfe', 'P'>::get_rx_buffer_size();

    // Verifies the performance of payload and buffer size accessor (get) methods.
    TEST_ASSERT_EQUAL_UINT8(254, protocol.get_maximum_tx_payload_size());
    TEST_ASSERT_EQUAL_UINT16(260, tx_buffer_size);  // Payload +  COBS (2) + Preamble (2) + Postamble (2)
    TEST_ASSERT_EQUAL_UINT8(80, protocol.get_maximum_rx_payload_size());
    TEST_ASSERT_EQUAL_UINT16(86, rx_buffer_size);  // Payload +  COBS (2) + Preamble (2) + Postamble (2)

    // Initializes two test and expected buffers to 0. Uses two buffers due to using different sizes for reception and
    // transmission buffers. Test buffers are used to expose the contents of the STP class iternal buffers, and expected
    // buffers are used to verify the state of the buffer contents extracted via test buffers.
    uint8_t expected_tx_buffer[tx_buffer_size] = {};
    uint8_t expected_rx_buffer[rx_buffer_size] = {};
    uint8_t test_tx_buffer[tx_buffer_size]     = {};
    uint8_t test_rx_buffer[rx_buffer_size]     = {};

    // Sets all variables in expected buffers to 0 (It is expected that class buffers initialize to 0). Sets all
    // variables in tests classes to 11, so that they would be set to unexpected values should the test fail in some
    // way.
    memset(test_tx_buffer, 11, tx_buffer_size);
    memset(expected_tx_buffer, 0, tx_buffer_size);
    expected_tx_buffer[0] = 129;  // Accounts for the start_byte that is statically assigned at buffer instantiation.
    memset(test_rx_buffer, 11, rx_buffer_size);
    memset(expected_rx_buffer, 0, rx_buffer_size);

    // Verifies class status and buffer variables initialization (all should initialize to predicted values):

    // Transmission Buffer
    protocol.CopyTxDataToBuffer(test_tx_buffer);  // Reads _transmission_buffer contents into the test buffer
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_tx_buffer, tx_buffer_size);

    // Reception Buffer
    protocol.CopyRxDataToBuffer(test_rx_buffer);  // Reads _reception_buffer contents into the test buffer
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_rx_buffer, test_rx_buffer, rx_buffer_size);

    // Transfer Status
    constexpr auto expected_code = static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kStandby);
    TEST_ASSERT_EQUAL_UINT8(expected_code, protocol.transfer_status);

    // Payload size trackers. Generally, this is a redundant check since payload size is now part of the overall buffer
    // structure, but it verifies the functioning of accessor methods.
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_tx_payload_size());
    TEST_ASSERT_EQUAL_UINT8(0, protocol.get_rx_payload_size());

    // Instantiates test objects to be written to and read from the buffers
    struct TestStruct
    {
            uint8_t byte_value       = 122;
            uint16_t short_value     = 45631;
            uint32_t long_value      = 321123;
            int8_t signed_8b_value   = -55;
            int16_t signed_16b_value = -8213;
            int32_t signed_32b_value = -62312;
    } __attribute__((packed)) test_structure;

    const uint16_t test_array[15] = {1, 2, 3, 4, 5, 6, 7, 8, 101, 256, 1234, 7834, 15643, 38123, 65321};
    constexpr int32_t test_value  = -765;

    // Writes test objects into the _transmission_buffer
    uint16_t next_index = 0;
    next_index          = protocol.WriteData(test_structure, next_index);
    next_index          = protocol.WriteData(test_array, next_index);
    next_index          = protocol.WriteData(test_value, next_index);

    // Verifies that the buffer status matches the expected status (bytes successfully written)
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kObjectWrittenToBuffer,
        protocol.transfer_status
    );

    // Verifies that transmission bytes tracker matches the value returned by the final write operation
    TEST_ASSERT_EQUAL_UINT16(next_index, protocol.get_tx_payload_size());

    // Verifies that the payload size tracker does not change if one of the already written bytes is overwritten and
    // keeps the same value as achieved by the chain of the write operations above
    const uint16_t new_index = protocol.WriteData(test_structure, 0);  // Re-writes the structure to the same place
    TEST_ASSERT_NOT_EQUAL_UINT16(new_index, protocol.get_tx_payload_size());  // Should not be the same
    TEST_ASSERT_EQUAL_UINT16(next_index, protocol.get_tx_payload_size());     // Should be the same

    // Verifies that bytes' tracker matches the value expected given the byte-size of all written objects
    // Combines the sizes (in bytes) of all test objects to come up with the overall payload size
    constexpr uint16_t expected_bytes = sizeof(test_structure) + sizeof(test_array) + sizeof(test_value);
    TEST_ASSERT_EQUAL_UINT16(expected_bytes, protocol.get_tx_payload_size());

    // Checks that the _transmission_buffer itself is set to the expected values. For this, overwrites the initial
    // portion of the expected_tx_buffer with the expected values of the _transmission_buffer after data has been
    // written to it. Note, all values here have been manually converted to bytes, so this partially relies on the
    // tested platform endianness.
    expected_tx_buffer[0]  = 129;
    expected_tx_buffer[1]  = 48;
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
    expected_tx_buffer[13] = 152;
    expected_tx_buffer[14] = 12;
    expected_tx_buffer[15] = 255;
    expected_tx_buffer[16] = 255;
    expected_tx_buffer[17] = 1;
    expected_tx_buffer[18] = 0;
    expected_tx_buffer[19] = 2;
    expected_tx_buffer[20] = 0;
    expected_tx_buffer[21] = 3;
    expected_tx_buffer[22] = 0;
    expected_tx_buffer[23] = 4;
    expected_tx_buffer[24] = 0;
    expected_tx_buffer[25] = 5;
    expected_tx_buffer[26] = 0;
    expected_tx_buffer[27] = 6;
    expected_tx_buffer[28] = 0;
    expected_tx_buffer[29] = 7;
    expected_tx_buffer[30] = 0;
    expected_tx_buffer[31] = 8;
    expected_tx_buffer[32] = 0;
    expected_tx_buffer[33] = 101;
    expected_tx_buffer[34] = 0;
    expected_tx_buffer[35] = 0;
    expected_tx_buffer[36] = 1;
    expected_tx_buffer[37] = 210;
    expected_tx_buffer[38] = 4;
    expected_tx_buffer[39] = 154;
    expected_tx_buffer[40] = 30;
    expected_tx_buffer[41] = 27;
    expected_tx_buffer[42] = 61;
    expected_tx_buffer[43] = 235;
    expected_tx_buffer[44] = 148;
    expected_tx_buffer[45] = 41;
    expected_tx_buffer[46] = 255;
    expected_tx_buffer[47] = 3;
    expected_tx_buffer[48] = 253;
    expected_tx_buffer[49] = 255;
    expected_tx_buffer[50] = 255;
    protocol.CopyTxDataToBuffer(test_tx_buffer);  // Copies the _transmission_buffer contents to the test_buffer

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_tx_buffer, tx_buffer_size);

    // Initializes new test objects, sets all to 0, which is different from the originally used test object values
    struct TestStruct2
    {
            uint8_t byte_value       = 0;
            uint16_t short_value     = 0;
            uint32_t long_value      = 0;
            int8_t signed_8b_value   = 0;
            int16_t signed_16b_value = 0;
            int32_t signed_32b_value = 0;
    } __attribute__((packed)) test_structure_new;

    uint16_t test_array_new[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int32_t test_value_new      = 0;

    // Copies the contents of the _transmission_buffer to the _reception_buffer to test reception buffer manipulation
    // (reading)
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_TRUE(copied);

    // Reads the data from the _reception_buffer into the newly instantiated test objects, resetting them to the
    // original test object values
    uint16_t bytes_read = 0;
    bytes_read          = protocol.ReadData(test_structure_new, bytes_read);

    // Verifies that the bytes-read does NOT match reception bytes tracker, since bytes_in_reception_buffer is not
    // modified by the read method
    TEST_ASSERT_NOT_EQUAL_UINT16(bytes_read, protocol.get_rx_payload_size());

    // Continues reading data from the _transmission_buffer
    bytes_read = protocol.ReadData(test_array_new, bytes_read);
    bytes_read = protocol.ReadData(test_value_new, bytes_read);

    // Now should be equal, as the whole payload has been effectively consumed
    TEST_ASSERT_EQUAL_UINT16(bytes_read, protocol.get_rx_payload_size());

    // Verifies that the buffer status matches the expected status (bytes successfully read)
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kObjectReadFromBuffer,
        protocol.transfer_status
    );

    // Verifies that the objects read from the buffer are the same as the original objects:
    // Structure (tests field-wise)
    TEST_ASSERT_EQUAL_UINT8(test_structure.byte_value, test_structure_new.byte_value);
    TEST_ASSERT_EQUAL_UINT16(test_structure.short_value, test_structure_new.short_value);
    TEST_ASSERT_EQUAL_UINT32(test_structure.long_value, test_structure_new.long_value);
    TEST_ASSERT_EQUAL_INT8(test_structure.signed_8b_value, test_structure_new.signed_8b_value);
    TEST_ASSERT_EQUAL_INT16(test_structure.signed_16b_value, test_structure_new.signed_16b_value);
    TEST_ASSERT_EQUAL_INT32(test_structure.signed_32b_value, test_structure_new.signed_32b_value);

    // Array
    TEST_ASSERT_EQUAL_UINT16_ARRAY(test_array, test_array_new, 15);

    // Value
    TEST_ASSERT_EQUAL_INT32(test_value, test_value_new);

    // Verifies that the reception buffer (which is basically set to the _transmission_buffer state now) was not
    // altered by the read method runtime
    memcpy(expected_rx_buffer, expected_tx_buffer, rx_buffer_size);  // Copies expected values from tx to rx buffer
    protocol.CopyRxDataToBuffer(test_rx_buffer);  // Sets test_rx_buffer to the actual state of the rx buffer
    expected_tx_buffer[0] = 0;  // RX buffer is not set to the start byte value, so this expectation has to be corrected
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_tx_buffer, test_rx_buffer, rx_buffer_size);
}

// Tests expected error handling by WriteData() and ReadData() methods of SerializedTransferProtocol class. This is a
// fairly minor function, as buffer reading and writing can only fail in a small subset of cases. Uses the same payload
// size for the _reception_buffer and the _transmission_buffer.
void TestSerializedTransferProtocolBufferManipulationErrors()
{
    // Initializes the tested class
    StreamMock<> mock_port;
    // Uses identical rx and tx payload sizes
    TransportLayer<uint16_t, 60, 60> protocol(mock_port, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000, false);

    // Initializes a test variable
    uint8_t test_value = 223;

    // Verifies that writing the variable to the last valid index of the payload works as expected and returns a valid
    // payload size and status code
    protocol.WriteData(test_value, TransportLayer<uint16_t, '<', '<'>::get_maximum_tx_payload_size() - 1);
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kObjectWrittenToBuffer,
        protocol.transfer_status
    );

    // Verifies that attempting to write the variable to an index beyond the payload range results in an error
    uint16_t error_index =
        protocol.WriteData(test_value, TransportLayer<uint16_t, '<', '<'>::get_maximum_tx_payload_size());
    TEST_ASSERT_EQUAL_UINT16(0, error_index);
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kWriteObjectBufferError,
        protocol.transfer_status
    );

    // Copies the contents of the _transmission_buffer to the _reception_buffer to test reception buffer manipulation
    // (reading)
    const bool copied = protocol.CopyTxBufferPayloadToRxBuffer();
    TEST_ASSERT_TRUE(copied);

    // Verifies that reading from the end of the payload functions as expected
    protocol.ReadData(test_value, TransportLayer<uint16_t, '<', '<'>::get_maximum_rx_payload_size() - 1);
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kObjectReadFromBuffer,
        protocol.transfer_status
    );

    // Verifies that attempting to read from an index beyond the payload range results in an error
    error_index =
        protocol.ReadData(test_value, TransportLayer<uint16_t, '<', '<'>::get_maximum_rx_payload_size());
    TEST_ASSERT_EQUAL_UINT16(0, error_index);
    TEST_ASSERT_EQUAL_UINT8(
        axtlmc_shared_assets::kTransportLayerCodes::kReadObjectBufferError,
        protocol.transfer_status
    );
}

// Tests major SendData() and ReceiveData() methods of the SerializedTransferProtocol class, alongside all used
// sub-methods (ParsePacket(), ValidatePacket(), ConstructPacket()) and auxiliary methods (Available()). Note, assumes
// lower level tests have already verified the functionality of StreamMock and buffer manipulation methods, which are
// also used here to facilitate testing.
void TestSerializedTransferProtocolDataTransmission()
{
    // Initializes the tested class
    StreamMock<254> mock_port;  // Minimal required size

    // Uses identical rx and tx payload sizes and tests maximal supported sizes for both buffers. Also uses a CRC-16
    // to test multibyte CRC handling.
    TransportLayer<uint16_t> protocol(mock_port, 0x1021, 0xFFFF, 0x0000, 129, 0, 20000, false);

    // Instantiates separate instances of encoder classes used to verify processing results
    COBSProcessor<> cobs_class;
    // CRC settings HAVE to be the same as used by the SerializedTransferProtocol instance.
    auto crc_class = CRCProcessor<uint16_t>(0x1021, 0xFFFF, 0x0000);

    // Generates the test array to be packaged and 'sent'
    const uint8_t test_array[10] = {1, 2, 3, 0, 0, 6, 0, 8, 0, 0};

    // Writes the package into the _transmission_buffer
    protocol.WriteData(test_array, 0);

    // Sends the payload to the Stream buffer. If all steps of this process succeed, the method returns 'true'.
    const bool sent_status = protocol.SendData();

    // Verifies that the data has been successfully sent to the Stream buffer
    TEST_ASSERT_TRUE(sent_status);
    TEST_ASSERT_EQUAL_UINT8(axtlmc_shared_assets::kTransportLayerCodes::kPacketSent, protocol.transfer_status);

    // Manually verifies the contents of the StreamMock class tx_buffer to confirm that the data has been
    // processed correctly:

    // Instantiates an array to simulate the _transmission_buffer after the data has been added to it.
    // Currently, the layout is: START, PAYLOAD_SIZE, OVERHEAD, PAYLOAD[10], DELIMITER, CRC[2]
    uint8_t buffer_array[16] = {129, 10, 0, 1, 2, 3, 0, 0, 6, 0, 8, 0, 0, 0, 0, 0};

    // Simulates COBS encoding the buffer. Note, assumes COBSProcessor methods have been tested before running this
    // test. Specifically, targets the 10-value payload starting from index 3. Uses the same delimiter byte value as
    // does the serial protocol class
    const uint16_t packet_size = cobs_class.EncodePayload(buffer_array, 0);

    // Calculates the CRC for the COBS-encoded buffer. Also assumes that the CRCProcessor methods have been tested
    // before running this test. The CRC calculation includes the overhead byte, the encoded payload and the inserted
    // delimiter byte. Note, the returned checksum depends on the used polynomial type.
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(buffer_array, 2, packet_size);

    // Adds the CRC to the end of the buffer. The insertion location has to be statically shifted to account for the
    // metadata preamble bytes
    crc_class.AddCRCChecksumToBuffer(buffer_array, packet_size + 2, crc_checksum);

    // Verifies that the packet inside the StreamMock tx_buffer is the same as the packet created via the manual steps
    // above.
    for (uint8_t i = 0; i < static_cast<uint8_t>(sizeof(buffer_array)); i++)
    {
        TEST_ASSERT_EQUAL_UINT8(buffer_array[i], static_cast<uint8_t>(mock_port.tx_buffer[i]));
    }

    // Copies the fully encoded package into the rx_buffer to simulate packet reception and test ReceiveData() method.
    // Note, adjusts the size to account for the fact mock class uses uint16 buffers
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(buffer_array) * 2);

    // Ensures that the overhead byte copied to the rx_buffer is not zero (that the packet is COBS-encoded). This check
    // has to be true for the decoding to work as expected and not throw a 'packet already decoded' error.
    TEST_ASSERT_NOT_EQUAL_UINT16(mock_port.rx_buffer[1], 0);

    // Receives the data stored in the StreamMock reception buffer. If all steps of this process succeed, the method
    // returns 'true'.
    const bool receive_status = protocol.ReceiveData();

    // Verifies that the data has been successfully received from the StreamMock rx buffer
    TEST_ASSERT_EQUAL_UINT8(axtlmc_shared_assets::kTransportLayerCodes::kPacketReceived, protocol.transfer_status);
    TEST_ASSERT_TRUE(receive_status);

    // Verifies that internal class _reception_buffer tracker was set to the expected payload size
    TEST_ASSERT_EQUAL_UINT16(10, protocol.get_rx_payload_size());

    // Verifies that the reverse-processed payload is the same as the original payload array. This is less involved than
    // the forward-conversion since there is no need to generate the CRC value or simulate COBS encoding here. This
    // assumes these methods have been fully tested before calling this test
    uint8_t decoded_array[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Placeholder-initialized
    protocol.ReadData(decoded_array, 0);                         // Reads the data from _transmission_buffer

    // Verifies that the decoded payload fully matches the test payload array contents
    TEST_ASSERT_EQUAL_UINT8_ARRAY(test_array, decoded_array, sizeof(test_array));

    // Verifies that the minor Available() method works as expected. This method returns 'true' if data to parse is
    // available and 'false' otherwise. Since StreamMock class initializes its buffers with zeroes, which is a valid
    // data value, this method should return 'true' even after fully consuming the test payload.
    bool data_available = protocol.Available();
    TEST_ASSERT_TRUE(data_available);

    // Verifies that ResetReceptionBuffer() method works as expected. This method resets the overhead and payload_size
    // variables of the buffer. Since the overhead is already reset by the decoder method, only the latter action is
    // evaluated below.
    protocol.ResetReceptionBuffer();
    TEST_ASSERT_EQUAL_UINT16(0, protocol.get_rx_payload_size());

    // Also verifies ResetTransmissionBuffer() method, which works the same as the ResetReceptionBuffer() method, but
    // specifically targets the _transmission_buffer
    protocol.ResetTransmissionBuffer();
    TEST_ASSERT_EQUAL_UINT16(0, protocol.get_rx_payload_size());

    // Fully resets the mock rx_buffer with -1, which is used as a stand-in for no available data. This is to test the
    // 'false' return portion of the Available() method.
    memset(mock_port.rx_buffer, -1, 254 * sizeof(mock_port.rx_buffer[0]));  // Converts from elements to bytes

    // Verifies that available() correctly returns 'false' if no data is actually available to be read from the
    // Stream class rx_buffer
    data_available = protocol.Available();
    TEST_ASSERT_FALSE(data_available);
}

// Tests the errors and, where applicable, edge cases associated with the SendData() and ReceiveData() methods of the
// SerializedTransferProtocol class. No auxiliary methods are tested here since they do not raise any errors. Note,
// focuses specifically on errors raised by SerializedTransferProtocol class methods, COBS and CRC errors should be
// tested by their respective test functions. Also, does not test errors that are generally impossible to encounter
// without modifying the class code, such as COBS encoding due to incorrect overhead placeholder value error.
// Note, to conserve used memory, uses CRC8 rather than CRC16. This should not affect the tested logic, but will reduce
// the memory size reserved by these functions
void TestSerializedTransferProtocolDataTransmissionErrors()
{
    // Initializes the tested class
    StreamMock<60> mock_port;  // Initializes to the minimal required size
    TransportLayer<uint16_t, 60, 60, 5> protocol(mock_port, 0x07, 0x00, 0x00, 129, 0, 20000, false);

    // Instantiates crc encoder class separately to generate test data
    auto crc_class = CRCProcessor<uint16_t>(0x07, 0x00, 0x00);

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Currently, it is effectively impossible to encounter an error during data sending as there are now compile-time
    // guards against every possible error engineered into the class itself or buffer manipulation methods. As such,
    // simply runs the method sequence here and moves on to testing reception, which can run into errors introduced
    // during transmission.
    protocol.WriteData(test_payload);
    protocol.SendData();

    // Verifies that the data has been 'sent' successfully
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketSent),
        protocol.transfer_status
    );

    // Instantiates the test buffer. The buffer is set to the state it is expected to be found after writing and COBS
    // encoding the data, but the CRC is calculated and added separately (see below).
    uint8_t test_buffer[15] = {129, 10, 5, 1, 2, 3, 4, 3, 6, 7, 3, 9, 10, 0, 0};

    // Calculates and adds packet CRC checksum to the postamble section of the test_buffer array
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, 12);
    crc_class.AddCRCChecksumToBuffer(test_buffer, 14, crc_checksum);

    // Writes the components to the mock class rx buffer to simulate data reception
    // Note, adjusts the size to account for the fact mock class uses uint16 buffers
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(test_buffer) * 2);

    // Verifies that the algorithm correctly handles missing start byte error. By default, the algorithm is configured
    // to treat these 'errors' as 'no bytes available for reading' status, which is a non-error status
    mock_port.rx_buffer[0] = 0;  // Removes the start byte
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer),
        protocol.transfer_status
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Changes the value of the allow_start_byte_errors boolean flag to allow raising start_byte-related errors.
    protocol.set_allow_start_byte_errors(true);

    // Verifies that when Start Bytes are enabled, the algorithm correctly returns the error code.
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketStartByteNotFound),
        protocol.transfer_status
    );
    mock_port.rx_buffer[0]    = 129;              // Restores the start byte
    mock_port.rx_buffer_index = 0;                // Resets readout index back to 0
    protocol.set_allow_start_byte_errors(false);  // Restores the flag to its default value

    // Verifies that when not enough bytes are available to parse (according to the minimum_expected_payload_size)
    // argument, the algorithm correctly aborts parsing with kNoBytesToParseFromBuffer code. Recall that for this test,
    // the class expects payloads of size 5 at a minimum.
    mock_port.rx_buffer[1] = -1;  // Essentially aborts reception at the payload_size byte value.
    const bool result      = protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kNoBytesToParseFromBuffer),
        protocol.transfer_status
    );
    TEST_ASSERT_FALSE(result);
    mock_port.rx_buffer[1] = static_cast<int16_t>(test_buffer[1]);

    // Verifies that the algorithm correctly handles a CRC checksum error (indicates corrupted packets).
    mock_port.rx_buffer[14] = 123;  // Fake CRC byte, overwrites the crc byte value found at the end of the packet
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kCRCCheckFailed),
        protocol.transfer_status
    );
    mock_port.rx_buffer[14]   = test_buffer[14];  // Restores the CRC byte value
    mock_port.rx_buffer_index = 0;                // Resets readout index back to 0

    // Verifies that the algorithm correctly handles missing payload_size byte errors. Due to reasons discussed above,
    // for this test to work, the buffer has to be modified to contain valid bytes before the start byte in a way that
    // makes the overall available() method result sufficiently range to trigger the parsing.
    // Starts by prepending 'filler' data to the buffer before the start_byte
    const uint16_t prepended_data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // All values are non-start-byte
    memcpy(mock_port.rx_buffer, prepended_data, sizeof(prepended_data));

    // Then, adds the main data to the buffer right after the prepended data. For this, uses 'reinterpret_cast' to shift
    // the target buffer pointer to the correct location.
    memcpy(
        reinterpret_cast<uint8_t*>(mock_port.rx_buffer) + sizeof(prepended_data),
        mock_port.tx_buffer,
        sizeof(test_buffer) * 2
    );

    // Note that from now on all indices are statically shifted by 10 to account for the prepended data
    mock_port.rx_buffer[11] = -1;  // Essentially aborts reception at the payload_size byte value.
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPayloadSizeByteNotFound),
        protocol.transfer_status
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Verifies that the algorithm correctly handles invalid payload_size byte errors. Tests payload_size byte
    // being too small (4) and too large (61). Note, these sizes depend on the template maximum_payload_size and
    // constructor minimum_expected_payload_size parameters.
    // Too small payload
    mock_port.rx_buffer[11] = 4;  // Too small payload value
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kInvalidPayloadSize),
        protocol.transfer_status
    );
    mock_port.rx_buffer_index = 0;  // Resets readout index back to 0

    // Too large payload
    mock_port.rx_buffer[11] = 61;  // Too large payload value
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kInvalidPayloadSize),
        protocol.transfer_status
    );
    mock_port.rx_buffer_index = 0;   // Resets readout index back to
    mock_port.rx_buffer[11]   = 10;  // Restores the payload_size byte value

    // Sets the entire rx_buffer to valid non-delimiter byte-values for the test below to work, as it has to consume
    // most of the rx_buffer to run out of the _reception_buffer space of the SerializedTransferProtocol class.
    for (uint16_t i = 15; i < StreamMock<60>::buffer_size; i++)
    {
        mock_port.rx_buffer[i] = 11;
    }

    // Verifies that the algorithm correctly handles encountering no valid bytes for a long time as a stale packet
    // error. For that, inserts an invalid value in the middle of the packet, which will be interpreted as not receiving
    // data until the timeout guard kicks-in to break the stale runtime.
    mock_port.rx_buffer[17] = -1;  // Sets byte 8 to an 'invalid' value to simulate not receiving valid bytes at index 7
    protocol.ReceiveData();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPacketTimeoutError),
        protocol.transfer_status
    );
    mock_port.rx_buffer[17]   = test_buffer[7];  // Restores the invalidated byte back to the original value
    mock_port.rx_buffer_index = 0;               // Resets readout index back to 0
}

void TestSerializedTransferProtocolDelimiterNotFoundError()
{
    // Initializes the tested class
    StreamMock<60> mock_port;
    TransportLayer<uint16_t, 60, 60, 5> protocol(mock_port, 0x07, 0x00, 0x00, 129, 0, 20000, false);
    CRCProcessor<uint16_t> crc_class(0x07, 0x00, 0x00);

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    protocol.WriteData(test_payload);
    protocol.SendData();

    // Instantiates the test buffer. Delimiter is changed.
    uint8_t test_buffer[15] = {129, 10, 5, 1, 2, 3, 4, 3, 6, 7, 3, 9, 10, 0, 0};

    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(test_buffer) * 2);
    mock_port.rx_buffer[13] = 1;  // Changes delimiter byte to non zero

    // Calculates and adds packet CRC checksum to the postamble section of the test_buffer to avoid CRC check error
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, 12);
    crc_class.AddCRCChecksumToBuffer(test_buffer, 14, crc_checksum);

    // Simulate receiving data
    protocol.ReceiveData();

    // Verifies that the delimiter was not found
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kDelimiterNotFoundError),
        protocol.transfer_status
    );
    mock_port.rx_buffer[14]   = test_buffer[14];
    mock_port.rx_buffer_index = 0;
}

void TestSerializedTransferProtocolDelimiterFoundTooEarlyError()
{
    // Initializes the tested class
    StreamMock<60> mock_port;
    TransportLayer<uint16_t, 60, 60, 5> protocol(mock_port, 0x07, 0x00, 0x00, 129, 0, 20000, false);
    CRCProcessor<uint16_t> crc_class(0x07, 0x00, 0x00);

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    protocol.WriteData(test_payload);
    protocol.SendData();

    // Instantiates the test buffer. Delimiter is changed.
    uint8_t test_buffer[15] = {129, 10, 5, 1, 2, 3, 4, 3, 6, 7, 3, 9, 10, 0, 0};

    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(test_buffer) * 2);
    mock_port.rx_buffer[7] = 0;  // Add delimiter value too early

    // Calculates and adds packet CRC checksum to the postamble section of the test_buffer to avoid CRC check error
    const uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, 12);
    crc_class.AddCRCChecksumToBuffer(test_buffer, 14, crc_checksum);

    // Simulate receiving data
    protocol.ReceiveData();

    // Verifies that the delimiter was found too early
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kDelimiterFoundTooEarlyError),
        protocol.transfer_status
    );
    mock_port.rx_buffer[7]    = test_buffer[7];
    mock_port.rx_buffer_index = 0;
}

void TestSerializedTransferProtocolPostambleTimeoutError()
{
    // Initializes the tested class
    StreamMock<60> mock_port;
    TransportLayer<uint16_t, 60, 60, 5> protocol(mock_port, 0x07, 0x00, 0x00, 129, 0, 20000, false);
    CRCProcessor<uint16_t> crc_class(0x07, 0x00, 0x00);

    // Initializes a test payload
    const uint8_t test_payload[10] = {1, 2, 3, 4, 0, 0, 7, 8, 9, 10};

    // Currently, it is effectively impossible to encounter an error during data sending as there are now compile-time
    // guards against every possible error engineered into the class itself or buffer manipulation methods. As such,
    // simply runs the method sequence here and moves on to testing reception, which can run into errors introduced
    // during transmission.
    protocol.WriteData(test_payload);
    protocol.SendData();

    /// Initializes the test buffer, omitting the postamble to simulate the timeout
    uint8_t test_buffer[15] =
        {129, 10, 5, 1, 2, 3, 4, 3, 6, 7, 3, 9, 10, 0, 0};  // Postamble should be here but is missing

    // Writes the components to the mock class rx buffer to simulate data reception
    // Note, adjusts the size to account for the fact mock class uses uint16 buffers
    memcpy(mock_port.rx_buffer, mock_port.tx_buffer, sizeof(test_buffer) * 2);
    mock_port.rx_buffer[14] = -1;  // Sets postamble byte 8 to an 'invalid' value

    // Calculates and adds packet CRC checksum to the postamble section of the test_buffer to avoid CRC check error
    uint16_t crc_checksum = crc_class.CalculatePacketCRCChecksum(test_buffer, 2, 12);
    crc_class.AddCRCChecksumToBuffer(test_buffer, 14, crc_checksum);

    // Simulate receiving data
    protocol.ReceiveData();

    // Simulate timeout for postamble not being received
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(axtlmc_shared_assets::kTransportLayerCodes::kPostambleTimeoutError),
        protocol.transfer_status
    );
    mock_port.rx_buffer[14]   = test_buffer[14];
    mock_port.rx_buffer_index = 0;
}

// Specifies the test functions to be executed and controls their runtime. Use this function to determine which tests
// are run during test runtime and in what order. Note, each test function is encapsulated and will run even if it
// depends on other test functions ran before it which fail the tests.
int RunUnityTests()
{
    UNITY_BEGIN();

    // COBS Processor
    RUN_TEST(TestCOBSProcessor);
    RUN_TEST(TestCOBSProcessorErrors);

    // CRC Processor
    RUN_TEST(TestCRCProcessorGenerateTable_CRC8);
    RUN_TEST(TestCRCProcessorGenerateTable_CRC16);

// This test requires at least 2048 bytes of RAM to work, so prevents it from being evaluated by boards like Arduino
// Uno. Specifically, disables the test for selected architectures known to not have sufficient memory
#if !defined(__AVR_ATmega328P__) && !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega2560__) && \
    !defined(__AVR_ATtiny85__) && !defined(__AVR_ATmega168__) && !defined(__AVR_ATmega1280__) &&    \
    !defined(__AVR_ATmega8__) && !defined(__AVR_ATmega16U4__) && !defined(__AVR_ATmega32U4__) &&    \
    !defined(__SAMD21G18A__)
    RUN_TEST(TestCRCProcessorGenerateTable_CRC32);
#endif

    RUN_TEST(TestCRCProcessor);
    RUN_TEST(TestCRCProcessorErrors);

    // Stream Mock
    RUN_TEST(TestStreamMock);

    // Serial Transfer Protocol Write / Read Data
    RUN_TEST(TestSerializedTransferProtocolBufferManipulation);
    RUN_TEST(TestSerializedTransferProtocolBufferManipulationErrors);

    // Serial Transfer Protocol Send / Receive Data
    RUN_TEST(TestSerializedTransferProtocolDataTransmission);
    RUN_TEST(TestSerializedTransferProtocolDataTransmissionErrors);
    RUN_TEST(TestSerializedTransferProtocolDelimiterNotFoundError);
    RUN_TEST(TestSerializedTransferProtocolPostambleTimeoutError);
    RUN_TEST(TestSerializedTransferProtocolDelimiterFoundTooEarlyError);

    return UNITY_END();
}

// Defines the baud rates for different boards. Note, this list is far from complete and was designed for the boards
// the author happened to have at hand at the time of writing these tests. When testing on an architecture not
// explicitly covered below, it may be beneficial to provide the baudrate optimized for the tested platform.

// For Arduino Due, the maximum non-doubled stable rate is 5.25 Mbps at 84 MHz cpu clock.
#if defined(ARDUINO_SAM_DUE)
#define SERIAL_BAUD_RATE 5250000

// For Uno, Mega, and other 16 MHz AVR boards, the maximum stable non-doubled rate is 1 Mbps.
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA) ||  \
    defined(__AVR_ATmega328P__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega2560__) || \
    defined(__AVR_ATmega168__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega16U4__) ||  \
    defined(__AVR_ATmega32U4__)
#define SERIAL_BAUD_RATE 1000000

// For all other boards the default 9600 rate is used. Note, this is a very slow rate, and it is very likely your board
// supports a faster rate. Be sure to select the most appropriate rate based on the CPU clock of your board, as it
// directly affects the error rate at any given baudrate. Boards like Teensy and Dues using USB port ignore the baudrate
// setting and instead default to the fastest speed support by the particular USB port pair (480 Mbps for most modern
// ports).
#else
#define SERIAL_BAUD_RATE 9600
#endif

// This is necessary for the Arduino framework testing to work as expected, which includes teensy. All tests are
// run inside setup function as they are intended to be one-shot tests
void setup()
{
    // Starts the serial connection. Uses the SERIAL_BAUD_RATE defined above based on the specific board architecture
    // (or the generic safe 9600 baudrate, which is VERY slow and should not really be used in production code).
    Serial.begin(SERIAL_BAUD_RATE);

    // Waits ~2 seconds before the Unity test runner establishes connection with a board Serial interface. For teensy,
    // this is less important, since it uses a USB interface which does not reset the board on connection.
    delay(2000);

    // Runs the required tests
    RunUnityTests();

    // Stops the serial communication interface.
    Serial.end();
}

// Nothing here as all tests are done in a one-shot fashion using 'setup' function above
void loop()
{}
