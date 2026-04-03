/**
 * @file
 * @brief Demonstrates bidirectional serial communication using the TransportLayer class.
 *
 * This file exactly matches the rx_tx_loop.cpp example and is excluded from the compiled library. It is kept here
 * to facilitate library development. This example is intended to be used together with the quickstart loop for the
 * companion library: https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-pc#quickstart.
 *
 * @note See https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc for more details.
 * API documentation: https://ataraxis-transport-layer-mc-api-docs.netlify.app/.
 * Authors: Ivan Kondratyev (Inkaros), Jasmine Si.
 */

#include <Arduino.h>
#include <transport_layer.h>

/// Manages bidirectional serial communication with the connected PC.
TransportLayer<> transport_layer(Serial);  // NOLINT(*-interfaces-global-init)

/// Stores the scalar value used to verify numeric serialization.
uint32_t test_scalar = 0;

/// Stores the array used to verify sequential byte serialization.
uint8_t test_array[4] = {0, 0, 0, 0};

/// Stores the test data used to verify struct serialization.
struct TestStruct
{
        bool test_flag   = true;   ///< Determines whether the test flag is set.
        float test_float = 6.66;   ///< Stores the floating-point value used for serialization testing.
} PACKED_STRUCT test_struct;

/// Initializes the serial communication interface.
void setup()
{
    Serial.begin(115200);
}

/// Receives, processes, and re-transmits serialized data to demonstrate the TransportLayer API.
void loop()
{
    if (!transport_layer.Available()) return;

    // Reads the received byte-stream, parses the payload, and makes it available for reading.
    const bool data_received = transport_layer.ReceiveData();
    if (!data_received) return;

    // Overwrites the memory of the placeholder objects with the received data. The data must be read in the same
    // order as it was written to the payload.
    transport_layer.ReadData(test_scalar);
    transport_layer.ReadData(test_array);
    transport_layer.ReadData(test_struct);

    // Re-transmits the same data in the same order except for the test_scalar which is changed to a new value.
    test_scalar = 987654321;
    transport_layer.WriteData(test_scalar);
    transport_layer.WriteData(test_array);
    transport_layer.WriteData(test_struct);
    transport_layer.SendData();
}
