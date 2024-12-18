// This example is intended to be used together with the rx_tx_loop example of the Python companion library
// ataraxis-transport-layer-pc. When used correctly, the example code will continuously transmit and receive data
// between the microcontroller and the PC.
//
// See https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc for more details.
// API documentation: https://ataraxis-transport-layer-mc-api-docs.netlify.app/.
// Authors: Ivan Kondratyev (Inkaros), Jasmine Si.

// Includes the core dependency for all Teensyduino projects.
#include <Arduino.h>

// Includes the TransportLayer header to access class API.
#include <transport_layer.h>

// Instantiates a new TransportLayer object. Most template and constructor arguments should automatically scale with
// your microcontroller. Check the API documentation website if you want to fine-tune class parameters to better match
// your use case.
TransportLayer<> tl_class(Serial);  // NOLINT(*-interfaces-global-init)

void setup()
{
    Serial.begin(115200);  // Opens the Serial port, initiating PC communication
}

// Pre-creates the objects used for the demonstration below.
uint32_t test_scalar  = 0;
uint8_t test_array[4] = {0, 0, 0, 0};

struct TestStruct
{
        bool test_flag   = true;
        float test_float = 6.66;
} __attribute__((packed)) test_struct;  // Critically, the structure HAS to be packed!

void loop()
{
    // Checks if data is available for reception.
    if (tl_class.Available())
    {
        // If the data is available, carries out the reception procedure (reads the received byte-stream, parses the
        // payload, and makes it available for reading).
        const bool data_received = tl_class.ReceiveData();

        // If the reception was successful, reads the data, assumed to contain serialized test objects. Note, this
        // example is intended to be used together with the example script from the ataraxis-transport-layer-pc library.
        if (data_received)
        {
            // Overwrites the memory of placeholder objects with the received data.
            uint16_t next_index = 0;  // Starts reading from the beginning of the payload region.
            next_index          = tl_class.ReadData(test_scalar, next_index);
            next_index          = tl_class.ReadData(test_array, next_index);
            // Since test_struct is the last object in the payload, we do not need to save the new next_index.
            tl_class.ReadData(test_struct, next_index);

            // Now the placeholder objects are updated with the values transmitted from the PC. The section below
            // showcases sending the data to the PC. It re-transmits the same data in the same order except
            // for the test_scalar which is changed to a new value.
            test_scalar = 987654321;

            // Writes objects to the TransportLayer's transmission buffer, staging them to be sent with the next
            // SendData() command. Note, the objects are written in the order they will be read by the PC.
            next_index = 0;  // Resets the index to 0.
            next_index = tl_class.WriteData(test_scalar, next_index);
            next_index = tl_class.WriteData(test_array, next_index);
            tl_class.WriteData(test_struct, next_index);  // Once again, the index after last object is not saved.

            // Packages and sends the contents of the transmission buffer that were written above to the PC.
            tl_class.SendData();  // This also returns a boolean status that we discard for this example.
        }
    }
}