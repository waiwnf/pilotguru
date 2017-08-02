// demo: CAN-BUS Shield, send data
#include "mcp_can.h"
// #include <SPI.h>

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

void setup()
{
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN init fail");
        //Serial.println(" Init CAN BUS Shield again");
        delay(1000);
    }
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("CAN init ok!");
}

byte in_message[MAX_CHAR_IN_MESSAGE];
unsigned long in_id = 0;
char serial_out_message[MAX_CHAR_IN_MESSAGE * 2 + 2 + 8];
unsigned char len = 0;

void loop()
{
  if(CAN.readMsgBufID(&in_id, &len, in_message) == CAN_OK) {
    sprintf(serial_out_message, "%lX ", in_id);
    size_t id_string_length = strlen(serial_out_message);
    for (byte i = 0; i < len ; ++i) {
      sprintf(serial_out_message + id_string_length + (i * 2), "%02X", in_message[i]);
    }
    Serial.println(serial_out_message);
  }
}
