/*

   ESP32 Arduino Library for:
	
   MCP3008 Chip 

   Provides 8 additional Analog inputs, 10 bits each 

   Uses Hardware SPI Interface

   05/2019 Bill Row

*/

#include <mcp3008.h>

mcp3008 mcp = mcp3008();

const uint8_t _SS   = 25;   // Pins for SPI interface
const uint8_t _MISO = 12;
const uint8_t _MOSI = 13;   // example shows HSPI defaults
const uint8_t _SCLK = 14;   


void setup() {

  Serial.begin(115200);

  mcp.begin(_SCLK, _MISO, _MOSI, _SS);      
     
}

void loop() {

  for (int pin_ = 0; pin_ < 8; pin_++) {

    uint16_t value = mcp.analogRead(pin_);

    Serial.println(value);

  }

  Serial.println("-------------");

  delay(200);

}
