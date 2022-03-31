/**
 * @file   main.cpp
 * @author ZiTe (honmonoh@gmail.com) & Dmitry
 * @brief  Example for Arduino MCP2515 CAN interface library.
 *         Reference: https://github.com/autowp/arduino-mcp2515
 */

// #define TX
#define RX

#include <Arduino.h>
#include "mcp2515.h"

#if defined TX

struct can_frame canMsg1;
struct can_frame canMsg2;
MCP2515 mcp2515(10);

void setup()
{
  canMsg1.can_id = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x01;
  canMsg1.data[1] = 0x02;
  canMsg1.data[2] = 0x03;
  canMsg1.data[3] = 0x04;
  canMsg1.data[4] = 0x05;
  canMsg1.data[5] = 0x06;
  canMsg1.data[6] = 0x07;
  canMsg1.data[7] = 0x08;

  canMsg2.can_id = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x80;
  canMsg2.data[1] = 0x70;
  canMsg2.data[2] = 0x60;
  canMsg2.data[3] = 0x50;
  canMsg2.data[4] = 0x40;
  canMsg2.data[5] = 0x30;
  canMsg2.data[6] = 0x20;
  canMsg2.data[7] = 0x10;

  while (!Serial)
    ;
  Serial.begin(9600);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  Serial.println("Example: Write to CAN");
}

void loop()
{
  mcp2515.sendMessage(&canMsg1);
  mcp2515.sendMessage(&canMsg2);

  Serial.println("Messages sent");

  delay(100);
}

#elif defined RX

struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup()
{
  Serial.begin(9600);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop()
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" ");
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");

    for (int i = 0; i < canMsg.can_dlc; i++)
    { // print the data
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }
}

#endif