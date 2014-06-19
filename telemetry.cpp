#include <Arduino.h>

#define START_STOP 0x7e
#define BYTESTUFF  0x7d
#define STUFFMASK  0x20

#define MAX_FRAME_SIZE 19

uint8_t frskyRxBuffer[MAX_FRAME_SIZE];   // Receive buffer. 9 bytes (full packet), worst case 18 bytes with byte-stuffing (+1)
uint8_t frskyTxBuffer[MAX_FRAME_SIZE];   // Ditto for transmit buffer

void frskyPushValue(uint8_t *&ptr, uint8_t value) {
  // byte stuff the only byte than might need it
  bool bytestuff = false;

  if (value == START_STOP) {
    bytestuff = true;
    value = 0x5e;
  }
  else if (value == BYTESTUFF) {
    bytestuff = true;
    value = 0x5d;
  }

  *ptr++ = value;
  if (bytestuff)
    *ptr = BYTESTUFF;
}

void frskySendPacket(uint8_t vbat) {
  uint8_t *ptr = &frskyTxBuffer[0];

  *ptr++ = START_STOP;        // End of packet
  *ptr++ = 0x00;
  *ptr++ = 0x00;
  *ptr++ = 0x00;
  *ptr++ = 0x00;
  *ptr++ = 0x00;
  *ptr++ = 0xdd;
  *ptr++ = 0xcc;
  frskyPushValue(ptr, vbat);
  *ptr++ = 0xfe;
  *ptr++ = START_STOP; // Start of packet

  int frskyTxBufferCount = ptr - &frskyTxBuffer[0];

  while (frskyTxBufferCount--) {
    Serial.write(*ptr--);
  }
  Serial.flush();
}

