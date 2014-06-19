#include "Arduino.h"
#include "nrf24.h"
#include <util/delay.h>
#include <SPI.h>

//#define DEBUG 1

int crazyflie_channel;

void nrf24_begin() {
  int data[5];

  // initialize NRF24L01

  pinMode(NRF24L01_SS, OUTPUT);  // SS (CSN)
  pinMode(NRF24L01_CE, OUTPUT);  // CE

  Serial.println("activating SPI...");
  SPI.begin();
  Serial.println(" done.");

  nrf24_write_ce(LOW);
  nrf24_write_cs(HIGH);

  delay(15);

  Serial.println("activating radio...");

  // See if we can read a register.
  nrf24_read_register(0x0a, data, 5);

  // Start radio in PTX mode.  Clear down interrupts.
  nrf24_write_8b_register(NRF24_CONFIG, NRF24_CONFIG_MASK_RX_DR |
                                                NRF24_CONFIG_MASK_TX_DS |
                                                NRF24_CONFIG_MASK_MAX_RT |
                                                NRF24_CONFIG_EN_CRC |
                                                NRF24_CONFIG_CRCO |
                                                NRF24_CONFIG_PWR_UP);

  // Wait for radio up.
  Serial.println("Sleeping a second...");
  delay(1000);

  // Enable dynamic packet size and ACK payload features.
  nrf24_write_8b_register(NRF24_FEATURE, 0x06);
  nrf24_write_8b_register(NRF24_DYNPD, 0x01);
}


void nrf24_write_cs(int state) {
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  if (state)
    PORTB |= _BV(1);
  else
    PORTB &= ~(_BV(1));
}

void nrf24_write_ce(int state) {
  if (state)
    //PORTB |= _BV(2);
    PORTC |= _BV(5);
  else
    //PORTB &= ~(_BV(2));
    PORTC &= ~(_BV(5));
}


void nrf24_write_register(int reg, const int *addr, size_t len) {
  nrf24_write_cs(LOW);
  SPI.transfer(NRF24_CMD_W_REGISTER | (reg & 0x1f));
  while ( len-- ) {
#if DEBUG
    Serial.print("SPI: writing 0x");
    Serial.println(*addr, HEX);
#endif
    SPI.transfer(*addr++);
  }
  nrf24_write_cs(HIGH);
}


void nrf24_read_register(int reg, int *addr, size_t len) {
  int debug_addr;
  nrf24_write_cs(LOW);
  SPI.transfer(NRF24_CMD_R_REGISTER | (reg & 0x1f));
  while ( len-- ) {
    debug_addr = SPI.transfer(0xff);
#if DEBUG
    Serial.print("SPI: reading 0x");
    Serial.println(debug_addr, HEX);
#endif
    *addr++ = debug_addr;
  }
  nrf24_write_cs(HIGH);
}


void nrf24_write_8b_register(int reg, int value) {
#if DEBUG
  Serial.print("SPI: writing 0x");
  Serial.print(value, HEX);
  Serial.print(" to register 0x");
  Serial.println(NRF24_CMD_W_REGISTER | reg, HEX);
#endif
  nrf24_write_cs(LOW);
  SPI.transfer(NRF24_CMD_W_REGISTER | (reg & 0x1f));
  SPI.transfer(value);
  nrf24_write_cs(HIGH);
}


int nrf24_cmd(unsigned char cmd) {
  uint8_t status;
  nrf24_write_cs(LOW);
#if DEBUG
  Serial.print("Sending cmd 0x");
  Serial.println(cmd, HEX);
#endif
  status = SPI.transfer(cmd);
#if DEBUG
  Serial.print("Received answer 0x");
  Serial.println(status, HEX);
#endif
  nrf24_write_cs(HIGH);

  return status;
}

int nrf24_nop(NRF24_RADIO_t *self) {
  return nrf24_cmd(NRF24_CMD_NOP);
}

int nrf24_flush_tx(NRF24_RADIO_t *self) {
  return nrf24_cmd(NRF24_CMD_FLUSH_TX);
}

int nrf24_flush_rx(NRF24_RADIO_t *self) {
  return nrf24_cmd(NRF24_CMD_FLUSH_RX);
}

int nrf24_set_channel(NRF24_RADIO_t *self, int channel) {
  // Validate channel.
  if (channel < 0 || 125 < channel)
    return CTL_PARAMETER_ERROR;
   
  // Change the channel.
  nrf24_write_8b_register(NRF24_RF_CH, channel);

  // All done.
  return CTL_NO_ERROR;
}

void nrf24_recompute_retr(NRF24_RADIO_t *self) {
  int ard = 0;

  // Retry timeouts depend upon link speed and payload length.
  switch (self->setup & NRF24_RF_SETUP_RF_DR_MASK)
    {
    case NRF24_RF_SETUP_RF_DR_2Mbps:
      if (self->ack_payload_length <= 15)      ard = 0;  // 250 us
      else                                     ard = 1;  // 500 us
      break;

    case NRF24_RF_SETUP_RF_DR_1Mbps:
      if (self->ack_payload_length <= 5)       ard = 0;  // 250 us
      else                                     ard = 1;  // 500 us
      break;

    case NRF24_RF_SETUP_RF_DR_250kbps:  // table 18
      if      (self->ack_payload_length <=  0) ard = 1;  //  500 us
      else if (self->ack_payload_length <=  8) ard = 2;  //  750 us
      else if (self->ack_payload_length <= 16) ard = 3;  // 1000 us
      else if (self->ack_payload_length <= 24) ard = 4;  // 1250 us
      else                                     ard = 5;  // 1500 us

    default:
      break;
    }

  // Integrate ARD into shadow register.
  self->retr &= ~(0xf << 4);
  self->retr |=  (ard << 4);

  // Synchronize hardware and shadow registers.
  nrf24_write_8b_register(NRF24_SETUP_RETR, self->retr);
}

int nrf24_set_data_rate(NRF24_RADIO_t *self, int kHz) {
  // Clear data rate bits in shadow register.
  self->setup &= ~NRF24_RF_SETUP_RF_DR_MASK;
  Serial.print("Reset datarate to ");
  Serial.println(self->setup);

  // nRF24L01 does not support 250 kHz low data rate...
  if (kHz <= 250 && self->variant == NRF24L01P)
    {
      self->setup |= NRF24_RF_SETUP_RF_DR_250kbps;\
    }
  else if (kHz <= 1000) 
    {
      self->setup |= NRF24_RF_SETUP_RF_DR_1Mbps;
    }
  else
    {
      self->setup |= NRF24_RF_SETUP_RF_DR_2Mbps;
    }

  Serial.print("Set datarate to ");
  Serial.println(self->setup);
  // Synchronize hardware and shadow registers.
  nrf24_write_8b_register(NRF24_RF_SETUP, self->setup);

  // Synchronize RETR.
  nrf24_recompute_retr(self);

  // All done.
  return CTL_NO_ERROR;
}

int nrf24_set_auto_retransmit_count(NRF24_RADIO_t *self, int count) {
  // Clamp.
  if      (count <  0) count = 0;
  else if (count > 15) count = 15;

  // Update.
  self->retr &= ~0xf;
  self->retr |= count;

  // Synchronize hardware and shadow registers.
  nrf24_write_8b_register(NRF24_SETUP_RETR, self->retr);

  // All done.
  return CTL_NO_ERROR;
}

int nrf24_set_transmit_power(NRF24_RADIO_t *self, int dBm) {
  // Clear power bits in shadow register.
  self->setup &= ~(3 * NRF24_RF_SETUP_RF_PWR);

  // Encode power in shadow register.
  if      (dBm <= -18) { self->setup |= 0*NRF24_RF_SETUP_RF_PWR; }
  else if (dBm <= -12) { self->setup |= 1*NRF24_RF_SETUP_RF_PWR; }
  else if (dBm <= -6)  { self->setup |= 2*NRF24_RF_SETUP_RF_PWR; }
  else                 { self->setup |= 3*NRF24_RF_SETUP_RF_PWR; }

  // Synchronize hardware and shadow registers.
  nrf24_write_8b_register(NRF24_RF_SETUP, self->setup);

  // All done.
  return CTL_NO_ERROR;
}

void nrf24_set_ack_payload_length(NRF24_RADIO_t *self, int bytes) {
  // Update ack payload length.
  self->ack_payload_length = bytes;

  // Update ARD based on payload length and data rate.
  nrf24_recompute_retr(self);
}

static void nrf24_set_address(const int *address) {
  int i;

  nrf24_write_register(NRF24_TX_ADDR, address, 5);
  for (i = 0; i < 5; ++i)
    nrf24_write_8b_register(NRF24_RX_ADDR_P0+i, address[i]);
}

void nrf24_pulse_ce() {
  nrf24_write_ce(HIGH);
  _delay_us(20);  // From datasheet, must be > 10us
  nrf24_write_ce(LOW);
}

void nrf24_transmit_packet(const uint8_t *payload, size_t payload_size) {
  // Write payload to the buffer.
  nrf24_write_cs(LOW);
  SPI.transfer(NRF24_CMD_TX_PAYLOAD);
  while ( payload_size-- )
    SPI.transfer(*payload++);
  nrf24_write_cs(HIGH);

  
  // Pulse CE to start transmit.
  nrf24_pulse_ce();
}

int nrf24_read_receive_packet(NRF24_RADIO_t *self, unsigned char *payload) {
  int stat;

  // Get the packet length.
    nrf24_write_cs(LOW);

  SPI.transfer(NRF24_CMD_R_RX_PL_WID);
  stat = SPI.transfer(0xff);
  nrf24_write_cs(HIGH);

  byte len = stat;
  
  // Validate response.
  if (0 < len && len <= 32) {

    nrf24_write_cs(LOW);
    SPI.transfer(NRF24_CMD_RX_PAYLOAD);
    while ( len-- )
      *payload++ = SPI.transfer(0xff);
    nrf24_write_cs(HIGH);

    return stat;

  } else {
    return 0;
  }
}

int nrf24_wait_for_interrupt_and_acknowledge(NRF24_RADIO_t *self) {
  unsigned int interrupts;
  int stat;

  // Interrupts to wait for.
  interrupts = NRF24_STATUS_RX_DR |
               NRF24_STATUS_TX_DS |
               NRF24_STATUS_MAX_RT;

  // FIXME: add timeout


  for (;;)
    {
#if DEBUG
      Serial.print("waiting for interrupt...\r\nstat=0x");
      Serial.println(stat);
#endif
      if ((stat = nrf24_nop(self)) & interrupts) {
        _delay_ms(10);
#if DEBUG
        Serial.print("Interrupt received: 0x");
        Serial.println(stat, HEX);
        Serial.print("Result: 0x");
        Serial.println((stat = nrf24_nop(self)) & interrupts, HEX);
#endif
        break;
      }
    }

  // Acknowledge interrupt.
#if DEBUG
  Serial.println("Acknowledging interrupt");
#endif
  nrf24_write_8b_register(NRF24_STATUS, interrupts & stat);

  // Done.
  return stat;
}

int nrf24_send_packet(NRF24_RADIO_t *self, const void *payload, size_t payload_size, void *ack, size_t *ack_size) {
  int8_t stat;

  // Transmit packet.
  nrf24_transmit_packet((uint8_t*)payload, payload_size);

  // Wait for interrupt.
  stat = nrf24_wait_for_interrupt_and_acknowledge(self);
  
  // Did we manage to send this packet?  If we took the maximum number of
  // transmits, just drop it.
  if (stat & NRF24_STATUS_MAX_RT) {
    nrf24_flush_tx(self);
    return CTL_DEVICE_NOT_RESPONDING;
  }

  // Did we receive an acknowledgment?  If so, accept it.
  if (stat & NRF24_STATUS_RX_DR)
    *ack_size = nrf24_read_receive_packet(self, (unsigned char *)ack);
  else 
    *ack_size = 0;

  // Done with the acknowledge.  
  nrf24_flush_rx(self);

  // Return "data sent" status.
  return stat & NRF24_STATUS_TX_DS;
}


void channel_scan(NRF24_RADIO_t *self) {
  int separation, channel;
  int stat;
  unsigned char ack[33];
  size_t ack_size;
  CRAZYFLIE_COMMANDER_SETPOINT_t setpoint;

  Serial.println("setting up scan...");
  // Level, no thrust.
  setpoint.c = 3 << 4;
  setpoint.roll = 0;
  setpoint.pitch = 0;
  setpoint.yaw = 0;
  setpoint.thrust = 0;

  // Channel separation is different for 2Mbps link.
  separation = 1;
  if ((self->setup & NRF24_RF_SETUP_RF_DR_MASK) == NRF24_RF_SETUP_RF_DR_2Mbps)
    separation = 2;

  Serial.print("Separation: ");
  Serial.println(separation);

  // When scanning, don't use lots of retransmits.
  nrf24_set_auto_retransmit_count(self, 5);

  Serial.println(" done.\r\nStarting scan...\r\n");
  // Scan channels...
  for (channel = 0; channel <= 125; channel += separation) {
    Serial.print("Setting channel ");
    Serial.println(channel);
    nrf24_set_channel(self, channel);
    Serial.println(" done.\r\nSending packet...");
    stat = nrf24_send_packet(self, &setpoint, sizeof(setpoint), ack, &ack_size);
    Serial.println(" done.");

    if (stat > 0) {
      Serial.print("stat=");
      Serial.println(stat);
      Serial.print("\r\nResponse on channel ");
      Serial.println(channel);
      crazyflie_channel = channel;
      break;
    }
  }
}

void cf_log_setup(NRF24_RADIO_t *self) {
  CRAZYFLIE_CONSOLE_t ack;
  unsigned int ack_size;
  
  // send a packet on logging port, cmd channel (port 5, ch 1) and create a log block (0x00) with ID 0xbb for var pm.vbat (0x00) with type float32 (0x07)
  uint8_t payload[] = { 0x51, 0x00, 0xbb, 0x07, 0x00 };
  nrf24_send_packet(self, &payload, sizeof(payload), &ack, &ack_size);

  _delay_ms(10);

  payload[1] = 0x03; // start log block
  payload[2] = 0xbb; // log block id 0xbb
  payload[3] = 0x05; // 50ms
  nrf24_send_packet(self, &payload, 4, &ack, &ack_size);

}

#define PARAM_PORT 0x02
#define TOC_CH 0x00
#define CMD_PARAM_TOC_INFO 0x01
#define CMD_PARAM_TOC_GET_ITEM 0x00

uint8_t cf_find_hover_param(NRF24_RADIO_t *self) {
  CRAZYFLIE_CONSOLE_t ack;
  unsigned int ack_size;
  const char group[] = "flightmode";
  const char name[] = "althold";
  uint8_t payload[3];
  payload = { (PARAM_PORT << 4 | TOC_CH), 0x00 };
  uint8_t num_params = 0;
  uint8_t stat;
  int param_id = 0;
  bool match = 1;

  Serial.print("getting param id for althold: 0x");
  // get the number of parameters in TOC
  stat = nrf24_send_packet(self, &payload, 2, &ack, &ack_size);
  Serial.println(ack_size, HEX);
  payload[1] = CMD_PARAM_TOC_INFO;
  while (num_params <= 0) {
    stat = nrf24_send_packet(self, &payload, 2, &ack, &ack_size);
    Serial.print("ACK Size: 0x");
    Serial.println(ack_size, HEX);

    Serial.print("Packet Header: 0x");
    Serial.println(ack.c, HEX);
    Serial.print("Packet Payload[0]: 0x");
    Serial.println(ack.console.data[0], HEX);
    Serial.print("Packet Payload[1]: 0x");
    Serial.println(ack.console.data[1], HEX);
    if ((stat >= 0) && (ack_size > 0) && (ack.c == (PARAM_PORT << 4 | TOC_CH)) && (ack.console.data[0] == CMD_PARAM_TOC_INFO)) {
      num_params = ack.console.data[1];
      Serial.print("num_params: ");
      Serial.println(num_params, DEC);
    }
  }

  for (uint8_t i = 0; i < num_params; i++) {

    payload[1] = CMD_PARAM_TOC_GET_ITEM;
    payload[2] = i;

    
    while (1) {
      stat = nrf24_send_packet(self, &payload, 3, &ack, &ack_size);
      if ((stat >= 0) && (ack_size > 0) && (ack.c == (PARAM_PORT << 4 | TOC_CH)) && (ack.console.data[0] == CMD_PARAM_TOC_GET_ITEM) && (ack.console.data[1] == i)) {
        break;
      }
    }

    unsigned char * b = &(ack.console.data[3]);
    Serial.print(ack.console.data[1]);
    Serial.print(": ");
    int j = 0;
    match = 1;
    while (*b != '\0') {
      Serial.write(*b);
      if (*b != group[j]) {
        match = 0;
      }
      *b++;
      j++;
    }
    *b++;
    j=0;
    Serial.print(".");
    while (*b != '\0') {
      Serial.write(*b);
      if ((match == 0) || (*b != name[j])) {
        match = 0;
      }
      *b++;
      j++;
    }
    Serial.print(" - match: ");
    Serial.println(match);

    if (match) {
      Serial.println("Found althold!");
      param_id = ack.console.data[1];
      break;
    }
  }
  return param_id;
}

void cf_hovermode(NRF24_RADIO_t *self, bool hover) {
  CRAZYFLIE_CONSOLE_t ack;
  unsigned int ack_size;
  static bool last_hover;
  
  if  (hover != last_hover) {
    // send param to change param flightmode.althold
    uint8_t payload[] = { 0x22, hover_id, hover };
    nrf24_send_packet(self, &payload, sizeof(payload), &ack, &ack_size);
  }
  last_hover = hover;
}

