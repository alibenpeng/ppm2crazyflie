
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h> /* for sei() */
#include <util/delay.h> /* for _delay_ms() */
#include <SPI.h>

#include "nrf24.h"
#include "ppm.h"

static byte reportBuffer[8];



/* ------------------------------------------------------------------------- */

void setup() {
  int i;
  NRF24_RADIO_t radio;

  pinMode(8, INPUT); // ppm input
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(57600);
  Serial.println("\r\n\r\nAli's PPM to Crazyflie module. Wooohoo1!\r\n");
  Serial.println(F_CPU);

  // Turn link LED off.
  digitalWrite(LED_PIN, 0);


  // Initialize SPI device associated with NRF24L01 and attach it to the bus.
  memset(&radio, 0, sizeof(radio));
  radio.variant = NRF24L01P;

  nrf24_begin();

  // Configure fast data rate, high power, 32-byte ACK payload.
  nrf24_set_data_rate(&radio, 1000);
  nrf24_set_transmit_power(&radio, 0);
  nrf24_set_ack_payload_length(&radio, 32);
  nrf24_set_auto_retransmit_count(&radio, 10);

  // Zap any outstanding packets.
  for (i = 0; i < 10; ++i)
    {
      nrf24_flush_tx(&radio);
      nrf24_flush_rx(&radio);
    }

  Serial.println(" done.");

  // Show title and wait for user to read it.
  Serial.println("Scanning... Please wait");

  // Channel scan.
  channel_scan(&radio);

  // If we didn't detect it, drop out.
  if (crazyflie_channel == 0) {
    Serial.println("ERROR: Crazyflie Not Detected");
  } else {
    // Control copter.
    Serial.println("Crazyflie Go!");

    // Turn link LED on.
    digitalWrite(LED_PIN, 1);

    // Send command packets.
    nrf24_set_channel(&radio, crazyflie_channel);
  }


  //wdt_enable(WDTO_1S);
  ppmInit();
  sei();
  ppmNewData=1;
}


void loop() {

  NRF24_RADIO_t radio;
  CRAZYFLIE_COMMANDER_SETPOINT_t setpoint;
  CRAZYFLIE_CONSOLE_t ack;
  unsigned int ack_size;
  int stat;
  static int last_stat;

  // Setpoint command.
  setpoint.c = 3 << 4;
  setpoint.roll = 0;
  setpoint.pitch = 0;
  setpoint.yaw = 0;
  //wdt_reset();

  if (ppmNewData) {
    //Serial.println("new PPM Data!");
    ppmNewData=0;
    for (uint8_t i=0;i<sizeof(reportBuffer);i++) {
      unsigned char val=ppmGet(i);
      if (reportBuffer[i]!=val) {
        reportBuffer[i]=val;
      }
    }

    //setpoint.pitch = (float)(16 - report.right_joystick_y) / 16 * 50;
    setpoint.yaw = (float)(reportBuffer[0] - 128) / 128 * 256;
    setpoint.pitch = (float)(reportBuffer[1] - 128) / 128 * 50;
    setpoint.thrust = reportBuffer[2] * 256;
    setpoint.roll = (float)(reportBuffer[3] - 128) / 128 * 50;

    // x-mode
    float roll_copy = setpoint.roll;
    setpoint.roll = 0.707 * (setpoint.roll - setpoint.pitch);
    setpoint.pitch = 0.707 * (roll_copy + setpoint.pitch);

    setpoint.pitch = -setpoint.pitch;
      
  }

  // Send control packet.
  stat = nrf24_send_packet(&radio, &setpoint, sizeof(setpoint), &ack, &ack_size);

  // Reflect status on link LED: on when good link, off when lost.
  digitalWrite(LED_PIN, stat > 0);

}

