#ifndef NRF24_H
#define NRF24_H

// Pin configuration
#define LED_PIN 2
#define NRF24L01_CE 10 // PB2
#define NRF24L01_SS 9  // PB1

// missing defines
#define CTL_NO_ERROR 1
#define CTL_PARAMETER_ERROR -1
#define CTL_DEVICE_NOT_RESPONDING -2

// NRF24 CONFIG register bits.
#define NRF24_CONFIG_MASK_RX_DR      0x40
#define NRF24_CONFIG_MASK_TX_DS      0x20     // Asserted when ACK packet received
#define NRF24_CONFIG_MASK_MAX_RT     0x10
#define NRF24_CONFIG_EN_CRC          0x08
#define NRF24_CONFIG_CRCO            0x04
#define NRF24_CONFIG_PWR_UP          0x02
#define NRF24_CONFIG_PRIM_RX         0x01

// NRF24 EN_AA register bits.
#define NRF24_EN_AA_ENAA_P5          0x20
#define NRF24_EN_AA_ENAA_P4          0x10
#define NRF24_EN_AA_ENAA_P3          0x08
#define NRF24_EN_AA_ENAA_P2          0x04
#define NRF24_EN_AA_ENAA_P1          0x02
#define NRF24_EN_AA_ENAA_P0          0x01

// NRF24 EN_AA register bits.
#define NRF24_EN_RXADDR_ERX_P5       0x20
#define NRF24_EN_RXADDR_ERX_P4       0x10
#define NRF24_EN_RXADDR_ERX_P3       0x08
#define NRF24_EN_RXADDR_ERX_P2       0x04
#define NRF24_EN_RXADDR_ERX_P1       0x02
#define NRF24_EN_RXADDR_ERX_P0       0x08

// NRF24 RF_SETUP bits
#define NRF24_RF_SETUP_CONT_WAVE     0x80     // NRF24L01+ only: continuous carrier
#define NRF24_RF_SETUP_RF_DR_LOW     0x20     // low data rate, 250 kHz, need RF_DR_HIGH=0 if set
#define NRF24_RF_SETUP_PLL_LOCK      0x10
#define NRF24_RF_SETUP_RF_DR_HIGH    0x08
#define NRF24_RF_SETUP_RF_PWR        0x02
#define NRF24_RF_SETUP_RF_LNA_HCURR  0x01

// NRF24 data rates
#define NRF24_RF_SETUP_RF_DR_250kbps (1*NRF24_RF_SETUP_RF_DR_LOW + 0*NRF24_RF_SETUP_RF_DR_HIGH)
#define NRF24_RF_SETUP_RF_DR_1Mbps   (0*NRF24_RF_SETUP_RF_DR_LOW + 0*NRF24_RF_SETUP_RF_DR_HIGH)
#define NRF24_RF_SETUP_RF_DR_2Mbps   (0*NRF24_RF_SETUP_RF_DR_LOW + 1*NRF24_RF_SETUP_RF_DR_HIGH)
#define NRF24_RF_SETUP_RF_DR_MASK    (1*NRF24_RF_SETUP_RF_DR_LOW + 1*NRF24_RF_SETUP_RF_DR_HIGH)

// NRF24 STATUS bits
#define NRF24_STATUS_RX_DR           0x40     // Rx data ready interrupt
#define NRF24_STATUS_TX_DS           0x20     // Tx data sent interrupt
#define NRF24_STATUS_MAX_RT          0x10     // Maximum number of transmit interrupt


typedef struct {
  unsigned char c;
  float roll;
  float pitch;
  float yaw;
  unsigned short thrust;
} __attribute__((packed)) CRAZYFLIE_COMMANDER_SETPOINT_t;

typedef struct {
  unsigned char c;
  union {
    unsigned char data[32];  // one larger, to insert a null.
  } console;
} __attribute__((packed)) CRAZYFLIE_CONSOLE_t;

// NRF24 variants:
typedef enum {
  NRF24L01,     // nRF24L01
  NRF24L01P,    // nRF24L01+
} NRF24_VARIANT_t;


// NRF24 registers
typedef enum {
  NRF24_CONFIG       = 0x00,
  NRF24_EN_AA,
  NRF24_EN_RXADDR,
  NRF24_SETUP_AW,
  NRF24_SETUP_RETR,
  NRF24_RF_CH,
  NRF24_RF_SETUP,
  NRF24_STATUS,
  NRF24_OBSERVE_TX,  // 0x08
  NRF24_CD,
  NRF24_RX_ADDR_P0,
  NRF24_RX_ADDR_P1,
  NRF24_RX_ADDR_P2,
  NRF24_RX_ADDR_P3,
  NRF24_RX_ADDR_P4,
  NRF24_RX_ADDR_P5,
  NRF24_TX_ADDR,     // 0x10
  NRF24_RX_PW_P0,
  NRF24_RX_PW_P1,
  NRF24_RX_PW_P2,
  NRF24_RX_PW_P3,
  NRF24_RX_PW_P4,
  NRF24_RX_PW_P5,
  NRF24_FIFO_STATUS, // 0x07
  NRF24_DYNPD        = 0x1c,
  NRF24_FEATURE      = 0x1d
} NRF24_REGISTER_t;


typedef enum {
  NRF24_CMD_R_REGISTER          = 0x00,
  NRF24_CMD_W_REGISTER          = 0x20,
  NRF24_CMD_RX_PAYLOAD          = 0x61,
  NRF24_CMD_TX_PAYLOAD          = 0xA0,
  NRF24_CMD_FLUSH_TX            = 0xE1,
  NRF24_CMD_FLUSH_RX            = 0xE2,
  NRF24_CMD_TX_PL               = 0xE3,
  NRF24_CMD_ACTIVATE            = 0x50,
  NRF24_CMD_R_RX_PL_WID         = 0x60,
  NRF24_CMD_W_ACK_PAYLOAD       = 0xA8,
  NRF24_CMD_W_TX_PAYLOAD_NO_ACK = 0xB0,
  NRF24_CMD_NOP                 = 0xFF
} NRF24_CMD_t;

typedef struct NRF24_RADIO_s {
  // Shadow registers.
  unsigned short setup;
  unsigned short retr;

  // Operating mode.
  NRF24_VARIANT_t variant;
  unsigned short ack_payload_length;
} NRF24_RADIO_t;

extern int crazyflie_channel;



void nrf24_begin();
void nrf24_write_cs(int state);
void nrf24_write_ce(int state);

void nrf24_read_register(int reg, int *addr, size_t len);

void nrf24_write_register(int reg, const int *addr, size_t len);
void nrf24_write_8b_register(int reg, int value);
int nrf24_cmd(unsigned char cmd);
int nrf24_nop(NRF24_RADIO_t *self);
int nrf24_flush_tx(NRF24_RADIO_t *self);
int nrf24_flush_rx(NRF24_RADIO_t *self);
int nrf24_set_channel(NRF24_RADIO_t *self, int channel);
void nrf24_recompute_retr(NRF24_RADIO_t *self);
int nrf24_set_data_rate(NRF24_RADIO_t *self, int kHz);
int nrf24_set_auto_retransmit_count(NRF24_RADIO_t *self, int count);
int nrf24_set_transmit_power(NRF24_RADIO_t *self, int dBm);
void nrf24_set_ack_payload_length(NRF24_RADIO_t *self, int bytes);
void nrf24_pulse_ce();
void nrf24_transmit_packet(const uint8_t *payload, size_t payload_size);
int nrf24_read_receive_packet(NRF24_RADIO_t *self, unsigned char *payload);
int nrf24_wait_for_interrupt_and_acknowledge(NRF24_RADIO_t *self);
int nrf24_send_packet(NRF24_RADIO_t *self, const CRAZYFLIE_COMMANDER_SETPOINT_t *payload, size_t payload_size, void *ack, size_t *ack_size);
void channel_scan(NRF24_RADIO_t *self);

#endif
