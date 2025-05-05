#include "dw3000.h"

const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4; // spi select pin

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
  5,               /* Channel number. */
  DWT_PLEN_128,    /* Preamble length. Used in TX only. */
  DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
  9,               /* TX preamble code. Used in TX only. */
  9,               /* RX preamble code. Used in RX only. */
  1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,      /* Data rate. */
  DWT_PHRMODE_STD, /* PHY header mode. */
  DWT_PHRRATE_STD, /* PHY header rate. */
  (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0      /* PDOA mode off */
};

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32_t status_reg;

static uint8_t rx_buffer[128];

void setup()
{
  UART_init();

  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  delay(200); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  dwt_softreset();
  delay(200);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  dwt_wakeup_ic();
    Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event) 

  /* Restore the required configurations on wake */
  dwt_restoreconfig();

  pinMode(PIN_IRQ, INPUT_PULLUP);
  attachInterrupt(PIN_IRQ, dwt_isr, RISING);

  /* Register RX callback. */
  dwt_setcallbacks(NULL, rx_ok_cb, rx_err_cb, rx_err_cb, NULL, NULL);

  /* Enable wanted interrupts (RX good frames and RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);

  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_IRQ, 1);
  esp_light_sleep_start();
}

void loop()
{
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len <= sizeof(rx_buffer))
  {
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[frame_len - 2] = 0;
    Serial.println((char*)&rx_buffer[2]);
    Serial.println(rx_buffer[1]);
  }

  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  esp_light_sleep_start();
}

static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
}

static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
}
