#include "dw3000.h"

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

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
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0     /* PDOA mode off */
};

static uint8_t tx_msg[] = {0xC5, 0, 'U', 'n', 'i', 'v', 'e', 'r', 's', 'i', 't', 'y', ' ',
  'a', 't', ' ', 'B', 'u', 'f', 'f', 'a', 'l', 'o', ' ', 'C', 'o', 'm', 'p', 'u', 't', 'e', 'r',
  ' ', 'S', 'c', 'i', 'e', 'n', 'c', 'e'};

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 500 * 5

extern dwt_txconfig_t txconfig_options;

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
    test_run_info((unsigned char *)"IDLE FAILED01\r\n");
    while (100)
      ;
  }

  dwt_softreset();
  delay(200);

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    test_run_info((unsigned char *)"IDLE FAILED02\r\n");
    while (100)
      ;
  }

  // test_run_info((unsigned char *)"IDLE OK\r\n");
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    test_run_info((unsigned char *)"INIT FAILED\r\n");
    while (100)
      ;
  }
  // test_run_info((unsigned char *)"INIT OK\r\n");

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    test_run_info((unsigned char *)"CONFIG FAILED\r\n");
    while (100)
      ;
  }
  // test_run_info((unsigned char *)"CONFIG OK\r\n");
  /* Configure the TX spectrum parameters (power PG delay and PG Count) */
  dwt_configuretxrf(&txconfig_options);

  /* Configure sleep and wake-up parameters. */
  dwt_configuresleep(DWT_CONFIG, DWT_PRES_SLEEP | DWT_WAKE_WUP | DWT_SLP_EN);
  dwt_entersleepaftertx(1);

  uint32_t reg = dwt_read32bitreg(SYS_STATUS_ID);

  /* need to disable default interrupts, device will not go to sleep if interrupt line is high */
  dwt_setinterrupt(SYS_ENABLE_LO_SPIRDY_ENABLE_BIT_MASK, 0, DWT_DISABLE_INT);

  dwt_write32bitreg(SYS_STATUS_ID, reg); //clear interrupt/events
  dwt_wakeup_ic();

  Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event) 

  /* Restore the required configurations on wake */
  dwt_restoreconfig();
}

void loop()
{
  /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
  dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

  /* In this example since the length of the transmitted frame does not change,
   * nor the other parameters of the dwt_writetxfctrl function, the
   * dwt_writetxfctrl call could be outside the main while(1) loop.
   */
  dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

  /* Start transmission. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  test_run_info((unsigned char *)"TX Frame Sent");

  /* Execute a delay between transmissions. */
  //Sleep(TX_DELAY_MS);

  /* Increment the blink frame sequence number (modulo 256). */
  //tx_msg[BLINK_FRAME_SN_IDX]++;
  //Serial.println(tx_msg[BLINK_FRAME_SN_IDX]);
  
  //Serial.println("Configure and enter esp32 sleep");
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_34, 1);

  esp_sleep_enable_timer_wakeup(TX_DELAY_MS * 100);
  esp_deep_sleep_start();
}