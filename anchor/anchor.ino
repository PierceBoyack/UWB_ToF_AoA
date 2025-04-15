#include "dw3000.h"
#include <SPI.h>
#include <WiFi.h>
#include <cmath>


#define PIN_RST 27
#define PIN_IRQ 34
#define PIN_SS 4

#define RNG_DELAY_MS 1000
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
#define POLL_TX_TO_RESP_RX_DLY_UUS 240
#define RESP_RX_TIMEOUT_UUS 400

// Info to connect to WiFi
const char *ssid = "Verizon_9HQ43J";
const char *password = "mew9-rich-ego";
WiFiClient client;

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. RX_TUNE_EN is set to 1.  Operating at 64MHz */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance;
extern dwt_txconfig_t txconfig_options;

float estimateReceiveSignalPower(){
  //Get DGC Decision index
  uint8_t dgcDecisionRegister[4];
  dwt_readfromdevice(0x03, 0x60, 4, dgcDecisionRegister);
  uint8_t dgcDecision = (dgcDecisionRegister[3] >> 4) & 0x07; // 0x07 = 00000111
  //Get Channel Impulse Response Power
  uint8_t ipDiag1[4];
  dwt_readfromdevice(0x0C, 0x2C, 4, ipDiag1);
  uint32_t bigEndIpDiag1 = ((uint32_t)ipDiag1[3] << 24) | ((uint32_t)ipDiag1[2] << 16) |
                          ((uint32_t)ipDiag1[2] << 8) | ((uint32_t)ipDiag1[0]);
  uint32_t CIR = bigEndIpDiag & 0x1FFFF;
  //GET Preamble Accumulation Count
  uint8_t ipDiag12[4];
  uint32_t bigEndIpDiag12 = ((uint32_t)ipDiag12[3] << 24) | ((uint32_t)ipDiag12[2] << 16) |
                          ((uint32_t)ipDiag12[2] << 8) | ((uint32_t)ipDiag12[0]);
  uint32_t PreambleAC = bigEndIpDiag12 & 0xFFF;
  dwt_readfromdevice(0x0C, 0x2C, 4, ipDiag12);
  //signal constant
  float A = 121.7;
  int cFactor = 0x200000; //2^21
  float rxLevel = 10*(log10((CIR*cFactor)/(PreambleAC*PreambleAC))) + (6*dgcDecision) - A;
  return rxLevel;
}

//Connect to WiFi and get server IP from user.
//Returns 0 on success, -1 on failure
int wifiSetup(String &serverIP){
  WiFi.begin(ssid, password);
  short connectAttempt = 0;
  Serial.println("Attempting to connect to WiFi...");
  while ((WiFi.status() != WL_CONNECTED) && (connectAttempt < 20)) {
    delay(500);
    Serial.print(".");
    connectAttempt++;
  }
  if (connectAttempt >= 20){
    Serial.println("Failed to connect to WiFi");
    return -1;
  }  
  Serial.println("");
  Serial.println("WiFi connected.\n");
  IPAddress ip;
  ip = WiFi.localIP();
  printf("My IP: ");
  Serial.println(ip);
  Serial.println("In 30 seconds, enter IP address of computer proxy:\n");
  Serial.setTimeout(30000);
  serverIP = Serial.readStringUntil('\n');
  serverIP.trim();
  return 0;
}

//Connect to server. Returns 0 on success, -1 on failure
int connectToServer(String serverIP){
  IPAddress ip;
  ip.fromString(serverIP);
  Serial.printf("Connecting to ");
  Serial.print(ip);
  Serial.println("");
  int status = client.connect(ip, 12345);
  if(status == true){
    Serial.println("Connected to Server");
    return 0;
  } else {
    Serial.println("Failed to connect to server");
    return -1;
  }
  return -1;
}

void setup()
{
  UART_init();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  //enable CIA diagnostics, clear minDiag
  uint_t minDiag[4];
  dwt_readfromdevice(0x0E, 0X00, 4, minDiag);
  minDiag[2] &= ~(1 << 4);
  dwt_writetodevice(0x0E, 0x00, 4, minDiag);

  //connect to WiFi and Server
  String serverIP;
  while(wifiSetup(serverIP) == -1){
    wifiSetup(serverIP);
  }
  while(connectToServer(serverIP) == -1){
    connectToServer(serverIP);
  }


  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1)
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range RX");
  Serial.println("Setup over........");
}

void loop()
{
  /* Write frame data to DW IC and prepare transmission. See NOTE 7 below. */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);          /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);

      /* Check that the frame is the expected response from the companion "SS TWR responder" example.
       * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        /* Get timestamps embedded in response message. */
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance on LCD. */
        snprintf(dist_str, sizeof(dist_str), "DIST: %3.2f m", distance);
        test_run_info((unsigned char *)dist_str);
        client.write(dist_str, sizeof(dist_str));
        /* Display signal power */
        float powerEstimate = estimateReceiveSignalPower();
        char powerStr[16];
        memset(powerStr, '\0', 16);
        snprintf(powerStr, sizeof(powerStr), "Power: %3.2f dBm", powerEstimate);
        client.write(powerStr, 16);
      }
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }

  /* Execute a delay between ranging exchanges. */
  Sleep(RNG_DELAY_MS);
}