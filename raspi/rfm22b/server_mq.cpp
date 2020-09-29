// rf22b_server_mq.cpp
// -*- mode: C++ -*-
// Test RH_RF22B based server on Raspberry Pi, with POSIX IPC Message Queue interface
// Uses the bcm2835 library to access the GPIO pins to drive the RFM22B module
// Uses POSIX inter-process message queue to make available the received data packet and other info
// Requires bcm2835 library to be already installed: http://www.airspayce.com/mikem/bcm2835/
// Requires boot/config.txt contains the line dtoverlay=gpio-no-irq
//
// Based on sample code rf22b_server.cpp
// By Istvan Z. Kovacs

#include <bcm2835.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mqueue.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <mqueue.h>
#include <time.h>

#include <RH_RF22.h>

// Undef/comment to remove output to stdout
// Output to stderr is always active
#define STDOUT_MSG

// RFM22B board
#define BOARD_RFM22B

// Constants with CS and IRQ pin definition
// HopeRF RFM22B based radio modules (no onboard led - reset pin not used)
// =========================================
// see http://www.sparkfun.com/products/10153
#define RF_CS_PIN  RPI_V2_GPIO_P1_24 // Slave Select on CE0 so P1 connector pin #24
#define RF_IRQ_PIN RPI_V2_GPIO_P1_22 // IRQ on GPIO25 so P1 connector pin #22
#define RF_LED_PIN NOT_A_PIN	     // No onboard led to drive
#define RF_RST_PIN NOT_A_PIN		 // No onboard reset

// Our RFM22B Configuration
#define RF_FREQUENCY  434.0
#define RF_TXPOW      RH_RF22_TXPOW_11DBM
#define RF_GROUP_ID   22 // All devices
#define RF_GATEWAY_ID 1  // Server ID (where to send packets)
#define RF_NODE_ID    10 // Client ID (device sending the packets)


// Message queues parameters
#define TXQUEUE_NAME  "/rf22b_server_tx"
#define MAX_TXMSG_SIZE 255

//#define RXQUEUE_NAME  "/rf22b_server_rx"
#define MAX_RXMSG_SIZE 10


#define CHECK(x) \
    do { \
        if (!(x)) { \
            fprintf(stderr, "%s:%d: ", __func__, __LINE__); \
            perror(#x); \
            exit(1); \
        } \
    } while (0) \


// MQ descriptors
mqd_t mqTX;
mqd_t mqRX;
// MQ attributes
struct mq_attr tx_attr;
struct mq_attr rx_attr;
// TX message buffer
char mqTX_buffer[MAX_TXMSG_SIZE];
// RX message buffer
char mqRX_buffer[MAX_RXMSG_SIZE];
// TX default message priority to use
unsigned int TXmsg_prio = 0;


// Time and time string
time_t tm_now;
char *char_time;
struct tm *timeinfo;

struct rf22msg_t {
    //struct tm timeinfo; //= localtime(&tm_now);
    uint16_t tm_sec;
    uint16_t tm_min;
    uint16_t tm_hour;
    uint16_t tm_mday;
    uint16_t tm_mon;
    uint16_t tm_year;
    uint8_t from;        //= rf22.headerFrom();
    uint8_t to;          //= rf22.headerTo();
    uint8_t id;          //= rf22.headerId();
    uint8_t flags;       //= rf22.headerFlags();
    int8_t rssi_dBm;     //= rf22.lastRssi();
    uint8_t len;         //= sizeof(buf);
    uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];
} rf22_message;


// Create an instance of the RF22B driver
RH_RF22 rf22(RF_CS_PIN, RF_IRQ_PIN);

// Flag for stop
volatile sig_atomic_t force_stop = false;

// Signal handler
void end_sig_handler(int sig)
{
#ifdef STDOUT_MSG
    fprintf(stdout, "\n%s Interrupt signal (%d) received. Exiting!\n", __BASEFILE__, sig);
#endif

    // End SPI & close bcm
    bcm2835_gpio_clr_len(RF_IRQ_PIN);
    bcm2835_spi_end();
    bcm2835_close();

    // Close & unlink message queues
#ifdef TXQUEUE_NAME
    CHECK((mqd_t)-1 != mq_close(mqTX));
    CHECK((mqd_t)-1 != mq_unlink(TXQUEUE_NAME));
#endif
#ifdef RXQUEUE_NAME
    CHECK((mqd_t)-1 != mq_close(mqRX));
    CHECK((mqd_t)-1 != mq_unlink(RXQUEUE_NAME));
#endif

    if(sig>0)
        exit(sig);
}


// Main Function
// TODOs:
// - Re-start if the bcm2835_init fails or hangs
// - Put RX radio packet data into a structure to be sent over MQ msg - Done
// - Open/create msg queue for data write - Done
// - Open/create msg queue for data read - Done
// - Put RX data msg to the TX queue - Partly done. timeout, priorities?
// - Read TX data msg from the RX queue - timout, priorities?
// - Infinite loop with error detection
// - ...
int main (int argc, const char* argv[] )
{
    uint8_t syncwords[2];
    uint8_t buf[RH_RF22_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    uint8_t to;
    uint8_t id;
    uint8_t flags;
    int8_t rssi_dBm;
    uint8_t last_id = 0;

#ifdef STDOUT_MSG
    fprintf(stdout, "%s started\n", __BASEFILE__);
#endif

    // Signal handler
    signal(SIGABRT, end_sig_handler);
    signal(SIGTERM, end_sig_handler);
    signal(SIGINT, end_sig_handler);

    //
    // Create the message queues
    //

    // Data from RF22B module & additional info from this server app made available to other client app
#ifdef TXQUEUE_NAME
    tx_attr.mq_flags   = 0;                 /* Flags (ignored for mq_open()): 0 or O_NONBLOCK */
    tx_attr.mq_maxmsg  = 10;                /* Max. # of messages on queue; /proc/sys/fs/mqueue/msg_max */
    tx_attr.mq_msgsize = MAX_TXMSG_SIZE;    /* Max. message size (bytes); /proc/sys/fs/mqueue/msgsize_max */
    tx_attr.mq_curmsgs = 0;                 /* # of messages currently in queue (ignored for mq_open()) */
    mode_t omask;
    omask = umask(0);
    mqTX = mq_open (TXQUEUE_NAME, O_WRONLY | O_CREAT | O_NONBLOCK,  S_IWUSR | S_IRUSR | S_IRGRP | S_IROTH | S_IWGRP | S_IWOTH, &tx_attr); //
    umask(omask);
    if((mqd_t)-1 == mqTX) {
        perror("TX MQ open failed");
        exit(1);
    }
#ifdef STDOUT_MSG
    fprintf(stdout,"MQ: TX open() OK. rf22msg_t: %02dB\n", sizeof(struct rf22msg_t));
#endif

    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE); /* reset message buffer */
#endif

    // Data & cmd from the other client app consuming the RF22B data (e.g. a Python script)
#ifdef RXQUEUE_NAME
    rx_attr.mq_flags   = 0;                 /* Flags (ignored for mq_open()): 0 or O_NONBLOCK */
    rx_attr.mq_maxmsg  = 10;                /* Max. # of messages on queue; /proc/sys/fs/mqueue/msg_max */
    rx_attr.mq_msgsize = MAX_RXMSG_SIZE;    /* Max. message size (bytes); /proc/sys/fs/mqueue/msgsize_max */
    rx_attr.mq_curmsgs = 0;                 /* # of messages currently in queue (ignored for mq_open()) */
    mqRX = mq_open (RXQUEUE_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, S_IRUSR | S_IWGRP | S_IWOTH, &rx_attr);
    if((mqd_t)-1 == mqRX) {
        perror("RX MQ open failed");
        exit(1);
    }
#ifdef STDOUT_MSG
    fprintf(stdout,"MQ: RX open() OK\n");
#endif

    memset(mqRX_buffer, 0, MAX_RXMSG_SIZE);
#endif


    // Delay needed?
    //bcm2835_delay(100);

    //
    // Init the bcm2835 library
    //

    // This can sometimes hang indefinetly (why?)
    // TODO#1: timout loop
    //unsigned long starttime = millis();
    //if ((millis() - starttime) > 100)
    if (!bcm2835_init()) {
        fprintf(stderr, "\n%s BCM2835: init() failed\n", __BASEFILE__);
        return 1;
    }

    // IRQ Pin input/pull up
    // When RX packet is available the pin is pulled down (IRQ is low on RF22B!)
    bcm2835_gpio_fsel(RF_IRQ_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(RF_IRQ_PIN, BCM2835_GPIO_PUD_UP);

#ifdef TXQUEUE_NAME
    // Send msg on the queue
    tm_now                = time(NULL);
    timeinfo              = localtime(&tm_now);
    rf22_message.tm_sec   = (uint16_t) timeinfo->tm_sec;
    rf22_message.tm_min   = (uint16_t) timeinfo->tm_min;
    rf22_message.tm_hour  = (uint16_t) timeinfo->tm_hour;
    rf22_message.tm_mday  = (uint16_t) timeinfo->tm_mday;
    rf22_message.tm_mon   = (uint16_t) timeinfo->tm_mon;
    rf22_message.tm_year  = (uint16_t) timeinfo->tm_year;

    rf22_message.len   = (uint8_t) snprintf((char*) rf22_message.buf, RH_RF22_MAX_MESSAGE_LEN, "BCM2835: Init OK. IRQ=GPIO%d", RF_IRQ_PIN);

    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
    memcpy(mqTX_buffer, (const char*)&rf22_message, sizeof(struct rf22msg_t));
    if ( mq_send(mqTX, mqTX_buffer, MAX_TXMSG_SIZE, TXmsg_prio) ) {
        perror("TX MQ init send#1 failed");
    }

    fprintf(stderr, "MQ: First TX message sent with header (%d,%d,%d,%d,%d,%d,%d)\n", rf22_message.tm_sec, rf22_message.tm_min, rf22_message.tm_hour, rf22_message.tm_mday, rf22_message.tm_mon, rf22_message.tm_year, rf22_message.len);
#endif



    // Init the RF22B module
    if (!rf22.init()) {
        fprintf(stderr, "\nRF22B: Module init() failed. Please verify wiring/module\n");
        end_sig_handler(1);
    } else {
        // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36, 8dBm Tx power
#ifdef STDOUT_MSG
        fprintf(stdout, "RF22B: Module seen OK. Using: CS=GPIO%d, IRQ=GPIO%d\n", RF_CS_PIN, RF_IRQ_PIN);
#endif
    }

    // Since we check IRQ line with bcm_2835 level detection
    // in case radio already have a packet, IRQ is low and will never
    // go to high so never fire again
    // Except if we clear IRQ flags and discard one if any by checking
    rf22.available();

    // Enable Low Detect for the RF_IRQ_PIN pin
    // When a low level is detected, sets the appropriate pin in Event Detect Status
    bcm2835_gpio_len(RF_IRQ_PIN);
#ifdef STDOUT_MSG
    fprintf(stdout, "BCM2835: Low detect enabled on GPIO%d\n", RF_IRQ_PIN);
#endif


    // Set transmitter power to at least 11 dBm (up to 20dBm)
    rf22.setTxPower(RF_TXPOW);

    // Set Network ID (by sync words)
    // Use this for any non-default setup, else leave commented out
    syncwords[0] = 0x2d;
    syncwords[1] = 0xd4; //RF_GROUP_ID;
    rf22.setSyncWords(syncwords, sizeof(syncwords));

    // Adjust Frequency
    //rf22.setFrequency(RF_FREQUENCY,0.05);

    // This is our Gateway ID
    rf22.setThisAddress(RF_GATEWAY_ID);
    rf22.setHeaderFrom(RF_GATEWAY_ID);

    // Where we're sending packet
    rf22.setHeaderTo(RF_NODE_ID);

    // Be sure to grab all node packet
    // we're sniffing to display, it's a demo
    rf22.setPromiscuous(false);

    // We're ready to listen for incoming message
    rf22.setModeRx();

    tm_now    = time(NULL);

#ifdef STDOUT_MSG
    char_time = ctime(&tm_now);
    char_time[24] = '\0' ;
    fprintf(stdout, "%s - RF22B: Init OK. Group #%d, GW 0x%02X to Node 0x%02X. %3.2fMHz, 0x%02X TxPw\n", char_time, RF_GROUP_ID, RF_GATEWAY_ID, RF_NODE_ID, RF_FREQUENCY, RF_TXPOW);
#endif

#ifdef TXQUEUE_NAME
    // Send msg on the queue
    timeinfo              = localtime(&tm_now);
    rf22_message.tm_sec   = (uint16_t) timeinfo->tm_sec;
    rf22_message.tm_min   = (uint16_t) timeinfo->tm_min;
    rf22_message.tm_hour  = (uint16_t) timeinfo->tm_hour;
    rf22_message.tm_mday  = (uint16_t) timeinfo->tm_mday;
    rf22_message.tm_mon   = (uint16_t) timeinfo->tm_mon;
    rf22_message.tm_year  = (uint16_t) timeinfo->tm_year;

    rf22_message.len = (uint8_t) snprintf((char*) rf22_message.buf, RH_RF22_MAX_MESSAGE_LEN, "RF22B: Init OK. CS=GPIO%d, IRQ=GPIO%d. Group #%d, GW 0x%02X to Node 0x%02X. %3.2fMHz, 0x%02X TxPw", RF_CS_PIN, RF_IRQ_PIN, RF_GROUP_ID, RF_GATEWAY_ID, RF_NODE_ID, RF_FREQUENCY, RF_TXPOW);

    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
    memcpy(mqTX_buffer, (const char*)&rf22_message, sizeof(struct rf22msg_t));
    if ( mq_send(mqTX, mqTX_buffer, MAX_TXMSG_SIZE, TXmsg_prio) ) {
        perror("TX MQ init send#2 failed");
    }
#endif


#ifdef STDOUT_MSG
    fprintf(stdout, "\tListening ...\n");
#endif

    // Begin the main loop
    while (!force_stop) {

      // Low Detect ?
      if (bcm2835_gpio_eds(RF_IRQ_PIN)) {

        // Clear the eds flag
        bcm2835_gpio_set_eds(RF_IRQ_PIN);
#ifdef STDOUT_MSG
        fprintf(stdout, "BCM2835: LOW detected for pin GPIO%d\n", RF_IRQ_PIN);
#endif
        // Get the packet, header information and RSSI
        if (rf22.recvfrom(buf, &len, &from, &to, &id, &flags)) {
            rssi_dBm = rf22.lastRssi();
            tm_now = time(NULL);

#ifdef STDOUT_MSG
            char_time = ctime(&tm_now);
            char_time[24] = '\0' ;
            fprintf(stdout, "%s - RF22B: Packet received, %02d bytes, from 0x%02X to 0x%02X, ID: 0x%02X, F: 0x%02X, with %ddBm => '", char_time, len, from, to, id, flags, rssi_dBm);
            fprintbuffer(stdout, buf, len);
            fprintf(stdout, "'\n");
#endif
            // Send back an ACK if not done already
            if ((id != last_id) || (flags == 0x40)) {
                uint8_t ack = '!';
                rf22.setHeaderId(id);
            	rf22.setHeaderFlags(0x80);
            	rf22.sendto(&ack, sizeof(ack), from);
            	rf22.setModeRx();
                last_id = id;
            }

            // Send msg on the TX queue. Initial packet only.
#ifdef TXQUEUE_NAME
            if (flags == 0x0) {
                timeinfo              = localtime(&tm_now);
                rf22_message.tm_sec   = (uint16_t) timeinfo->tm_sec;
                rf22_message.tm_min   = (uint16_t) timeinfo->tm_min;
                rf22_message.tm_hour  = (uint16_t) timeinfo->tm_hour;
                rf22_message.tm_mday  = (uint16_t) timeinfo->tm_mday;
                rf22_message.tm_mon   = (uint16_t) timeinfo->tm_mon;
                rf22_message.tm_year  = (uint16_t) timeinfo->tm_year;
                rf22_message.from     = from;
                rf22_message.to       = to;
                rf22_message.id       = id;
                rf22_message.flags    = flags;
                rf22_message.rssi_dBm = rssi_dBm;
                rf22_message.len      = len;
                memcpy(rf22_message.buf, buf, len);

                memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
                memcpy(mqTX_buffer, (const char*)&rf22_message, sizeof(struct rf22msg_t));
                if ( mq_send(mqTX, mqTX_buffer, sizeof(struct rf22msg_t), TXmsg_prio) ) {
                    perror("TX MQ loop send failed");
                    // When MQ is full, this returns "TX MQ loop send failed: Resource temporarily unavailable" corresponding to EAGAIN error code
                }
            }
#endif


        } else {
#ifdef STDOUT_MSG
            fprintf(stdout,"RF22B: Packet receive failed\n");
#endif
        }
      }

      // Check RX queue for ctrl msg
      //...
      //force_stop = true;


#ifdef STDOUT_MSG
      fflush(stdout);
#endif
      // Let OS do other tasks
      bcm2835_delay(100);

    }

    // End & close
#ifdef STDOUT_MSG
    fprintf(stdout, "\n%s Ending\n", __BASEFILE__ );
#endif
    end_sig_handler(0);

    return 0;
}

