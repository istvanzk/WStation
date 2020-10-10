// server_mq.cpp
// -*- mode: C++ -*-
// RH_RF69 or RH_RF22 based server on Raspberry Pi, with POSIX IPC Message Queue interface
// Uses the bcm2835 library to access the GPIO pins to drive the RFM22B or RFM69HCW module
// Uses POSIX inter-process message queue to make available the received data packet and other info
// Requires bcm2835 library to be already installed: http://www.airspayce.com/mikem/bcm2835/
// Requires boot/config.txt contains the line dtoverlay=gpio-no-irq
//
// Author: Istvan Z. Kovacs, 2019-2020
//

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
#include <ctime.h>


/// High level configurations

// Select the radio to use. Can be done in the Makefile.
// RFM22B based module
//#define RH_RF22
// OR
// RFM69HCW based module
//#define RH_RF69

// Select debug info level to stdout
// Output to stderr is always active!
//#define DEBUG_LEV1
//#define DEBUG_LEV2

// Select use of Meessage Queue (POSIX IPC)
// Undef to disable the meessage quueue
#define USE_MQ

// Select the radio driver to use
#if defined (RH_RF22)
#include <RH_RF22.h>
#elif defined (RH_RF69)
#include <RH_RF69.h>
#else
fprintf(stderr, "%s: The RH_RF22 or RH_RF69 macro must be defined!\n", __BASEFILE__);
#endif


/// Low level configurations

// Constants with CS and IRQ pin definition
// HopeRF RFM22B or RFM69HCW based radio modules (no onboard led - reset pin not used)
// see https://www.sparkfun.com/products/retired/10154, https://www.sparkfun.com/products/13910
#define RF_CS_PIN  RPI_V2_GPIO_P1_24 // Slave Select on CE0 so P1 connector pin #24
#define RF_IRQ_PIN RPI_V2_GPIO_P1_22 // IRQ on GPIO25 so P1 connector pin #22
#define RF_LED_PIN NOT_A_PIN	     // No onboard led to drive
#define RF_RST_PIN NOT_A_PIN		 // No onboard reset

// Commonn radio configurations
#define RF_FREQUENCY  434.0
#define RF_GROUP_ID   22 // All devices
#define RF_GATEWAY_ID 1  // Server ID (where to send packets)
#define RF_NODE_ID    10 // Client ID (device sending the packets)


// Message queues parameter
#if defined(USE_MQ)
#define MAX_TXMSG_SIZE 255
#endif

#if defined (RH_RF22)

// RFM22B Configuration
#define RF_TXPOW      RH_RF22_TXPOW_11DBM

// Message queue name
#if defined(USE_MQ)
#define TXQUEUE_NAME  "/rf22_server_tx"
#endif

#elif defined (RH_RF69)

// RFM69HCW Configuration
// Since the radio is the high power HCW model, the Tx power is set in the range 14 to 20
#define RF_TXPOW      14 

// Message queue name
#if defined(USE_MQ)
#define TXQUEUE_NAME  "/rf69_server_tx"
#endif

#endif

#define CHECK(x) \
    do { \
        if (!(x)) { \
            fprintf(stderr, "%s:%d: ", __func__, __LINE__); \
            perror(#x); \
            exit(1); \
        } \
    } while (0) \


#if defined(USE_MQ)
// Message queue descriptor
mqd_t mqTX;
// Message queues attributes
struct mq_attr tx_attr;
// TX message buffer
char mqTX_buffer[MAX_TXMSG_SIZE];
// TX default message priority to use
unsigned int TXmsg_prio = 0;
#endif

// Create an instance of the RFM driver
#if defined (RH_RF22)
RH_RF22 rfmdrv(RF_CS_PIN, RF_IRQ_PIN);
#define RH_RFM_MAX_MESSAGE_LEN RH_RF22_MAX_MESSAGE_LEN
#elif defined (RH_RF69)
RH_RF69 rfmdrv(RF_CS_PIN, RF_IRQ_PIN);
#define RH_RFM_MAX_MESSAGE_LEN RH_RF69_MAX_MESSAGE_LEN
#endif

// Time and time string
time_t tm_now;
char *char_time;
struct tm *timeinfo;

struct rfmmsg_t {
    uint16_t tm_sec;
    uint16_t tm_min;
    uint16_t tm_hour;
    uint16_t tm_mday;
    uint16_t tm_mon;
    uint16_t tm_year;
    uint16_t tm_wday;
    uint16_t tm_yday;
    uint16_t tm_isdst;
    uint8_t from;        //= rfmdrv.headerFrom();
    uint8_t to;          //= rfmdrv.headerTo();
    uint8_t id;          //= rfmdrv.headerId();
    uint8_t flags;       //= rfmdrv.headerFlags();
    int8_t rssi_dBm;     //= rfmdrv.lastRssi();
    uint8_t len;         //= sizeof(buf);
    uint8_t buf[RH_RFM_MAX_MESSAGE_LEN];
} rfm_message;

// Flag for stop
volatile sig_atomic_t force_stop = false;

// Signal handler
void end_sig_handler(int sig)
{
#if defined(DEBUG_LEV2)
    fprintf(stdout, "\n%s Interrupt signal (%d) received. Exiting!\n", __BASEFILE__, sig);
#endif

    // End SPI & close bcm
    bcm2835_gpio_clr_len(RF_IRQ_PIN);
    bcm2835_spi_end();
    bcm2835_close();

    // Close & unlink message queues
    close_mq();

    if(sig>0)
        exit(sig);
}

void close_mq()
{
    // Close & unlink message queue
#if defined(TXQUEUE_NAME)
    CHECK((mqd_t)-1 != mq_close(mqTX));
    CHECK((mqd_t)-1 != mq_unlink(TXQUEUE_NAME));
#endif
}

// Set current time info in the rfm_message
void set_rfmmsg_timeinfo()
{
    // Convert some of the fields to match Python time.struct_time
    struct tm *timeinfo   = localtime(&tm_now);
    rfm_message.tm_sec   = (uint16_t) timeinfo->tm_sec;
    rfm_message.tm_min   = (uint16_t) timeinfo->tm_min;
    rfm_message.tm_hour  = (uint16_t) timeinfo->tm_hour;
    rfm_message.tm_mday  = (uint16_t) timeinfo->tm_mday;
    rfm_message.tm_mon   = (uint16_t) timeinfo->tm_mon + 1; // => months since January: 1-12
    rfm_message.tm_year  = (uint16_t) (timeinfo->tm_year + 1900); // => absolute year
    if (timeinfo->tm_wday == 0) {                            // => days since Monday: 0-6
        rfm_message.tm_wday  = (uint16_t) 6;    
    } else {
        rfm_message.tm_wday  = (uint16_t) (timeinfo->tm_wday - 1); 
    }    
    rfm_message.tm_yday  = (uint16_t) timeinfo->tm_yday + 1;// => days since January 1: 1-366
    rfm_message.tm_isdst = (uint16_t) timeinfo->tm_isdst;
}

// Main Function
// TODOs:
// - Infinite loop with error detection
int main (int argc, const char* argv[] )
{
    uint8_t syncwords[2];
    uint8_t buf[RH_RFM_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    uint8_t to;
    uint8_t id;
    uint8_t flags;
    int8_t rssi_dBm;
    uint8_t last_id = 0;

    fprintf(stdout, "%s started\n", __BASEFILE__);

    // Signal handler
    signal(SIGABRT, end_sig_handler);
    signal(SIGTERM, end_sig_handler);
    signal(SIGINT, end_sig_handler);

    //
    // Create the message queue
    //

    // Data from RFM module & additional info from this server app made available to other client app
#if defined(TXQUEUE_NAME)
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
#if defined(DEBUG_LEV2)
    fprintf(stdout,"MQ: TX open() OK. rfmmsg_t: %02dB\n", sizeof(struct rfmmsg_t));
#endif

    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE); /* reset message buffer */
#endif


    //
    // Init the bcm2835 library
    //
    unsigned long starttime = millis();
    while (!bcm2835_init()) {
        fprintf(stderr, "\n%s BCM2835: init() failed\n", __BASEFILE__);
        if ((millis() - starttime) > 1000) {
            close_mq();
            return 1;
        }
        usleep(100000);
    }

    // IRQ Pin input/pull up
    // When RX packet is available the pin is pulled down (IRQ is low on RFM!)
    bcm2835_gpio_fsel(RF_IRQ_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(RF_IRQ_PIN, BCM2835_GPIO_PUD_UP);

#if defined(TXQUEUE_NAME)
    // Send init msg on the queue
    tm_now  = time(NULL);
    set_rfmmsg_timeinfo();

    rfm_message.len   = (uint8_t) snprintf((char*) rfm_message.buf, RH_RF22_MAX_MESSAGE_LEN, "BCM2835: Init OK. IRQ=GPIO%d", RF_IRQ_PIN);

    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
    memcpy(mqTX_buffer, (const char*)&rfm_message, sizeof(struct rfmmsg_t));
    if ( mq_send(mqTX, mqTX_buffer, MAX_TXMSG_SIZE, TXmsg_prio) ) {
        perror("MQ: TX Init message send failed!");
    }
#if defined(DEBUG_LEV2)
    fprintf(stderr, "MQ: TX Init message sent with header (%d,%d,%d,%d,%d,%d,%d)\n", rfm_message.tm_sec, rfm_message.tm_min, rfm_message.tm_hour, rfm_message.tm_mday, rfm_message.tm_mon, rfm_message.tm_year, rfm_message.len);
#endif

#endif


    // Init the RFM module
    if (!rfmdrv.init()) {
        fprintf(stderr, "\nRFM: Module init failed. Please verify wiring/module\n");
        end_sig_handler(1);
    } 
#if defined(DEBUG_LEV2)    
    else {
        // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36, 8dBm Tx power
        fprintf(stdout, "RFM: Module init OK. Using: CS=GPIO%d, IRQ=GPIO%d\n", RF_CS_PIN, RF_IRQ_PIN);
    }
#endif

    // Since we check IRQ line with bcm_2835 level detection
    // in case radio already have a packet, IRQ is low and will never
    // go to high so never fire again
    // Except if we clear IRQ flags and discard one if any by checking
    rfmdrv.available();

    // Enable Low Detect for the RF_IRQ_PIN pin
    // When a low level is detected, sets the appropriate pin in Event Detect Status
    bcm2835_gpio_len(RF_IRQ_PIN);
#if defined(DEBUG_LEV2)
    fprintf(stdout, "BCM2835: Low detect enabled on GPIO%d\n", RF_IRQ_PIN);
#endif


    // Set transmitter power to at least 11 dBm (up to 20dBm)
    rfmdrv.setTxPower(RF_TXPOW);

    // Set Network ID (by sync words)
    // Use this for any non-default setup, else leave commented out
    syncwords[0] = 0x2d;
    syncwords[1] = 0xd4; //RF_GROUP_ID;
    rfmdrv.setSyncWords(syncwords, sizeof(syncwords));

    // Adjust Frequency
    //rfmdrv.setFrequency(RF_FREQUENCY,0.05);

    // This is our Gateway ID
    rfmdrv.setThisAddress(RF_GATEWAY_ID);
    rfmdrv.setHeaderFrom(RF_GATEWAY_ID);

    // Where we're sending packet
    rfmdrv.setHeaderTo(RF_NODE_ID);

    // Be sure to grab all node packet
    // we're sniffing to display, it's a demo
    rfmdrv.setPromiscuous(false);

    // We're ready to listen for incoming message
    rfmdrv.setModeRx();

    // Current time
    tm_now    = time(NULL);

    char_time = ctime(&tm_now);
    char_time[24] = '\0' ;
    fprintf(stdout, "%s - RFM: Init OK. Group #%d, GW 0x%02X to Node 0x%02X. %3.2fMHz, 0x%02X TxPw\n", char_time, RF_GROUP_ID, RF_GATEWAY_ID, RF_NODE_ID, RF_FREQUENCY, RF_TXPOW);

#if defined(TXQUEUE_NAME)
    // Send msg on the queue
    set_rfmmsg_timeinfo();
    rfm_message.len = (uint8_t) snprintf((char*) rfm_message.buf, RH_RF22_MAX_MESSAGE_LEN, "RFM: Init OK. CS=GPIO%d, IRQ=GPIO%d. Group #%d, GW 0x%02X to Node 0x%02X. %3.2fMHz, 0x%02X TxPw", RF_CS_PIN, RF_IRQ_PIN, RF_GROUP_ID, RF_GATEWAY_ID, RF_NODE_ID, RF_FREQUENCY, RF_TXPOW);
    memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
    memcpy(mqTX_buffer, (const char*)&rfm_message, sizeof(struct rfmmsg_t));
    if ( mq_send(mqTX, mqTX_buffer, MAX_TXMSG_SIZE, TXmsg_prio) ) {
        perror("MQ: TX Start message send failed!");
    }
#endif


#if defined(DEBUG_LEV2)
    fprintf(stdout, "\tListening ...\n");
#endif

    // Begin the main loop
    while (!force_stop) {

      // Low Detect ?
      if (bcm2835_gpio_eds(RF_IRQ_PIN)) {

        // Clear the eds flag
        bcm2835_gpio_set_eds(RF_IRQ_PIN);
#if defined(DEBUG_LEV2)
        fprintf(stdout, "BCM2835: LOW detected for pin GPIO%d\n", RF_IRQ_PIN);
#endif
        // Get the packet, header information and RSSI
        if (rfmdrv.recvfrom(buf, &len, &from, &to, &id, &flags)) {
            rssi_dBm = rfmdrv.lastRssi();
            tm_now = time(NULL);

#if defined(DEBUG_LEV1)
            char_time = ctime(&tm_now);
            char_time[24] = '\0' ;
            fprintf(stdout, "%s - RFM: Packet received, %02d bytes, from 0x%02X to 0x%02X, ID: 0x%02X, F: 0x%02X, with %ddBm => '", char_time, len, from, to, id, flags, rssi_dBm);
            fprintbuffer(stdout, buf, len);
            fprintf(stdout, "'\n");
#endif
            // Send back an ACK if not done already
            if ((id != last_id) || (flags == 0x40)) {
                uint8_t ack = '!';
                rfmdrv.setHeaderId(id);
            	rfmdrv.setHeaderFlags(0x80);
            	rfmdrv.sendto(&ack, sizeof(ack), from);
            	rfmdrv.setModeRx();
                last_id = id;
            }

#if defined(TXQUEUE_NAME)
            // Send msg on the TX queue. Payload: the first-transmission (not re-transmssion) packets.
            if (flags == 0x0) {
                // Send msg on the queue
                set_rfmmsg_timeinfo();
                rfm_message.from     = from;
                rfm_message.to       = to;
                rfm_message.id       = id;
                rfm_message.flags    = flags;
                rfm_message.rssi_dBm = rssi_dBm;
                rfm_message.len      = len;
                memcpy(rfm_message.buf, buf, len);
                memset(mqTX_buffer, 0, MAX_TXMSG_SIZE);
                memcpy(mqTX_buffer, (const char*)&rfm_message, sizeof(struct rfmmsg_t));
                if ( mq_send(mqTX, mqTX_buffer, sizeof(struct rfmmsg_t), TXmsg_prio) ) {
                    perror("MQ: TX Loop message send failed!");
                    // When MQ is full, this returns "TX MQ loop send failed: Resource temporarily unavailable" corresponding to EAGAIN error code
                }
            }
#endif

        } 
#if defined(DEBUG_LEV2)
        else {
            fprintf(stdout,"RFM: Packet receive failed\n");
        }
#endif
      }

#if defined(DEBUG_LEV1) || defined(DEBUG_LEV2)
      fflush(stdout);
#endif
      // Let OS do other tasks
      bcm2835_delay(100);

    }

    // End & close
    fprintf(stdout, "\n%s Ending\n", __BASEFILE__ );
    end_sig_handler(0);

    return 0;
}

