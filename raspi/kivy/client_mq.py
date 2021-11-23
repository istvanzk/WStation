'''
Module implements a POSIX IPC Message Queue client class to receive weather data 
from a POSIX IPC Message Queue server running on the same machine

Author: Istvan Z. Kovacs, 2019-2020
'''

import time
import sys
import os
import signal
import struct
import posix_ipc

def end_sig_handler(signal, frame):
    print("\nInterrupt signal received.")
    sys.exit(signal)


class ClientMQ(object):
    '''
    Implements a POSIX IPC Message Queue client class 
    to receive weather data from a POSIX IPC Message Queue server 
    (see e.g. raspi/rfm/server_mq.cpp) running on the same machine
    '''

    # The message queue object
    mqRX = None

    def __init__(self, rxqueue_name):
        # Create the message queue
        # The umask is not relevant for reading, when the queue has been created as RW!
        #old_umask = os.umask(0)
        #os.umask(old_umask)
        if posix_ipc.MESSAGE_QUEUES_SUPPORTED: #hasattr(posix_ipc, 'MessageQueue')
            try:
                # Connect to the queue
                self.mqRX = posix_ipc.MessageQueue(rxqueue_name, posix_ipc.O_RDONLY | posix_ipc.O_CREAT)

                # Request notifications
                #mqRX.request_notification((self.process_notification, foo))

            except:
                self.mqRX = None
                pass
        else:
            self.mqRX = None

    def __del__(self):
        if self.mqRX is not None:
            try:
                self.mqRX.close()
                print(F"MessageQueue {self.mqRX.name:s} closed.")
            except:
                pass

    def process_notification(self,foo):
        '''Notification function'''

        # Read and decode data    
        self.read_weather_data(1)

        # Re-register for notifications
        self.mqRX.request_notification((self.process_notification, foo))


    def read_weather_data(self, timeout_sec=None):
        '''Read message from the MessageQueue and decode weather data'''
        
        # The local dictionary for storing the most recent weather data
        _weather_data = {}

        # Return if no RX message queue
        if self.mqRX is None:
            return None

        # Receive from the MessageQueue
        # Read all available messages and keep the last one only
        bendRX = False
        bmsgRX = False
        while not bendRX:
            try:
                msg, pri = self.mqRX.receive(timeout=timeout_sec)
                bmsgRX = True
            except posix_ipc.BusyError:
                bendRX = True

        if not bmsgRX:
            return None

        # Unpack header info    
        _unpacked_msgheader = struct.unpack('HHHHHHHHHBBBBBb', msg[:24])
        _weather_data["Header"] = list(_unpacked_msgheader)

        # Show raw data
        #print(F"Message ({p:d}):")
        #print(unpacked_msgheader)
        #print(":".join("{:1s}".format(chr(c)) for c in msg[24:44]))

        # Decode the 21 bytes weather data (from the RFM-Arduino)
        if _weather_data["Header"][14] == 21:
            ii = 24
            lng = 0
            while ii < 44:
                if chr(msg[ii]) == 'N':
                    val = msg[ii+1]
                    lng = 1

                    #print(F"{chr(msg[ii]):1s}: {val:1d}")
                    _weather_data[F"{chr(msg[ii]):1s}"] = val
                
                elif chr(msg[ii]) in ['S', 'T', 'P', 'H']:
                    if chr(msg[ii]) is 'P':
                        val_d = bytearray(msg[ii+1:ii+6])
                        lng = 5
                    else:
                        val_d = bytearray(msg[ii+1:ii+4])                    
                        lng = 3

                    if val_d[0] == 0x20:
                        val_d[0] = 0x30

                    val = 0
                    for dd in range(lng):
                        val += (val_d[lng-dd-1]-0x30)*10**(dd-1)   
                        
                    if chr(msg[ii]) is 'T':
                        val = val-100.0 if val > 50 else val

                    #print(F"{chr(msg[ii]):1s}: {val:.1f}")
                    _weather_data[F"{chr(msg[ii]):1s}"] = val

                ii += (lng+1)

        # Initial message with start-up info (string)
        else:
            _weather_data["IniMsg"] = "".join("{:1s}".format(chr(c)) for c in msg[18:(18+_weather_data["Header"][11])])

        return _weather_data

if __name__ == '__main__':

    # Message queue parameters
    # Message queue read interval (seconds)
    RXQUEUE_TIME  = 15 
    # Message queue name to connect to
    RXQUEUE_NAME  = "/rf69_server_tx"

    # Wind meter calibration parameters
    NORTH_INDEX   = 16
    WINDDIR_STEPS = 16

    # Signal handlers
    signal.signal(signal.SIGABRT, end_sig_handler)
    signal.signal(signal.SIGTERM, end_sig_handler)
    signal.signal(signal.SIGINT, end_sig_handler)

    # Create the receive message queue
    # msg_queue.mqRX is set to None when no MessageQueue is available (e.g. MacOS)!
    msg_queue = ClientMQ(RXQUEUE_NAME)

    # Get data from the remote weather station (via local MessageQueue)
    if msg_queue.mqRX is not None:
        
        print(F"MessageQueue {msg_queue.mqRX.name:s} opened succesfully.")

        while True:
            # Wait 
            print(f"Sleep {RXQUEUE_TIME:d} seconds")
            time.sleep(RXQUEUE_TIME)

            # Read message queue
            weather_data = msg_queue.read_weather_data(1.0)

            # Data received
            if weather_data is not None:
                if weather_data["Header"][14] == 20:
                    # Weather data
                    weather_data["IniMsg"] = None

                    # Adjust North direction based on calibration data
                    weather_data["N"]     -= NORTH_INDEX 
                    if weather_data["N"] <= 0:
                        weather_data["N"] += WINDDIR_STEPS 

                    weather_data["N"]  = (360.0/WINDDIR_STEPS) * weather_data["N"]

                # RSSI (dBm)
                weather_data["Header"][13] -= 256

                # Display message
                print(weather_data)

            else:   
                print("No valid message received!")

    else:
        if posix_ipc.MESSAGE_QUEUES_SUPPORTED: 
            print(F"MessageQueue {RXQUEUE_NAME:s} could not be open.")
        else:
            print(F"MessageQueue is not available/supported.")

