#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  client_mq.py
#
#  Copyright 2019 Istvan Z. Kovacs
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#
#

import time
import sys
import os
import signal
import struct
import posix_ipc

# Message queue parameters
RXQUEUE_NAME  = "/rf22b_server_tx"
MAX_RXMSG_SIZE = 255


def sigint_handler(signal, frame):

    mqRX.close()
    print("CTRL-C\n")
    sys.exit(signal)

def process_notification(mq):

    s, p = mq.receive()
    sd = s.decode()

    # Re-register for notifications
    mq.request_notification((process_notification, mq))

    print("Message received: %s (%d) - %s\n" % (s, p, sd))



# Create the message queue
# The umask is not relevant for reading, when the queue has been created as RW!
#old_umask = os.umask(0)
#os.umask(old_umask)
mqRX = posix_ipc.MessageQueue(RXQUEUE_NAME, posix_ipc.O_RDONLY)

# Request notifications
#mqRX.request_notification((process_notification, mqRX))

# Register SIGINT signal handler
signal.signal(signal.SIGINT, sigint_handler)


def main(args):

    m = 0
    while True:
        s, p = mqRX.receive()
        unpacked_s = struct.unpack('HHHHHHBBBBBb', s[:18])
        #sd = s.decode()
        print("Message#%d:(%d):" % (m, p))
        print(unpacked_s)
        #print(":".join("{:02x}".format(ord(c)) for c in s[18:38].decode()))
        #print(":".join("{:1s}".format(chr(c)) for c in s[18:38]))
        ii = 18
        lng = 0
        while ii < 38:
            if chr(s[ii]) == 'N':
                val = s[ii+1]
                lng = 1

                print(F"{chr(s[ii]):1s}: {val:1d}")
               
            elif chr(s[ii]) in ['S', 'T', 'P', 'H']:
                if chr(s[ii]) is 'P':
                    val_d = bytearray(s[ii+1:ii+6])
                    lng = 5
                else:
                    val_d = bytearray(s[ii+1:ii+4])                    
                    lng = 3

                if val_d[0] == 0x20:
                    val_d[0] = 0x30

                val = 0
                for dd in range(lng):
                    val += (val_d[lng-dd-1]-0x30)*10**(dd-1)   

                print(F"{chr(s[ii]):1s}: {val:.1f}")
                #print("".join("{:1s}".format(chr(c)) for c in val_d))

            ii += (lng+1)

        m = m+1

if __name__ == '__main__':
    sys.exit(main(sys.argv))
