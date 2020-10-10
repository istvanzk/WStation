# Makefile
# Compile the RH_RF69 (default) or RH_RF22 (option) based server with POSIX Message Queue on Raspberry Pi
# Note: Requires the bcm2835 library to be already installed, see http://www.airspayce.com/mikem/bcm2835/
#
# Author: Istvan Z. Kovacs, 2019-2020
#

CC            = g++
CFLAGS        = -DRASPBERRY_PI -DBCM2835_NO_DELAY_COMPATIBILITY -D__BASEFILE__=\"$*\" 
LIBS          = -lbcm2835 -lrt
RADIOHEADBASE = ../../../RadioHead
INCLUDE       = -I$(RADIOHEADBASE)

ifdef RFM22
CFLAGS += -DRH_RF22
else
CFLAGS += -DRH_RF69
endif

all: server_mq

RasPi.o: $(RADIOHEADBASE)/RHutil_izk/RasPi.cpp
				$(CC) $(CFLAGS) -c $(RADIOHEADBASE)/RHutil_izk/RasPi.cpp $(INCLUDE)

server_mq.o: server_mq.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<
ifdef RFM22
RH_RF22.o: $(RADIOHEADBASE)/RH_RF22.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<
else
RH_RF69.o: $(RADIOHEADBASE)/RH_RF69.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<
endif

RHHardwareSPI.o: $(RADIOHEADBASE)/RHHardwareSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHSPIDriver.o: $(RADIOHEADBASE)/RHSPIDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHGenericDriver.o: $(RADIOHEADBASE)/RHGenericDriver.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

RHGenericSPI.o: $(RADIOHEADBASE)/RHGenericSPI.cpp
				$(CC) $(CFLAGS) -c $(INCLUDE) $<

ifdef RFM22
server_mq: server_mq.o RH_RF22.o RasPi.o RHHardwareSPI.o RHGenericDriver.o RHGenericSPI.o RHSPIDriver.o
				$(CC) $^ $(LIBS) -o server_mq
else
server_mq: server_mq.o RH_RF69.o RasPi.o RHHardwareSPI.o RHGenericDriver.o RHGenericSPI.o RHSPIDriver.o
				$(CC) $^ $(LIBS) -o server_mq
endif

clean:
				rm -rf *.o server_mq
