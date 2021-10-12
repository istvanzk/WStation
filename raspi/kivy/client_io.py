'''
Module implements an Adafruit IO client class to send custom feed data (N,T,S,P,H, Rssi)

Author: Istvan Z. Kovacs, 2021
'''

import os
from typing import Any
import datetime
from pathlib import Path

# Import Adafruit IO REST client.
from Adafruit_IO import Client, Feed, Group, Data, errors

# Enable debug info
DEBUG = 0

class AdafruitClientIO(object):
    '''
    Implements the Adafruit IO custom client class
    using groups to send/update multiple feeds with one request 
    See: https://github.com/adafruit/Adafruit_IO_Python/tree/master/Adafruit_IO
    '''
    
    # The Adafruit IO Client and Group
    aio = None
    group = None

    def __init__(self, update_sec=30, cfg_file="aiocfg.txt"):

        # Update period
        # Rate limit is 30 updates every 60 seconds = 6 feeds every 12 seconds
        if update_sec > 60/(30/6):
            self.update_sec = datetime.timedelta(seconds=update_sec)
        else:
            self.update_sec = datetime.timedelta(seconds=12)

        # Force feed update when the send_data_all_feeds() is called for the first time
        self._crt_time  = datetime.datetime.utcnow()
        self._last_time = datetime.datetime.utcnow() - self.update_sec

        # Read the access info
        self.cfg_file = os.path.join(Path(__file__).parent.absolute(), cfg_file)
        try:
            with open(self.cfg_file,'r') as f:
                aio_info = f.read().split('\n')

                # Access
                self.aio_acc = aio_info[0].split(',',2)

                # Feed group
                self.aio_grp = aio_info[1].split(',',2)

                if DEBUG:
                    print(f"Read config file {self.cfg_file}:")
                    print(self.aio_grp)

                # Feed names
                self.aio_feednames = aio_info[2].split(',',6)

                if DEBUG:
                    print(self.aio_feednames)

                # Location (lat, lon and ele)
                self.aio_loc = aio_info[3].split(',',3)

                if DEBUG:
                    print(self.aio_loc)

        except IOError:
            print(f"Adafruit IO:: Configuration file '{self.cfg_file}' read error! Exiting!", exc_info=True)
            raise

        # Create an instance of the REST client.
        self.aio = Client(self.aio_acc[0], self.aio_acc[1])

        # Get/Create the group
        try:
            # Get the WS group
            self.group  = self.aio.groups(self.aio_grp[1])

        except errors.RequestError:
            # The returned object will contain all the details about the created group.
            self.group = self.aio.create_group(Group(name=self.aio_grp[0]))
            pass

        # Get/Create the feeds
        if len(self.group.feeds):

            # Get the feed names
            self._feednames = dict()
            for _idx in range(len(self.group.feeds)):
                self._feednames[self.group.feeds[_idx].name] = _idx

            if DEBUG > 1:
                print(self._feednames)

            self._direction   = self.group.feeds[self._feednames[self.aio_feednames[0]]]
            self._temperature = self.group.feeds[self._feednames[self.aio_feednames[1]]]
            self._speed       = self.group.feeds[self._feednames[self.aio_feednames[2]]]
            self._pressure    = self.group.feeds[self._feednames[self.aio_feednames[3]]]
            self._humidity    = self.group.feeds[self._feednames[self.aio_feednames[4]]]
            self._rssi        = self.group.feeds[self._feednames[self.aio_feednames[5]]]

            if DEBUG:
                print(f"Got group {self.group.name} with {len(self.group.feeds)} feeds.")

        else:

            # Create feeds in the group
            feed = Feed(name=self.aio_feednames[0])
            self._direction = self.aio.create_feed(feed, self.group.name)
            feed = Feed(name=self.aio_feednames[1])
            self._temperature = self.aio.create_feed(feed, self.group.name)
            feed = Feed(name=self.aio_feednames[2])
            self._speed = self.aio.create_feed(feed, self.group.name)
            feed = Feed(name=self.aio_feednames[3])
            self._pressure = self.aio.create_feed(feed, self.group.name)
            feed = Feed(name=self.aio_feednames[4])
            self._humidity = self.aio.create_feed(feed, self.group.name)
            feed = Feed(name=self.aio_feednames[5])
            self._rssi = self.aio.create_feed(feed, self.group.name)

            if DEBUG:
                print(f"Created group {self.group.name} with {len(self.group.feeds)} feeds.")

    def send_data_all_feeds(self, data_dict, metadata=None):
        
        # Trigger update with the set time period
        self._crt_time = datetime.datetime.utcnow()
        if self._crt_time >= self._last_time + self.update_sec:
            
            # Time update
            self._last_time = datetime.datetime.utcnow()

            # Metadata
            metadata = metadata or {'lat': float(self.aio_loc[0]), 
                'lon': float(self.aio_loc[1]), 
                'ele': float(self.aio_loc[2]), 
                'created_at': self._crt_time.isoformat()}

            # Send data to all feeds
            try:
                self.aio.send_data(self._direction.key, data_dict['N'], metadata)
                self.aio.send_data(self._temperature.key, data_dict['T'], metadata)
                self.aio.send_data(self._speed.key, data_dict['S'], metadata)
                self.aio.send_data(self._pressure.key, data_dict['P'], metadata)
                self.aio.send_data(self._humidity.key, data_dict['H'], metadata)
                self.aio.send_data(self._rssi.key, data_dict['Rssi'], metadata)
            except ConnectionError:
                pass

            if DEBUG:
                print(f"Data {data_dict} sent to {len(self.group.feeds)} feeds with metadata {metadata}")

    def send_batch_data(self, feed, data_list, created_at=None):

        # Setup batch data with custom created_at values
        if created_at is None:
            time = datetime.datetime.utcnow().isoformat()
        else:
            time = created_at.isoformat()

        # The data batch    
        data_tuplelist = [Data(value=data_list[_idx], created_at=time) for _idx in range(len(data_list))]

        # Send batch data
        self.aio.send_batch_data(feed.key, data_tuplelist)


if __name__ == '__main__':

    # Enable debug messages
    DEBUG = 1

    # Create IO client
    clientIO = AdafruitClientIO()

    # Test data
    data_dict = dict()
    data_dict['N'] = 70
    data_dict['T'] = 9.0
    data_dict['S'] = 0.2
    data_dict['P'] = 1013.25
    data_dict['H'] = 90
    data_dict['Rssi'] = -93

    # Send data with default metadata attached
    clientIO.send_data_all_feeds(data_dict)


