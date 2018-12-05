#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2018 Pradeep.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy
from gnuradio import gr
#import ring_buffer as rb
from threading import Thread
from threading import Lock

BUFF_SIZE = 1024

class usb_source(gr.sync_block):
    """
    docstring for block usb_source
    """
    def __init__(self, device,parity,baudrate,stopbits,bytesize,wait_for_newline):
        gr.sync_block.__init__(self,
            name="usb_source",
            in_sig=None,
            out_sig=[numpy.float32])
        ''' 
        self.mgr = pmt.pmt_mgr()
        for i in range(64):
            self.mgr.set(pmt.pmt_make_blob(10000))        
        '''

        self.device = device
        self.parity = parity
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.wait_for_newline = wait_for_newline
        
        #set parity
        
        if self.parity == NONE:
            self.parity = serial.PARITY_NONE
        elif self.parity == EVEN:
            self.parity = serial.PARITY_EVEN
        else:
            self.parity = serial.PARITY_ODD
        
        if self.stopbits == STOPBITS_ONE:
            self.stopbits = serial.STOPBITS_ONE
        elif self.stopbits == STOPBITS_TWO:
            self.stopbits = serial.STOPBITS_TWO
        
        if self.bytesize == WORD_SIZE_7:
            self.bytesize = serial.SEVENBITS
        else:
            self.bytesize = serial.EIGHTBITS
        
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize
        )
    
        try:
            #write_thread = Thread(target=work_fn, args=(buff, buff_lock, ser))
            read_thread = Thread(target=rx_work, args=(buff,buff_lock, ser))
        except:
            print("failed to create thread")
        
        ## Buffer and lock
        #self.rbuff = rb.ring_buffer(BUFF_SIZE)
        self.buff_lock = Lock()
        self.buff = [None]

        '''
        self.ser = serial.Serial(
            port=self.device,
            baudrate=self.baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        '''
            
        """
        print "Opened: ",self.ser.portstr       # check which port was really used
        self.ser.write("hel;lkfsdsa;lkfjdsaflo\n\r")      # write a string
        #ser.close()             # close port
        """

    ## Thread to read from the serial port and to write to the buff. 
    # reads one line at a time and add to the buff.
    # @buff buffer to read into.
    # @buff_lock lock to serialize buffer access. Should be the lock from "threadding" package.
    # @ser instance of the serial class.
    def rx_work(self, rbuff, rbuff_lock, ser):
        """
        Lock mutex
            read from the serial port. ----?? how much data to be read.
            write the data into buffer. 
            mark underflow or overflow if any. 
        Unlock mutex
        """
        while(1):
            if(ser.is_open == False):
                ## Need to raise a non fatal exception. 
                #raise IOError("Serial port is not open")
                continue
            """    
            if(self.wait_for_newline):
                line = ser.readline()
            else:
                line = ser.read()
            """
            tmp_buff = ser.read(BUFF_SIZE)
            bytes_read = len(buff)

            with self.buff_lock():
                self.buff = tmp_buff[:]
                

    def work(self, input_items, output_items):
        ##
        # Acquire the lock
        # copy buffer and update the nooutput_items.
        # Release the lock 
         
        out = output_items[0]
        with self.buff_lock:
            # copy from buff to output_items
            out[:] = self.buff[:]
        return len(output_items[0])

