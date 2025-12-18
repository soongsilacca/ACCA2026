#! /usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015-2020, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# import rospy
# from std_msgs.msg import String

import base64
import socket
import errno
import time
import threading
import datetime
import select
import serial
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

if (sys.version_info > (3, 0)):
    import queue as Queue
else:
    import Queue as Queue

# Testing
test_mode = False
dummy_gga = '$GPGGA,134451.797,4250.202,N,08320.949,W,1,12,1.0,0.0,M,0.0,M,,*7D'
gga = ''

gga_dict = {
    'UTC Time': datetime.datetime.utcnow(),
    'Lat': 4250.202,
    'N/S': 'N',
    'Lon': 8320.949,
    'E/W': 'W',
    'Fix': 0,
    'Satellites': 0,
    'HDOP': 0,
    'Altitude': 0.0,
    'Altitude Units': 'M',
    }
# ,093022.00,3733.44920,N,12702.78018,E,1,08,1.14,119.5,M,18.6,M,,*48\r\n

# Global variables
rtcm_queue = Queue.Queue()
gga_queue = Queue.Queue()

def parse_latlon(latlon):

    integer = latlon.split('.')[0] 
    decimal = latlon.split('.')[1]

    degree = int(integer[:-2])
    minutes = float(integer[-2:] + '.' + decimal)

    return minutes / 60 + degree

def parse_gga(gga_bytes):
    gga_string = str(gga_bytes, 'utf-8')

    gga_split = gga_string.split(',')

    gga_dict['UTC Time'] = gga_split[1]
    gga_dict['Lat'] = parse_latlon(gga_split[2])
    gga_dict['N/S'] = gga_split[3]
    gga_dict['Lon'] = parse_latlon(gga_split[4])
    gga_dict['E/W'] = gga_split[5]
    gga_dict['Fix'] = int(gga_split[6])
    gga_dict['Satellites'] = int(gga_split[7])
    gga_dict['HDOP'] = float(gga_split[8])
    gga_dict['Altitude'] = float(gga_split[9])
    gga_dict['Altitude Units'] = gga_split[10]

    print(gga_dict)

class NtripSocketThread (threading.Thread):
    def __init__(self, caster_ip, caster_port, mountpoint, useragent):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()
        self.no_rtcm_data_count = 0
        self.sent_gga = False
        self.ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected_to_caster = False
        self.useragent = useragent
        
        self.mountpoint = mountpoint
        self.caster_ip = caster_ip
        self.caster_port = caster_port

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def getGGABytes(self):
        now = datetime.datetime.utcnow()
        ggaString = (
            "GPGGA,%02d%02d%04.2f,3734.087,N,12702.603,E,1,12,1.0,0.0,M,0.0,M,,"
            % (now.hour, now.minute, now.second)
        )

        checksum = self.calcultateCheckSum(ggaString)
        return bytes("$%s*%s\r\n" % (ggaString, checksum), "ascii")

    def connect_to_ntrip_caster(self):
        print('Connecting to NTRIP caster at %s:%d' % (self.caster_ip, self.caster_port))

        try:
            self.ntrip_tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.ntrip_tcp_sock.settimeout(5.0)
            self.ntrip_tcp_sock.connect((self.caster_ip, self.caster_port))
            self.ntrip_tcp_sock.settimeout(None)
            print('Successfully opened socket')
        except Exception as ex:
            print('Error connecting socket: %s' % ex)
            self.ntrip_tcp_sock.settimeout(None)
            return False

        encoded_credentials = base64.b64encode((self.useragent).encode('ascii'))
        if (sys.version_info > (3, 0)):
            server_request = ('GET /%s HTTP/1.0' % self.mountpoint).encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'User-Agent: NTRIP ABC/1.2.3'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Accept: */*'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Connection: close'.encode('utf-8') + b'\x0D' + b'\x0A' \
                            + 'Authorization: Basic '.encode('utf-8') + encoded_credentials + b'\x0D' + b'\x0A' + b'\x0D' + b'\x0A'
        else:
            server_request = 'GET /%s HTTP/1.0\r\nUser-Agent: NTRIP ABC/1.2.3\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic %s\r\n\r\n' % (self.mountpoint, encoded_credentials)

        self.ntrip_tcp_sock.sendall(server_request)

        while True:
            try:
                response = self.ntrip_tcp_sock.recv(10000)
            except socket.error as e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    continue
                else:
                    # a "real" error occurred
                    print(e)
                    return False
            else:
                if ('ICY 200 OK').encode() in response:
                    print('Successfully connected to NTRIP caster')
                    return True
                else:
                    print('Received unexpected response from caster:\n%s' % response)
                    return False

    def run(self):
        print('Starting NTRIP TCP socket thread')
        while not self.stop_event.isSet():

            if not self.connected_to_caster:
                if self.connect_to_ntrip_caster():
                    self.connected_to_caster = True
                else:
                    time.sleep(0.05)
                    continue

            # Receive RTCM messages from NTRIP caster and put in queue to send to GPS receiver
            try:
                ready_to_read, ready_to_write, in_error = select.select([self.ntrip_tcp_sock, ], [self.ntrip_tcp_sock, ], [], 5)
            except select.error:
                self.ntrip_tcp_sock.close()
                self.connected_to_caster = False
                print('Error calling select(): resetting connection to NTRIP caster')
                continue

            if len(ready_to_read) > 0: # RTCM Parsing
                rtcm_msg = self.ntrip_tcp_sock.recv(100000)
                print(type(rtcm_msg))
                if len(rtcm_msg) > 0:
                    if (sys.version_info > (3, 0)):
                        if rtcm_msg[0] == 211:
                            rtcm_msg_len = 256 * rtcm_msg[1] + rtcm_msg[2]
                            rtcm_msg_no = (256 * rtcm_msg[3] + rtcm_msg[4]) / 16
                            print('Received RTCM message %d with length %d' % (rtcm_msg_no, rtcm_msg_len))
                    else:
                        if ord(rtcm_msg[0]) == 211:
                            rtcm_msg_len = 256 * ord(rtcm_msg[1]) + ord(rtcm_msg[2])
                            rtcm_msg_no = (256 * ord(rtcm_msg[3]) + ord(rtcm_msg[4])) / 16
                            print('Received RTCM message %d with length %d' % (rtcm_msg_no, rtcm_msg_len))

                    rtcm_queue.put(rtcm_msg)
                    # print(list(rtcm_msg))
                    self.no_rtcm_data_count = 0

            # Get GPGGA messages from receive queue and send
            # to NTRIP server to keep connection alive
            if len(ready_to_write) > 0:
                try:
                    # gga_msg = gga_queue.get_nowait()
                    gga_msg = self.getGGABytes()
                    print('Sending new GGA message to NTRIP caster %s' % gga_msg)
                    # self.ntrip_tcp_sock.send(gga_msg.encode('utf-8'))
                    self.ntrip_tcp_sock.send(gga_msg)
                    self.sent_gga = True
                except Queue.Empty:
                    pass

            if self.no_rtcm_data_count > 200:
                print('No RTCM messages for 10 seconds; resetting connection to NTRIP caster')
                self.ntrip_tcp_sock.close()
                self.connected_to_caster = False
                self.no_rtcm_data_count = 0

            if self.sent_gga:
                self.no_rtcm_data_count += 1

            time.sleep(0.05)

        print('Stopping NTRIP TCP socket thread')
        self.ntrip_tcp_sock.close()

    def stop(self):
        self.stop_event.set()


class SerialThread(threading.Thread):

    def __init__(self, serial_port, baud_rate):
        threading.Thread.__init__(self)
        self.stop_event = threading.Event()


        self.serial_port = serial_port
        self.baud_rate = baud_rate

        self._ser = serial.Serial(self.serial_port, self.baud_rate, timeout=10)
        if self._ser.name != self.serial_port:
            raise Exception("Couldn't find Device!")

    def run(self):
        print('Starting Serial Read/Write thread')
        print('Writing RTCM message to %s:%d' % (self.serial_port, self.baud_rate))
        while not self.stop_event.isSet():
            try:
                # rtcm_msg = rtcm_queue.get_nowait()
                # print(rtcm_msg)
                # self._ser.write(rtcm_msg)
                while True:
                    line = self._ser.readline()
                    if line.find(b'$GNGGA') > 0:
                        parse_gga(line.split(b'$GNGGA')[-1])
                        # print("readline : ", line.split(b'$GNGGA')[-1])
                        break
            except Queue.Empty:
                # Nothing in the RTCM message queue this time
                pass

    def stop(self):
        self.stop_event.set()

def stop_threads(workers):
    for worker in workers:
        worker.stop()
        worker.join()

def start_threads(caster_ip, caster_port, mountpoint, useragent, serial_port, baud_rate):
    
    if serial_port:
        # workers = [NtripSocketThread(caster_ip, caster_port, mountpoint, useragent), SerialThread(serial_port, baud_rate)]
        workers = [SerialThread(serial_port, baud_rate)]
    else:
        print("No broadcast_port")
        workers = [NtripSocketThread(caster_ip, caster_port, mountpoint, useragent)]

    for worker in workers:
        print("=== worker.started ===")
        worker.start()
    return workers

class SocketNtrip(Node):

    def __init__(self):
        super().__init__('socket_ntrip')

        self.declare_parameter('rtcm_topic', '/rtcm')
        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.get_logger().info(f"rtcm_topic : {self.rtcm_topic}")

        self.declare_parameter('target_host', 'RTS2.ngii.go.kr')
        self.target_host = self.get_parameter('target_host').value
        self.get_logger().info(f"target_host : {self.target_host}")
        
        self.declare_parameter('target_port', 2101)
        self.target_port = self.get_parameter('target_port').value
        self.get_logger().info(f"target_port : {self.target_port}")

        self.declare_parameter('mountpoint', 'VRS-RTCM32')
        self.mountpoint = self.get_parameter('mountpoint').value
        self.get_logger().info(f"mountpoint : {self.mountpoint}")

        self.declare_parameter('useragent', 'Ntrip ublox')
        self.useragent = self.get_parameter('useragent').value
        self.get_logger().info(f"useragent : {self.useragent}")

        self.declare_parameter('user', 'tge1375@naver.com:gnss')
        self.user = self.get_parameter('user').value
        self.get_logger().info(f"user : {self.user}")

        self.declare_parameter('verbose', True)
        self.verbose = self.get_parameter('verbose').value
        self.get_logger().info(f"verbose : {self.verbose}")

        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.serial_port = self.get_parameter('serial_port').value
        self.get_logger().info(f"serial_port : {self.serial_port}")

        self.declare_parameter('baud_rate', 119200)
        self.baud_rate = self.get_parameter('baud_rate').value
        self.get_logger().info(f"baud_rate : {self.baud_rate}")

        self.workers = start_threads(
            self.target_host, 
            self.target_port, 
            self.mountpoint, 
            self.useragent, 
            self.serial_port,
            self.baud_rate
        )

        # timer_period = 1  # seconds
        # self.timer = self.create_timer(timer_period, self.publishRTCM)

    def __del__(self):
        self.get_logger().warn('Shutting down')
        stop_threads(self.workers)

def main(args=None):
    rclpy.init(args=args)

    socket_ntrip_node = SocketNtrip()
    rclpy.spin(socket_ntrip_node)
    socket_ntrip_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':

    main()

    # caster_ip = 'RTS2.ngii.go.kr'
    # caster_port = 2101

    # mountpoint = 'VRS-RTCM32'
    # username = 'tge1375@naver.com'
    # password = 'gnss'

    # workers = start_threads(caster_ip, caster_port, mountpoint, \
    #     username, password, None, None)

    # start = time.time()
    # stop = time.time()
    # duration = stop - start

    # while duration < 30:

    #     time.sleep(0.1)

    #     if duration % 5 == 0:
    #         print("The time of the run:", duration)
    #     duration = time.time() - start
        
    # stop_threads(workers)



    # ros_interface = RosInterface()
    # rospy.on_shutdown(ros_interface.on_shutdown)

    # rospy.spin()