#!/usr/bin/env python3

import sys
import time
import socket
import datetime

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from rtcm_msgs.msg import Message


# 전역
factor = 2  # How much the sleep time increases with each failed attempt
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 1  # So the first one is 1 second
V2 = True

"""method called order

1. ntripConnection
2. createSocket success => getMountPointBytes then send 
    => casterRequest =>  getGGABytes => sendall
   else => handleClientException

- publishRTCM periodically (currently every 1 seconds)
- retrySocket if Exception during publish

"""

"""Retry Logic & Examples
TODO
"""

class SocketNtrip(Node):

    def __init__(self):
        super().__init__('socket_ntrip')

        self.declare_parameter('rtcm_topic', '/rtcm')
        self.rtcm_topic = self.get_parameter('rtcm_topic').value
        self.get_logger().info(f"rtcm_topic : {self.rtcm_topic}")

        self.declare_parameter('target_host', 'RTS2.ngii.go.kr')
        self.target_host = self.get_parameter('target_host').value
        self.get_logger().info(f"target_host : {self.target_host}")
        
        self.declare_parameter('target_port', '2101')
        self.target_port = self.get_parameter('target_port').value
        self.get_logger().info(f"target_port : {self.target_port}")

        self.declare_parameter('mountpoint', 'VRS-RTCM32')
        self.mountpoint = self.get_parameter('mountpoint').value
        self.get_logger().info(f"mountpoint : {self.mountpoint}")

        self.declare_parameter('useragent', 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/95.0.4638.69 Safari/537.36')
        self.useragent = self.get_parameter('useragent').value
        self.get_logger().info(f"useragent : {self.useragent}")

        self.declare_parameter('user', 'tge1375@naver.com:ngii')
        self.user = self.get_parameter('user').value
        self.get_logger().info(f"user : {self.user}")

        self.declare_parameter('verbose', True)
        self.verbose = self.get_parameter('verbose').value
        self.get_logger().info(f"verbose : {self.verbose}")

        self.force_stop = False
        self.ssl = False

        self.maxConnectTime = 0
        self.found_header = False

        self.reconnectTry = 1
        self.sleepTime = 1
        self.RTCM_ARR = []
        self.data = "Initial data"

        # Socket Client
        self.client = None

        self.publisher = self.create_publisher(Message, self.rtcm_topic, 10)
        self.pub_msg = Message()

        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.publishRTCM)

        # Create Fisrt Connection!!
        self.ntripConnection()

    def calcultateCheckSum(self, stringToCheck):
        xsum_calc = 0
        for char in stringToCheck:
            xsum_calc = xsum_calc ^ ord(char)
        return "%02X" % xsum_calc

    def getGGABytes(self):
        now = datetime.datetime.utcnow()
        ggaString = (
            "GPGGA,%02d%02d%04.2f,3729.778,N,12657.412,E,1,12,1.0,0.0,M,0.0,M,,"
            % (now.hour, now.minute, now.second)
        )

        checksum = self.calcultateCheckSum(ggaString)
        print(checksum)
        if self.verbose:
            self.get_logger().info("$%s*%s\r\n" % (ggaString, checksum))
        return bytes("$%s*%s\r\n" % (ggaString, checksum), "ascii")

    def getMountPointBytes(self):
        mountPointString = (
            "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAccept: */*\r\nAuthorization: Basic %s\r\nConnection: close\r\n"
            % (self.mountpoint, self.useragent, self.user)
        )

        # if self.host or self.V2:
        #    hostString = "Host: %s:%i\r\n" % (self.caster,self.port)
        #    mountPointString += hostString

        if V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"

        if self.verbose:
            self.get_logger().info(mountPointString)
        return mountPointString.encode()

    def createSocket(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if self.ssl:
            self.client = self.ssl.wrap_socket(self.client)

        self.get_logger().info(f"{self.target_host} {self.target_port}")

        error_indicator = self.client.connect_ex(
            (self.target_host, int(self.target_port))
        )

        return error_indicator

    def retrySocket(self):
        self.force_stop = False
        self.ssl = False

        self.maxConnectTime = 0
        self.found_header = False

        self.reconnectTry = 1
        self.sleepTime = 1
        self.RTCM_ARR = []
        self.data = "Initial data"
        if self.client:
            self.client.close()

        # Socket Client Reset
        self.client = None
        self.pub_msg = Message()

        self.ntripConnection()

    def handleClientException(self, error_indicator):
        self.client = None

        if self.verbose:
            self.get_logger().info("Error indicator: {}".format( error_indicator))

        if self.reconnectTry < maxReconnect:
            sys.stderr.write(
                "%s No Connection to NtripCaster.  Trying again in %i seconds\n"
                % (datetime.datetime.now(), sleepTime)
            )

            time.sleep(sleepTime)

            sleepTime *= factor
            if sleepTime > maxReconnectTime:
                sleepTime = maxReconnectTime

        self.reconnectTry += 1

    def ntripConnection(self):
        if self.maxConnectTime > 0:
            self.EndConnect = datetime.timedelta(seconds=self.maxConnectTime)
        try:
            while self.reconnectTry <= maxReconnect:
                self.found_header = False
                if self.verbose:
                    self.get_logger().warn("Connection {0} of {1}\n".format(self.reconnectTry, maxReconnect))

                error_indicator = self.createSocket()

                if error_indicator == 0:
                    self.get_logger().warn("Socket Creation Succeed")

                    self.sleepTime = 1
                    self.connectTime = datetime.datetime.now()

                    self.client.settimeout(10)
                    self.client.send(self.getMountPointBytes())

                    self.casterRequest()
                    self.reconnectTry += 1
                else:
                    self.handleClientException(error_indicator)
        except KeyboardInterrupt:
            if self.client:
                self.client.close()
            sys.exit()

    def casterRequest(self):
        while not self.found_header:
            casterResponse = self.client.recv(4096)  # All the data
            header_lines = casterResponse.decode("utf-8").split("\r\n")

            if self.verbose:
                self.get_logger().info("".join(header_lines))

            for line in header_lines:
                if line == "":
                    if not self.found_header:
                        self.found_header = True
                        if self.verbose:
                            self.get_logger().info("End Of Header" + "\n")
                else:
                    if self.verbose:
                        self.get_logger().info("Header: " + line + "\n")

            request_output = None
            for line in header_lines:
                if line.find("SOURCETABLE") >= 0:
                    sys.stderr.write("Mount point does not exist")
                    sys.exit(1)
                elif line.find("401 Unauthorized") >= 0:
                    sys.stderr.write("Unauthorized request\n")
                    sys.exit(1)
                elif line.find("404 Not Found") >= 0:
                    sys.stderr.write("Mount Point does not exist\n")
                    sys.exit(2)
                elif line.find("ICY 200 OK") >= 0:
                    # Valid case
                    self.get_logger().warn("Valid Request : ICY 200 OK")
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.0 200 OK") >= 0:
                    # Valid case
                    self.get_logger().warn("Valid Request : HTTP/1.0 200 OK")
                    self.client.sendall(self.getGGABytes())
                elif line.find("HTTP/1.1 200 OK") >= 0:
                    # Valid case
                    self.get_logger().warn("Valid Request : HTTP/1.1 200 OK")
                    self.client.sendall(self.getGGABytes())

    def publishRTCM(self):
        self.RTCM_ARR = []
        while self.data:
            try:
                if self.client == None:
                    continue
                
                

                self.data = self.client.recv(1)
                
                if ord(self.data) == 211:
                    new_data = self.client.recv(2)
                    if len(new_data) < 2:
                        continue
                    self.RTCM_ARR.append((self.data[0]))
                    
                    self.RTCM_ARR.append((new_data[0]))
                    self.RTCM_ARR.append((new_data[1]))
                    
                    cnt =(new_data[0]*256) +  ((new_data[1]))
                    new_data = self.client.recv(2)
                    self.RTCM_ARR.append((new_data[0]))
                    self.RTCM_ARR.append((new_data[1]))
                    
                    typ = ((new_data[0]*256) + ((new_data[1])))/16
                    print(cnt, typ)
                    cnt=cnt+1
                    '''
                    if cnt > 255:  # Just Abstract Value
                        self.get_logger().warn("Over Msg Error, Ingnore Current Packet")
                        self.RTCM_ARR = []
                        continue
                        '''
                    for x in range(cnt):
                        new_data = self.client.recv(1)
                        self.RTCM_ARR.append((new_data[0]))
                  
                    
                        
                    '''

                    new_data = self.client.recv(2)
                    #print(new_data, len(new_data), new_data[0], new_data[1])
                    if len(new_data) < 2:
                        continue
                    

                    self.RTCM_ARR.append(new_data[0])
                    self.RTCM_ARR.append(new_data[1])

                    cnt = new_data[0] * 256 + new_data[1]
                    typ = (new_data[0] * 256 + new_data[1]) / 16
                    
                    cnt = cnt + 1
                    # self.get_logger().info("Msg Length : %d" % cnt)

                    if cnt > 255:  # Just Abstract Value
                        self.get_logger().warn("Over Msg Error, Ingnore Current Packet")
                        self.RTCM_ARR = []
                        continue

                    for i in range(cnt):
                        new_data = self.client.recv(1)

                        self.RTCM_ARR.append(ord(new_data))

                    if self.force_stop:
                        return
'''
                    #self.get_logger().info("Final Array prepared, lenght : %d " % len(self.RTCM_ARR))
                    self.pub_msg.message = (self.RTCM_ARR)

                    self.pub_msg.header.frame_id = ""
                    self.pub_msg.header.stamp = self.get_clock().now().to_msg()
                    print(self.RTCM_ARR)
                    

                    self.publisher.publish(self.pub_msg)

                    self.RTCM_ARR = []

                if self.maxConnectTime:
                    if datetime.datetime.now() > self.connectTime + self.EndConnect:
                        if self.verbose:
                            self.get_logger().error("Connection Time exceeded\n")
                        sys.exit(0)

            except KeyboardInterrupt:
                if self.client:
                    self.get_logger().error("KeyboardInterrupt\n")
                    self.client.close()
                sys.exit()

            # # TODO : TypeError: catching classes that do not inherit from BaseException is not allowed
            # except self.client.timeout:
            #     if self.verbose:
            #         self.get_logger().error("Connection TimedOut\n")
            #     self.data = False
            # except self.client.error:
            #     if self.verbose:
            #         self.get_logger().error("Connection Error\n")
            #     self.data = False
            # except IndexError:
            #     self.RTCM_ARR = []
            #     pass
            except Exception as e:
                print(e)
                self.retrySocket()
                if e == "timed out":
                    print("timed out")
                pass

def main(args=None):
    rclpy.init(args=args)

    socket_ntrip_node = SocketNtrip()
    rclpy.spin(socket_ntrip_node)
    socket_ntrip_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
