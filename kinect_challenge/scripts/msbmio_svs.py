#!/usr/bin/env python

#######################
#
#   Kinect Challenge MSBMIOSvs  (MicroSoft BenchMark IO Service)
#
#   Manages the communications with the Microsoft Benchmark computer for the Kinect Challenge.
#
#
#  Author: Ralph Gnauck
#  License: BSD
#
#
#   Created for the SV ROS entry in the 2014 IROS Microsoft Kinect Challenge Competition
#
#######################
import rospy
from   kinect_challenge.srv import *
from   std_msgs.msg import String
import socket
import struct
import time

MSGLEN = 9 # as defined in Microsoft Benchmark documentation


#################################################
#
#  class: MSBMIOmsg
#
#  Used to format packets to send commands to the Microsoft Benchmark PC
#  
#  Constructor takes two args:
#   lm_id = landmark ID (Byte)
#   run_id = the run ID (Int 64)
#
#  method get_packet returns packed binary form of the massage

class MSBMIOmsg:
    def __init__(self, lm_id=0, run_id=0):
        self._lm_id = lm_id
        self._run_id = run_id
       
    def get_packet(self):
        return struct.pack('<BQ', self._lm_id, self._run_id)

##############################################
#
#  ROS Service Node
#
#   MSBMISSvs
#
#   Handles comms with the Microsoft Benchmark PC 
#
#  Listens for ROS MSBMIO  messages that contain three string fields:
#       sction
#       landmark_id
#       run_id
#
#   Sends a packet to the Microsoft Benchmark and waits for the response
#   TCP/IP coms is established during constructor call using IP address and port.
#   IP address and port of the MSBM PC should be set using launch paramaters host_IP and host_Port
#
#   NOTE: if host_IP is set to the string "sim" the service will simulate the MSBM PC and always return "OK" to any message sent to it.
#
#   action can be:
#       "map":    used during mapping phase, sends landmarkID and runID to MSBM, waits for response and returns appropriate status
#       "start":  used during navigation phase at the start of a leg, sends landmarkID and runID to MSBM, always returns "OK" without waiting for response (MSBM does not send a response for a start message)
#       "end":    used during navigation phase at the end of a leg, sends landmarkID and runID to MSBM, waits for response and returns status
#    
#   See MSBM Documentation for full protocol details
#
#   Returns:
#       "OK" if no error
#       "Fail" if MSBM responds with a fail status
#       "Timeout" if no response in designated time.
#       "Exception" is any exceptions are caught.
#

class MSBMIOSvs:
    
    # Constructor
    def __init__(self, host, port):
        self.sock = None
        self.svs = None
        self.do_sim =False

        # Flag if using simulation mode
        if host.lower()=="sim":
            self.do_sim =True
            rospy.loginfo("Simulating Microsoft Benchmark")
            
        STS_FAIL = 0
        
        self.connect_service()  # establish the ROS Service

        # open the TCP/IP conection to MSBM PC if not in sim mode
        if not self.do_sim:
            self.open_socket()
            self.connect_socket(host, port)

        # spin waiting for messages
        self.svs.spin() 

    # Close the TCP/IP connection     
    def close(self):
        if self.sock <> None:
            self.sock.close()

    # Create the ROS service and register the message callback
    def connect_service(self):
        self.svs = rospy.Service('MSBMIO', MSBMIO, self.handle_msbmio)
        rospy.loginfo( "MSBMIO Service Ready")
 

    # call back for messages sent to the service
    def handle_msbmio(self, req):
        print req
        try:
            action = req.action # get the action
            
            if not self.do_sim:
                self.send(int(req.landmark_id),int(req.run_id))  # send packet to MSBM PC if not in sim mode
            else:
                time.sleep(2) # simulate some handshake time

                
            if action != "start":  
                if self.do_sim:
                    response = 1  # simulate OK response
                else:
                    response = self.receive() # wait for response
            else:
                response = 1  # start command always just returns OK

            # Send back the response
            return MSBMIOResponse(["Fail","OK","Timeout"][response])
           
        except Exception as ex:
            # OOPS Something broke (probably lost TCP/IP connection)
            rospy.loginfo( "Got Exception in svs handler, shutting down. %s" % str(ex))
            self.svs.shutdown('Exception: %s' % str(ex))  # shut down as we will try to recreate the node and reconnect 
            return MSBMIOResponse("Exception")  # return an exception response
            
    # Create TCP/IP socket
    def open_socket(self, sock=None):
        
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    # open TCP/IP connection 
    def connect_socket(self, host, port):
        rospy.loginfo( "Connecting on %s:%d" %(host,port))
        self.sock.connect((host, port))
        rospy.loginfo( "Socket Connected")

    # send message on socket
    def send(self, lm_id=0, run_id=0):
        totalsent = 0
        msg = MSBMIOmsg(lm_id, run_id).get_packet()  # format byte array to send
        sent = self.sock.settimeout(30)   
        
        while totalsent < MSGLEN: # loop till entire message is sent
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    ############################################################
    #
    # read  response from MSBM PC from socket
    # waits for single byte massage on socket or till time out
    # returns:
    #   1 for OK
    #   0 for Fail response from MSBM PC or invalid message
    #   2 for timeout
    # throws exception on any network exception other than timeout
    def receive(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < 1:
            try:
                chunk = self.sock.recv(min(MSGLEN - bytes_recd, 2048))  # read data on socket
                if chunk == '':
                    raise RuntimeError("socket connection broken")
                chunks.append(chunk)
                bytes_recd = bytes_recd + len(chunk)
                
            except socket.timeout:
            
                rospy.loginfo( "Socket receive timeout, returning sts timeout")
                return 2
                
            except : raise
            
        rospy.loginfo("Got Response (%d bytes): sts=%02x" %(bytes_recd,ord(chunks[0])))

        # actually only expecting a single byte message
        if len(chunks) == 1 and chunks[0] <> '\0':
            return 1
        return 0
        
# main function, starts the node, gets parameters and creates the service class instance
# if service throws an RuntimeError exception the existing instance is destroyed and a new instance is created,
# handles disconnection from MSBM and automatic reconnection
if __name__ == '__main__':
    try:
        
        rospy.init_node('MSBMIOSvs')
        
        rospy.loginfo("%s" % sys.argv)

        host=rospy.get_param("~host_IP","169.254.232.242") # needs to be overridden in launch file
        port=int(rospy.get_param("~host_port","7576"))
        

        while (1):  # loop to retry connection if socket broken
            try:
                svs=MSBMIOSvs(host, port) # create and start the service
                svs.close()
                
                rospy.loginfo("Service handler exited, retry connection")
                
            except RuntimeError as ex: # thrown when socket connection broken, restart the service
                svs.close()
                rospy.loginfo("Got Runtime Error in Main. %s " % str(ex))
                
            except Exception as ex: # something broke, give up
                rospy.loginfo("Got exception in main: %s" % str(ex))
                break
        
        rospy.loginfo("MSBMIOSvs exited")
        
    except rospy.ROSInterruptException as ex:
        rospy.loginfo("Got ROS Interrupt exception in main: %s" % str(ex))
        
    
    
    
    
    

    
    
    
