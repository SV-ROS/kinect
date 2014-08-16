#!/usr/bin/env python


import rospy
from kinect_challenge.srv import *
from std_msgs.msg import String
import socket
import struct
import time

MSGLEN =9 # as defined in Microsoft Benchmark documentation

class MSBMIOmsg:
    def __init__(self,lmID=0,runID=0):
        
        self._lmID = lmID
        self._runID = runID
       
    def getPacket(self):
        return struct.pack('<BQ',self._lmID,self._runID)

class MSBMIOSvs:

    def __init__(self,host,port):
        self.sock = None
        self.svs = None
        self.doSim =False
        
        if host.lower()=="sim":
            self.doSim =True
            rospy.loginfo("Simulating Microsoft Benchmark")
            
        STS_FAIL=0
        
        self.connectService()
        
        if not self.doSim:
            self.openSocket()
            self.connectSocket(host,port)
        
        self.svs.spin() 
        
    def close(self):
        if self.sock <> None:
            self.sock.close()
        
    def connectService(self):
        self.svs = rospy.Service('MSBMIO',MSBMIO,self.handleMSBMIO)
        rospy.loginfo( "MSBMIO Service Ready")
 
        
    def handleMSBMIO(self,req):
        print req
        try:
            action = req.Action
            
            if not self.doSim:
                self.send(int(req.LandmarkID),int(req.RunID))
            else:
                time.sleep(2) # simulate some handshake time
                
            if action != "start":
                if self.doSim:
                    response =1
                else:
                    response = self.receive()
            else:
                response =1
                
            return MSBMIOResponse(["Fail","OK","Timeout"][response])
           
        except Exception as ex:
            rospy.loginfo( "Got Exception in svs handler, shutting down. %s" % str(ex))
            self.svs.shutdown('Exception: %s' % str(ex))
            return MSBMIOResponse("Exception")
            
        
    def openSocket(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def connectSocket(self, host, port):
        rospy.loginfo( "Connecting on %s:%d" %(host,port))
        self.sock.connect((host, port))
        rospy.loginfo( "Socket Connected")

    def send(self, lmID=0,runID=0):
        totalsent = 0
        msg = MSBMIOmsg(lmID,runID).getPacket()
        sent = self.sock.settimeout(30)
        
        while totalsent < MSGLEN:
            sent = self.sock.send(msg[totalsent:])
            if sent == 0:
                raise RuntimeError("socket connection broken")
            totalsent = totalsent + sent

    def receive(self):
        chunks = []
        bytes_recd = 0
        while bytes_recd < 1:
            try:
                chunk = self.sock.recv(min(MSGLEN - bytes_recd, 2048))
                if chunk == '':
                    raise RuntimeError("socket connection broken")
                chunks.append(chunk)
                bytes_recd = bytes_recd + len(chunk)
                
            except socket.timeout:
            
                rospy.loginfo( "Socket receive timeout, returning sts timeout")
                return 2
                
            except : raise
            
        rospy.loginfo(  "Got Response (%d bytes): sts=%02x" %(bytes_recd,ord(chunks[0])))
        if len(chunks)==1 and chunks[0] <> '\0':
            return 1
        return 0
        

if __name__ == '__main__':
    try:
        rospy.init_node('MSBMIOSvs')
        
        rospy.loginfo(  "%s" % sys.argv)
        host=rospy.get_param("/MSBMIOSvs/host_IP","192.168.100.99")
        port=int(rospy.get_param("/MSBMIOSvshost_port","7576"))
        
        nargs = len(sys.argv)
        argOffset = 0
        
        if nargs > 1:
            if sys.argv[1].startswith('__name:='):
                argOffset = 2
                
        if nargs > 1 + argOffset:
            host= sys.argv[1+argOffset]
            
        if nargs > 2+argOffset:   
            port= int(sys.argv[2+argOffset])

        while (1):
            try:
                svs=MSBMIOSvs(host,port)
                svs.close()
                
                rospy.loginfo(  "Service handler exited, retry connection")
                
            except RuntimeError as ex: 
                svs.close()
                rospy.loginfo(  "Got Runtime Error in Main. %s " % str(ex))
                
            except Exception as ex: 
                rospy.loginfo(  "Got exception in main: %s" % str(ex))
                break
        
        rospy.loginfo(  "MSBMIOSvs exited")
        
    except rospy.ROSInterruptException as ex:
        rospy.loginfo(  "Got ROS Interrupt exception in main: %s" % str(ex))
        
    
    
    
    
    

    
    
    