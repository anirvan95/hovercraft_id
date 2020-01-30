####
#
## Classes to decode from Cortex
#   Authors: Jean & Luca
####

import sys
import socket
import struct
import numpy as np
import errno
import threading
import time

class CortexReader(threading.Thread):
    def __init__(self):
        self.pos = np.mat(np.zeros((3,1)))                                            # 3D postion [mm]
        self.visible = 0;                                                       # Variable indicated if the quuadcopter is visible
        self.timestamp = 0                                                      # Timestamp at which the packet is received [ms]
        self.T_CORTEX_LOOP_FULL = 0.0002                                        # Time needed get last position measured [s]
        self.MCAST_GPR = '225.1.1.1'                                            # Parameter for cortex connection
        self.MCAST_PRT = 2002                                                   # Parameter for cortex connection
        self.cortexDecoder = CortexDecoder(self.MCAST_GPR,self.MCAST_PRT)       # Object of type CortexDecoder
        self.t = time.time()                                                    # Current time [s]
        self.startThread = True                                                 # Variable indicating the state of the thread
        threading.Thread.__init__(self)                                         # Initialize the thread
        self.start()                                                            # Start thread

    # Function called when the thread starts
    def run(self):
        # Loop while startThrad is true
        while self.startThread == True:
            self.updateData()

    # Function used to stop the thread
    def stop(self):
        self.startThread = False

    # Function used to update cortex data
    def updateData(self):
        # Read cortex until the buffer is empty => get last image
        while True:
            self.t = time.time() 
            self.cortexDecoder.readFromSocket()
            if (time.time()-self.t >self.T_CORTEX_LOOP_FULL):
                break
        try:
            # Get center position of the tracked object
            self.cortexDecoder.objs[0].computeCenter()
            self.pos[0] = self.cortexDecoder.objs[0].xC
            self.pos[1] = self.cortexDecoder.objs[0].yC
            self.pos[2] = self.cortexDecoder.objs[0].zC
            self.timestamp = round(time.time()*1000)
            self.visible = 1
        except:
            self.visible = 0

    # Function used to get the current data into the corresponding container
    def getData(self,cortexStruct):
        cortexStruct.pos[:] = self.pos[:]
        cortexStruct.visible = self.visible
        cortexStruct.timestamp = self.timestamp

    def getObjs(self):
        return self.cortexDecoder.objs

#####
## Structure for object to track
#####
class crazyObject:
    # Object name
    name = None
	
    # Number of markers
    nMrks = 0
	
    # Markers list
    markers = None
	
    # Number of segments
    nSegs = 0
	
    # Segments list
    segs = None
	
    # Number of dofs
    nDofs = 0
	
    # Dofs list
    dofs = None
	
    # x coor of barycenter of markers
    xC = 0.0
	
    # y coor of barycenter of markers
    yC = 0.0
	
    # z coor of barycenter of markers
    zC = 0.0
    

    # Constructor
    #    def __init__(self):
    
    # Center of markers computation
    def computeCenter(self):
        
        for i in range(0, self.nMrks):
            self.xC += self.markers[i].x
            self.yC += self.markers[i].y
            self.zC += self.markers[i].z
        
        self.xC = self.xC / self.nMrks
        self.yC = self.yC / self.nMrks
        self.zC /= self.nMrks


#####
## Structure Marker
#####
class Marker:
	# Position
	x = 0.0
	y = 0.0
	z = 0.0

#####
## Structure Referential
#####

class Referential:
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    
    def __init__(self,x,y,z,r,p,Y):
        self.x = x
        self.y = y
        self.z = z
        self.roll = r
        self.pitch = p
        self.yaw = Y
    
    def update(self,x,y,z,r,p,Y):
        self.x = x
        self.y = y
        self.z = z
        self.roll = r
        self.pitch = p
        self.yaw = Y

def changeReferential(coordinates,refOrigin,refDest):
    """Allows to compute the coordinates of an object in a new referential"""
    if refOrigin == None:    #if no origin referential is given, we assume it is the master referential
        refOrigin = Referential(0,0,0,0,0,0)
    x = coordinates[0]
    y = coordinates[1]
    z = coordinates[2]
    xShift = refOrigin.x - refDest.x
    yShift = refOrigin.y - refDest.y
    zShift = refOrigin.z - refDest.z
    phi = refOrigin.roll - refDest.roll
    theta = refOrigin.pitch - refDest.pitch
    psi = refOrigin.yaw - refDest.yaw
    xShifted = x + xShift
    yShifted = y + yShift
    zShifted = z + zShift
    # defining the rotation matrix
    r11 = np.cos(phi)*np.cos(psi)-np.cos(theta)*np.sin(phi)*np.sin(psi)
    r12 = -np.cos(psi)*np.sin(phi)-np.cos(phi)*np.cos(theta)*np.sin(psi)
    r13 = np.sin(theta)*np.sin(psi)
    r21 = np.cos(theta)*np.cos(psi)*np.sin(phi) + np.cos(phi)*np.sin(psi)
    r22 = np.cos(phi)*np.cos(phi)*np.cos(psi) - np.sin(phi)*np.sin(psi)
    r23 = -np.cos(psi)*np.sin(phi)
    r31 = np.sin(phi)*np.sin(theta)
    r32 = np.cos(phi)*np.sin(theta)
    r33 = np.cos(theta)
    #print (r11,r12,r13,r21,r22,r23,r31,r32,r33)
    #print xShift,yShift,zShift
    #Computing the new coordinates
    xNew = r11*xShifted + r12*yShifted + r13*zShifted
    yNew = r21*xShifted + r22*yShifted + r23*zShifted
    zNew = r31*xShifted + r32*yShifted + r33*zShifted

    return (xNew,yNew,zNew)


#####
## Structure segment
#####
class Segment:
	# Position
	x = 0.0
	y = 0.0
	z = 0.0
	# Angles
	ph = 0.0
	ps = 0.0
	th = 0.0
	# Length of segment
	leng = 0.0

#####
## Class to get data from Cortex
#####
class CortexDecoder:

    # Multi-cast adress
    mcast_grp = ''
    # Port
    mcast_prt = 0
    # Socket
    cortexSocket = None
    
    # Size of byte (b)
    szB = 1
    # Size of string (s)
    szS = 1
    # Size of unsigned int (I)
    szI = 4
    # Size of float (f)
    szF = 4
    # Size of double (d)
    szD = 8

    # List of objects
    objs = None
    # Number of objects
    nObjs = 0
    
    
    # Constructor from multi-cast address & port
    def __init__(self,mcast_grp,mcast_prt):
        
        self.mcast_grp = mcast_grp
        self.mcast_prt = mcast_prt
        
        # Create new socket and bind communication
        self.cortexSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.cortexSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cortexSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.cortexSocket.bind(('', mcast_prt))
        mreq = struct.pack("4sl", socket.inet_aton(mcast_grp), socket.INADDR_ANY)
        self.cortexSocket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        self.cortexSocket.setblocking(1)
    
    # Read data from socket
    def readFromSocket(self):
        
        szB = self.szB
        szS = self.szS
        szI = self.szI
        szF = self.szF
        szD = self.szD

        try:
            data, addr = self.cortexSocket.recvfrom(1024)
            self.objs = self.getPos(data)
            self.nObjs = len(self.objs)
        
        except socket.error as (code,msg):
            if code != errno.EINTR:
                raise

    # Function to exctract objects from dat
    def getPos(self,dat):
    
        szB = self.szB
        szS = self.szS
        szI = self.szI
        szF = self.szF
        szD = self.szD
        
        # print 'Exctracting data...'
        
        # Bytes 1 to 4 are useless
        dat = dat[4:]
    
        # Frame ID from byte 4 to 8
        fID = struct.unpack('<' + 'I', dat[:szI])
        dat = dat[szI:]
    
        objs = []
        # Number of objects from byte 8 to 12
        nObj = struct.unpack('<' + 'I', dat[:szI])
        dat = dat[szI:]
    
        # Loop over objects
        for n in range(1, nObj[0]+1):
		
            # Create object
            obj = crazyObject()
        
            # Get Object name
            name = []
		
            while True:
            
                cStr = struct.unpack('<' + 's', dat[:szS])
			
                if (cStr[0] == '\x00'):
                    dat = dat[szS:]
                    break
                else:
                    name.append(cStr[0])
                    dat = dat[szS:]
        
            obj.name = name
        
            # Get number of markers
            nMarks = struct.unpack('<' + 'I', dat[:szI])
            dat = dat[szI:]
            obj.nMrks = nMarks[0]
        
            # Loop over markers
            marks = []
            for k in range(1, nMarks[0]+1):
            
                x = struct.unpack('<' + 'f', dat[:szF])
                dat = dat[szF:]
			
                y = struct.unpack('<' + 'f', dat[:szF])
                dat = dat[szF:]
            
                z = struct.unpack('<' + 'f', dat[:szF])
                dat = dat[szF:]
            
                # Instantiate marker and append
                mark = Marker()
                mark.x = x[0]
                mark.y = y[0]
                mark.z = z[0]
            
                marks.append(mark)
        
            obj.markers = marks
        
            # Get number of segments
            nSegs = struct.unpack('<' + 'I', dat[:szI])
            dat = dat[szI:]
            obj.nSegs = nSegs[0]
        
            # Loop over segments (64-bits floating, or 'd')
            segs = []
            for k in range(1, nSegs[0]+1):
            
                # Position
                x = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                y = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                z = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                # Angles
                ph = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                ps = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                th = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                # Length
                l = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                # Instantiate segment and append
                seg = Segment()
                seg.x = x[0]
                seg.y = y[0]
                seg.z = z[0]
                seg.ph = ph[0]
                seg.ps = ps[0]
                seg.th =th[0]
                seg.leng = l[0]
            
                segs.append(seg)
        
            obj.segs = segs
        
            # Get number of dofs
            nDofs = struct.unpack('<' + 'I', dat[:szI])
            dat = dat[szI:]
            obj.nDofs = nDofs[0]
        
            dofs = []
            for k in range(1, nDofs[0]+1):
            
                dof = struct.unpack('<' + 'd', dat[:szD])
                dat = dat[szD:]
            
                # Append dof to dofs
                dofs.append(dof[0])
        
                obj.dofs = dofs
        
            objs.append(obj)
    
        return objs
















