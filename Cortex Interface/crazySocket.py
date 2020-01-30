#
# To get data from Cortex and create objects
#     Authors: Jean & Luca
#

import sys
import socket
import struct
import numpy as np

# Multi-cast address (see SetupCortex.vi)
MCAST_GRP = '225.1.1.1'
# Port number (see Cortex in Tools/Settings/Misc)
MCAST_PORT = 2002
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


# Structure for object to track
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






# Structure Marker
class Marker:
	# Position
	x = 0.0
	y = 0.0
	z = 0.0


# Structure segment
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



# Function to exctract objects from dat
def getPos(dat):
    
    print 'Exctracting data...'
    
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
            
            cStr = struct.unpack('<' + 's', dat[0])
			
            if (cStr[0] == '\x00'):
                dat = dat[1:]
                break
            else:
                name.append(cStr[0])
                dat = dat[1:]

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




if __name__ == '__main__':
    
    # Create new socket and bind communication
    crazySock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    crazySock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    crazySock.bind(('', MCAST_PORT))
    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    crazySock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    
    # Loop to extract positions of objects from socket
    while True:
        
        data, addr = crazySock.recvfrom(1024)

        # Exctract objects from Cortex data
        objs = getPos(data)

        nObjs = len(objs)
        
        for k in range(0 , nObjs):
            print 'Name of object '+str(k)+' '+str(objs[k].name)
            print 'Number of markers in object '+str(k)+': '+str(objs[k].nMrks)

            objs[k].computeCenter()
            print 'Center of object '+str(k)+': '
            print str(objs[k].xC)
            print str(objs[k].yC)
            print str(objs[k].zC)
            
            
            for i in range(0, objs[k].nMrks):
                print 'Marker '+str(i)+' of object '+str(k)
                print 'x = '+str(objs[k].markers[i].x)
                print 'y = '+str(objs[k].markers[i].y)
                print 'z = '+str(objs[k].markers[i].z)




        #data = data[4:]
        
        #frameID = struct.unpack('<' + 'I', data[:szI])
        #data = data[szI:]
        #print 'Frame ID:'+str(frameID)
        
        #numObj = struct.unpack('<' + 'I', data[:szI])
        #data = data[szI:]
        #print 'Number of objects:'+str(numObj)
        
        #unp_2 = struct.unpack('<' + 's'*5, data[:5])
        #data = data[5:]
        #print 'Object name:'+str(unp_2)
        
        #crap = struct.unpack('<' + 's', data[0])
        #data = data[1:]
        #print (crap[0] == '\x00')
        
        #numMarkers = struct.unpack('<' + 'I', data[:szI])
        #data = data[szI:]
        #print 'Num markers:'+str(numMarkers)
        
        # Marker 1
        #x_1 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'x_1='+str(x_1)
        
        #y_1 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'y_1='+str(y_1)
        
        #z_1 = struct.unpack('<' + 'f',data[:szF])
        #data = data[szF:]
        #print 'z_1='+str(z_1)
        
        # Marker 2
        #x_2 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'x_2='+str(x_2)
        
        #y_2 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'y_2='+str(y_2)
        
        #z_2 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'z_2='+str(z_2)
        
        # Marker 3
        #x_3 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'x_3='+str(x_3)
        
        #y_3 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'y_3='+str(y_3)
        
        #z_3 = struct.unpack('<' + 'f', data[:szF])
        #data = data[szF:]
        #print 'z_3='+str(z_3)
        
        #numSegs = struct.unpack('<' + 'I', data[:szI])
        #data = data[szI:]
        #print 'Number of segments='+str(numSegs)
        
        #numDofs = struct.unpack('<' + 'I', data[:szI])
        #data = data[szI:]
        #print 'Number of dofs='+str(numDofs)
















