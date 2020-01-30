#
# How to use the Cortex decoder
# Authors: Jean & Luca
#


import sys
import socket
import struct
import numpy as np
import time

# Import useful classes
from CortexDecoder import CortexDecoder
from CortexDecoder import CortexReader

# Multi-cast address (see SetupCortex.vi)
MCAST_GRP = '225.1.1.1'
# Port number (see Cortex in Tools/Settings/Misc)
MCAST_PORT = 2002


if __name__ == '__main__':

    # Bind communication
    deCortex = CortexReader()

    counter = 0.1

    while True:
        
        # Read Cortex data
        #deCortex.readFromSocket()

        if time.clock() > counter:
            print time.clock()

            counter = counter + 5

            print 'x = '+str(deCortex.pos[0])
            print 'y = '+str(deCortex.pos[1])
            print 'z = '+str(deCortex.pos[2])

            objs = deCortex.getObjs()
                        
            for k in range(0 , deCortex.cortexDecoder.nObjs):
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
    
  
























