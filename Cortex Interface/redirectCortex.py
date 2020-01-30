#
# How to use the Cortex decoder
# Authors: Jean & Luca
#


import sys
import socket
import struct
import numpy as np

# Import useful classes
from CortexDecoder import CortexDecoder
from CortexDecoder import CortexReader

# Multi-cast address (see SetupCortex.vi)
MCAST_GRP = '225.1.1.1'
# Port number (see Cortex in Tools/Settings/Misc)
MCAST_PORT = 2002



LOCAL_IP = "127.0.0.1"
LOCAL_PORT = 20001
REMOTE_PORT = 20002

if __name__ == '__main__':

    
    
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP

    sock.bind((LOCAL_IP, LOCAL_PORT))

    # Bind communication
    deCortex = CortexReader()

    while True:
        # Read Cortex data
        
        for k in range(0 , deCortex.cortexDecoder.nObjs):

            objs = deCortex.getObjs()
            
            objs[k].computeCenter()

            posVehicle = np.vstack((objs[k].xC,objs[k].yC,objs[k].zC))
            sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP

            sock.bind((LOCAL_IP, LOCAL_PORT))
            t = tuple(posVehicle)
            data=struct.pack('>' + 'd' * len(posVehicle),*t)
            sock.sendto(data, (LOCAL_IP, REMOTE_PORT))
            #sock.close()



























