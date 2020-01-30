#
# How to use the Cortex decoder
# Authors: Jean & Luca
#


import sys
import socket
import struct
import numpy as np
import xlwt

# Import useful classes
from CortexDecoder import CortexDecoder

# Multi-cast address (see SetupCortex.vi)
MCAST_GRP = '225.1.1.1'
# Port number (see Cortex in Tools/Settings/Misc)
MCAST_PORT = 2002


if __name__ == '__main__':

    # Bind communication
    deCortex = CortexDecoder(MCAST_GRP, MCAST_PORT)
    wb = xlwt.Workbook()
    ws = wb.add_sheet('Square Trajectory')
    counter = 0
    while counter < 1000:

        # Read Cortex data
        deCortex.readFromSocket()

        for k in range(0 , deCortex.nObjs):

            print 'Name of object '+str(k)+' '+str(deCortex.objs[k].name)
            #print 'Number of markers in object '+str(k)+': '+str(deCortex.objs[k].nMrks)

            deCortex.objs[k].computeCenter()
            '''
            print 'Center of object '+str(k)+': '
            print str(deCortex.objs[k].xC)
            print str(deCortex.objs[k].yC)
            print str(deCortex.objs[k].zC)
            '''
            if(counter < 5000):
                print 'saving'+str(counter)
                ws.write(counter, 0, deCortex.objs[k].xC)
                ws.write(counter, 1, deCortex.objs[k].yC)
                ws.write(counter, 2, deCortex.objs[k].zC)
                counter=counter+1
            else:
                print 'Data saved'
            '''
            for i in range(0, deCortex.objs[k].nMrks):
                print 'Marker '+str(i)+' of object '+str(k)
                print 'x = '+str(deCortex.objs[k].markers[i].x)
                print 'y = '+str(deCortex.objs[k].markers[i].y)
                print 'z = '+str(deCortex.objs[k].markers[i].z)
            '''

    print 'Data written to file'
    wb.save("Motionanalysis.xls")
