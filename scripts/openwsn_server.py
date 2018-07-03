#!/usr/bin/env python

import os
from beginner_tutorials.srv import *
import rospy
import sys
from   coap   import    coap,                            \
                        coapResource,                    \
                        coapDefines         as d        

import signal

class testResource(coapResource.coapResource):
    
    def __init__(self):
        # initialize parent class
        coapResource.coapResource.__init__(
            self,
            path = 'test',
        )
    
    def GET(self,options=[]):
        
        print 'GET received'
        
        respCode        = d.COAP_RC_2_05_CONTENT
        respOptions     = []
        respPayload     = [ord(b) for b in 'hello world 1 2 3 4 5 6 7 8 9 0']
        
        return (respCode,respOptions,respPayload)


def handle_openwsn_server(req):
	if req.accelx == 1:
		p=c.GET('coap://[{0}]/i'.format(req.mote_ip))
		print ''.join([chr(b) for b in p])
		return MimsyIMUResponse('Server Received Message')

def openwsn_server():
	rospy.init_node('openwsn_server')
	s = rospy.Service('openserver1',MimsyIMU,handle_openwsn_server)
	print "handling request"
	rospy.spin()

if __name__ == "__main__":
	
	here = sys.path[0]
        testResource = testResource()
	print here
	sys.path.insert(0,os.path.join(here,'..','..','..','coap'))
	MOTE_IP = 'bbbb::1415:92cc:0:2'
	UDPPORT = 61618 # can't be the port used in OV
	c = coap.coap(udpPort=UDPPORT)
	c.addResource(testResource)

	openwsn_server() 
