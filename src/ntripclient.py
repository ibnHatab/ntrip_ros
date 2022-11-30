#!/usr/bin/python

import subprocess
import signal
import datetime

import rospy
from pyrtcm import RTCMReader
from mavros_msgs.msg import RTCM


class NtripConnect:

    def __init__(self):

        try:
            self.rtcm_topic = rospy.get_param('~rtcm_topic', '/rtcm')
            self.ntrip_client = rospy.get_param('~ntrip_client', 'ntripclient')
            self.ntrip_server = rospy.get_param('~ntrip_server', 'rtk2go.com')
            self.ntrip_port = rospy.get_param('~ntrip_port', '2101')
            self.ntrip_user = rospy.get_param('~ntrip_user', 'user')
            self.ntrip_pass = rospy.get_param('~ntrip_pass', 'password')
            self.ntrip_stream = rospy.get_param('~ntrip_stream', 'NEAR3')
            self.nmea_gga = rospy.get_param('~nmea_gga', '$GPGGA,')
        except KeyError as ex:
            rospy.logerr(ex)
            raise

        self.pub = rospy.Publisher(self.rtcm_topic, RTCM, queue_size=10)

        self.p = None
        rospy.on_shutdown(self.switch_off)

    def switch_off(self):
        if self.p:
            self.p.send_signal(signal.SIGTERM)
            self.p = None
            rospy.sleep(3)


    def run(self):

        now = datetime.datetime.utcnow()
        now_str = '%02d%02d%04.2f' % (now.hour, now.minute, now.second)
        nmeadata = self.nmea_gga.split(',')
        nmeadata[1] = now_str   # update time
        nmeadata = ','.join(nmeadata[:-1])+','  # strip checksum
        if nmeadata.startswith('$'):
            nmeadata = nmeadata[1:]

        csum = 0
        for c in nmeadata:
            # XOR'ing value of csum against the next char in line
            # and storing the new XOR value in csum
            if ord(c)!=',': csum ^= ord(c)
        #convert hex characters to upper case
        csum = hex(csum).upper()
        #add 0x0 if checksum value is less than 0x10
        if len(csum)==3:
            csum='0'+csum[2]
        else:
            csum=csum[2:4]

        nmeastring = '$'+nmeadata+'*'+csum
        command = "%s -s %s -r %s -u %s -p %s -m %s -n '%s' -M 4" % \
            (
                self.ntrip_client,
                self.ntrip_server,
                self.ntrip_port,
                self.ntrip_user,
                self.ntrip_pass,
                self.ntrip_stream,
                nmeastring,
            )

        self.p = subprocess.Popen(command,
                                  stdin=subprocess.PIPE,
                                  stdout=subprocess.PIPE,
                                  shell=True,
                                  executable='/bin/bash')
        print(command)

        rmsg = RTCM()
        r = rospy.Rate(10)
        rtr = RTCMReader(self.p.stdout)
        for (raw_data, parsed_data) in rtr.iterate():
            # print(parsed_data)
            rmsg.header.seq += 1
            rmsg.header.stamp = rospy.get_rostime()
            rmsg.data = raw_data
            self.pub.publish(rmsg)

            if rospy.is_shutdown():
                break
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('ntrip_connect_node')
    try:
        client = NtripConnect()
        # Wait for shutdown signal to close rosbag record
        client.run()
    except rospy.ROSInterruptException:
        pass
