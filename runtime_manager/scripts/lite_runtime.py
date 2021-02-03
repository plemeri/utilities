#!/usr/bin/env python
import os
import rospy
import yaml
import threading
import time

import std_msgs.msg
from std_msgs.msg import Bool
from decimal import Decimal
from autoware_config_msgs.msg import *
from tablet_socket_msgs.msg import *
from geometry_msgs.msg import *
from autoware_msgs.msg import *

class lite_runtime:
    def __init__(self, stats):
        self.curdir = os.path.abspath(os.path.split(__file__)[0])
        self.configs = yaml.load(open(os.path.join(self.curdir, 'computing.yaml'), 'r'))['params']
        self.configs = [dic for dic in self.configs if 'topic' in dic.keys()]
        self.config_pubs = []

        for config in self.configs:
            pub = rospy.Publisher(config['topic'], eval(config['msg']), latch=True, queue_size=10)
            msg = eval(config['msg'])()
            for var in config['vars']:
                try:
                    setattr(msg, var['name'], var['v'])
                except AttributeError:
                    continue
            # print(msg)
            pub.publish(msg)
            self.config_pubs.append(pub)

        self.status = dict.fromkeys(stats.keys(), False)
        self.status_subs = []

        for key, item in stats.items():
            sub = rospy.Subscriber(item + '_stat', Bool, self.status_callback, callback_args=key)
            self.status_subs.append(sub)

        t = threading.Thread(target=self.status_print)
        t.daemon = True
        t.start()

    def status_print(self):
        while True:
            print('=' * 6 + 'STATUS' + '=' * 6)
            for key, item in self.status.items():
                print(key + ': ' + ('OK' if item else 'ERR'))
            print('=' * 18)
            time.sleep(1)
            os.system('clear')
    
    def status_callback(self, msg, stat):
        self.status[stat] = msg.data
        
    
if __name__ == "__main__":
    rospy.init_node('lite_runtime')
    runtime = lite_runtime({'GNSS': 'gnss', 
                            # 'NDT': 'ndt', 
                            'POINTCLOUD MAP': 'pmap', 
                            'VECTOR MAP': 'vmap', 
                            'Control': 'lf'})
    rospy.spin()