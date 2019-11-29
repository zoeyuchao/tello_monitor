import cv2
import numpy as np 
import threading
import time
from math import *

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


tello_state = None
tello_state_lock=threading.Lock()    
img = None
img_lock=threading.Lock()
command = None
command_lock=threading.Lock()
#info = None
#info_lock=threading.Lock()

start_yaw = None

def draw_angle(img, angle, color, width):
    l = 20
    cv2.line(img, (440, 30), (int(440+l*sin(angle/180.0*pi)), 
    int(30-l*cos(angle/180.0*pi))), color, width)
    cv2.circle(img, (440, 30), l, color, width-1)
    return img


def draw_roll(img, angle, color, width):
    l = 20
    cv2.line(img, (440, 80), (int(440+l*cos(angle/180.0*pi)), 
    int(85-l*sin(angle/180.0*pi))), color, width)
    cv2.line(img, (440, 80), (int(440-l*cos(angle/180.0*pi)), 
    int(75+l*sin(angle/180.0*pi))), color, width)

    cv2.line(img, (440, 80), (int(440+l*cos(angle/180.0*pi)), 
    int(75-l*sin(angle/180.0*pi))), color, width)
    cv2.line(img, (440, 80), (int(440-l*cos(angle/180.0*pi)), 
    int(85+l*sin(angle/180.0*pi))), color, width)
    return img


def draw_pitch(img, angle):
    if (angle > 10):
        color = (0, 0, 255)
    elif (angle < -10):
        color = (100, 255, 0)
    else:
        color = (255, 255, 255)
    
    cv2.circle(img, (440, 130), 20, color, -1)
    return img


def parse_info():
    global info, info_lock
    info_lock.acquire()
    infostr = info.split(';')
    # print(infostr)
    dict={}
    dict['time'] = float(infostr[0])
    dict['battery'] = int(infostr[1])
    dict['speed'] = float(infostr[2])
    info_lock.release()
    return dict

def parse_state():
    global tello_state,tello_state_lock, start_yaw
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    #print (statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['mpry'] = [int(mpry[0]),int(mpry[1]),int(mpry[2])]
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
        elif 'pitch:' in item:
            pitch = int(item.split(':')[-1])
            dict['pitch'] = pitch
        elif 'roll:' in item:
            roll = int(item.split(':')[-1])
            dict['roll'] = roll
        elif 'yaw:' in item:
            yaw = int(item.split(':')[-1])
            if (start_yaw is None):
                start_yaw = yaw
            dict['yaw'] = yaw - start_yaw
    tello_state_lock.release()
    return dict


class TelloMonitor(object):
    def __init__(self):
        rospy.Subscriber("tello_state",String,self.update_state)
        rospy.Subscriber("tello_img",Image, self.update_img)
        rospy.Subscriber("command",String, self.update_command)
        #rospy.Subscriber("tello_info",String, self.update_info)
        con_thread = threading.Thread(target=rospy.spin)
        con_thread.start()
        return

    def update_info(self, data):
        global info, info_lock
        info_lock.acquire()
        info = data.data
        info_lock.release()

    
    def update_command(self, data):
        global command, command_lock
        command_lock.acquire()
        command = data.data
        command_lock.release()


    def update_state(self,data):
        global tello_state,tello_state_lock
        tello_state_lock.acquire()
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)


    def update_img(self,data):
        global img,img_lock
        global tello_state,tello_state_lock
        img_lock.acquire()
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding="passthrough")
        img_lock.release()


    def show(self):
        global img,img_lock, command, command_lock
        img_mode = 0 # normal
        try:
            while not rospy.is_shutdown():
                command_lock.acquire()
                command_data = command
                command_lock.release()
                state = parse_state()
                #info = parse_info()
                # print(info)
                #info = {'time': 10.0, 'battery':20, 'speed': 2}
                #state = {'x':-1, 'y':-1, 'z':-1, 'mid':-1, 'yaw': 20, 'roll': 10, 'pitch': 20}
                #img = cv2.imread('fire_detected.jpg')
                img_lock.acquire()
                #print('img')
                if (img_mode == 0):
                    state_img = img
                elif (img_mode == 1):
                    b, g, r = cv2.split(img)
                    red_image = np.array(r / 255.0 * ((255.0 - b) /255.0) ** 2 * ((255.0 - g) /255.0) ** 2.0 * 255.0, dtype=np.uint8)
                    state_img = red_image

                img_lock.release()
                
                state_img = cv2.resize(state_img, (480, 360), cv2.INTER_AREA)
                font = cv2.FONT_HERSHEY_SIMPLEX
                state_img = cv2.putText(state_img, "mid: {mid}".format(**state), (20, 20), font, 0.6, (100,0,255), 2)
                state_img = cv2.putText(state_img, "pos: ({x}, {y}, {z})".format(**state), (20, 45), font, 0.6, (255,0,0), 2)
                #state_img = cv2.putText(state_img, "last command: {}".format(command_data), (20, 345), font, 0.6, (150,255,0), 2)
                #state_img = cv2.putText(state_img, "{:2.2f} {:.1f}s".format(info['speed'], info['time']), (346, 45), font, 0.6, (150,0,255), 2)
                #state_img = cv2.putText(state_img, "{}%".format(info['battery']), (414, 20), font, 0.6, (150,150,255), 2)
                cv2.line(state_img, (178, 200), (100, 335), (255,255,255), 4)
                cv2.line(state_img, (302, 200), (380, 335), (255,255,255), 4)
                
                state_img = draw_angle(state_img, int(state['yaw']), (255, 0, 255), 3)
                state_img = draw_roll(state_img, int(state['roll']), (255, 255, 0), 3)
                state_img = draw_pitch(state_img, int(state['pitch']))

                state_img = cv2.resize(state_img, (480*2, 360*2), cv2.INTER_AREA)
                cv2.rectangle(state_img, (20, 300), (220, 700), (0,0,0),-1)
                if (state['x'] < 800 and state['x'] > 0
                and state['y'] < 400 and state['y'] > 0):
                    cv2.circle(state_img, (int(20+state['y']/2), int(700-state['x']/2)), 4, (255, 255, 255), -1)

                '''
                yaw = state['yaw'] / 180.0 * pi
                pitch = state['pitch'] / 180.0 * pi
                roll = state['roll'] / 180.0 * pi
                x1 = 420+int(50*(cos (yaw) * cos (roll)))
                y1 = 300+int(50*(cos (pitch) * sin (roll) + cos (roll) * sin (pitch) * sin (yaw)))

                x2 = 420+int(50*(-cos (yaw) * sin (roll)))
                y2 = 300+int(50*(cos (pitch) * cos (roll) - sin (pitch) * sin (yaw) * sin (roll)))

                x3 = 420+int(50*(sin (yaw)))
                y3 = 300+int(50*(-cos(yaw) * sin (pitch)))

                cv2.line(state_img, (x1, y1), (x2, y2), (255,0,0), 2)
                cv2.line(state_img, (x1, y1), (x3, y3), (255,255,0), 2)
                cv2.line(state_img, (x3, y3), (x2, y2), (255,0,255), 2)
                '''
                if (state_img is not None and state_img.shape[0] > 0):
                    cv2.imshow("monitor", state_img)
                key = cv2.waitKey(30)
                if key&0xFF == ord('r'):
                    if (img_mode != 1):
                        img_mode = 1
                    else:
                        img_mode = 0
                elif key&0xFF == ord('s'):
                    cv2.imwrite('data/output/'+str(time.strftime('%H:%M:%S'))+'.jpg', img)
        except rospy.ROSInterruptException:
            pass
            


if __name__ == '__main__':
    rospy.init_node('monitor_node', anonymous=True)
    monitor = TelloMonitor()
    time.sleep(2)
    #print('start')
    monitor.show()