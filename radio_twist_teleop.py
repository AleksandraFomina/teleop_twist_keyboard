#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import serial


TwistMsg = Twist

ser = serial.Serial()

moveBindings = {
        'w':(1,0,0,0),
        'a':(0,0,0,1),
        'd':(0,0,0,-1),
        's':(-1,0,0,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'u':(1.1,1),
        'j':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        self.timeout = None

        self.start()

    
    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = TwistMsg()

        while not self.done:
            
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey():
    key = ser.read(1).decode()
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    
    rospy.init_node('teleop_twist_keyboard')
    ser.baudrate = rospy.get_param("~baudrate", 9600)
    ser.port = rospy.get_param("~port", "/dev/ttyUSB0")
    ser.dtr = False
    ser.timeout = 0.5
    ser.open()

    ser.flushInput()

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = 2
    turn_limit = 3
    key_timeout = 0.5
    
    pub_thread = PublishThread()

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.update(x, y, z, th, speed, turn)
        ser.write(vels(speed,turn).encode())
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    ser.write("Linear speed limit reached!".encode())
                if turn == turn_limit:
                    ser.write("Angular speed limit reached!".encode())
                ser.write(vels(speed,turn).encode())
                if (status == 14):
                    ser.write("see instruction".encode())
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)
     

    finally:
        pub_thread.stop()
        


