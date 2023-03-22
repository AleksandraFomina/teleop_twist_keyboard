#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

import serial

ser = serial.Serial()

#Словарь "движений" в зависимости от нажатой клавиши: w - вперед, a - влево,  s - назад, d - вправо
moveBindings = {
        'w':(1,0,0),
        'a':(0,0,1),
        'd':(0,0,-1),
        's':(-1,0,0),
    }

#Словарь коэффициентов скоростей - q/z - для линейной скорости и скорости поворота,  u/j -для линейной, e/c - для скорости поворота
#Если скорость домножается на коэффициент 1.1, она увеличивается на 10%, если на 0.9 - уменьшается на 10%
speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'u':(1.1,1),
        'j':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishMove():
    def __init__(self):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0

    
    def update(self, x, y, th, speed, turn):
        self.x = x
        self.y = y
        self.th = th
        self.speed = speed
        self.turn = turn

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0)
        self.run()

    def run(self):
        twist = Twist()

        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th * self.turn
        
        self.publisher.publish(twist)



def getKey():
    key = ser.read(1).decode()
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    
    rospy.init_node('teleop_twist_keyboard')

    #Задаем скорость и порт через параметры
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
    
    pub_move = PublishMove()

    x = 0
    y = 0
    
    th = 0
   
    try:
        pub_move.update(x, y, th, speed, turn)
        ser.write(vels(speed,turn).encode())
        while(1):
            #Читаем через порт какую кнопку нажал пользователь, она придет по радио каналу
            key = getKey()
            #Если нажатая клавиша в словаре движений -  w, a, s, d, задаем соответствующие x, y, th
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                th = moveBindings[key][2]

            #Если нажатая кнопка в словаре "коэффициентов скоростей", то задаем новые скорости
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0]) # линейная скорость
                turn = min(turn_limit, turn * speedBindings[key][1]) # скорость поворота
                if speed == speed_limit:
                    ser.write("Linear speed limit reached!".encode())
                if turn == turn_limit:
                    ser.write("Angular speed limit reached!".encode())
                ser.write(vels(speed,turn).encode())
                
            else:
                # Если нажата любая другая клавиша, робота останавливаем
                if key == '' and x == 0 and y == 0 and th == 0:
                    continue
                x = 0
                y = 0
                th = 0
                #Если пользователь нажал CTRL+С выходим из программы
                if (key == '\x03'):
                    break
            # Обновляем все состояния и скорости и отправляем команду движения роботу 
            pub_move.update(x, y, th, speed, turn)
            pub_move.run()

    except Exception as e:
        print(e)
    
    finally:
        pub_move.stop()
        



