#!/usr/bin/env python
import rospy
import math
# the message that we get from the arduino
from std_msgs.msg import Int32
from kobuki_msgs.msg import ButtonEvent,BumperEvent
# the output message controlling the speed and direction of the robot
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan

def judge_bumper(data):
    global bumper
    if data.state == BumperEvent.PRESSED:
        bumper = True

def judge_button(data):
    global bumper
    if data.state == ButtonEvent.Button0:
        bumper = False

def ir_callback(data):


    # Twist is a message type in ros, here we use an Twist message to control kobuki's speed
    # twist. linear.x is the forward velocity, if it is zero, robot will be static, 
    # if it is grater than 0, robot will move forward, otherwise, robot will move backward
    # twist.angular.axis is the rotatin velocity around the axis 
    # 
    # Around which axis do we have to turn? In wich direction will it turn with a positive value? 
    # Right hand coordinate system: x forward, y left, z up

    global former_start_time
    global former
    global ang_list
    global cone_list_right
    global cone_list_left,ang,round_time,_ang
    global cone_list,time_stop2,time_stop1,time_stop3,emerg
    global left_has_thing,right_has_thing,bumper



    def scan(above_sen,number2,number3):
        global former,right_has_thing,left_has_thing
        change = abs(former - above_sen)
        #print(change)
        if change >= 40:
            #print(change)
            if 0<number2<40: 
                cone_list_left.append((above_sen,number2))
                #print(cone_list_right)
            elif 140<number2<180:
                cone_list_right.append((above_sen,number2))
                #print(cone_list_left)
        
        if 0<number3<45 or 135<number3<180:
            #print(number2, above_sen)
            pass
        
        if (0<number3<20 or 350<number3<360) and above_sen > 410:
            left_has_thing = [True,above_sen]
        if 150<number3<195  and above_sen > 410:
            right_has_thing = [True,above_sen]
        former = above_sen
        #print(number3,above_sen)


    def move(time,ang):
        global time_stop1,time_stop2
        time_stop2 = time+abs(ang/(180/3.14159))


    def deta(r1,a1,r2,a2):
        n1 = r1*math.cos(a1/180*math.pi)+r2*math.cos(a2/180*math.pi)
        n2 = (r1**2+r2**2+2*r1*r2*math.cos((a1-a2)/180*math.pi))**0.5
        
        return math.acos(n1/n2)/math.pi*180


    def dist(x):
        #print(0.0003*(x**2) - 0.3721*x + 131.4)
        return 0.0003*(x**2) - 0.3721*x + 131.4


    twist = Twist()
    if not bumper:
        twist.linear.x = 0.6
        twist.angular.z = 0
        
        below_sen = int(str(bin(data.data))[3:-16],2)
        above_sen = int(str(bin(data.data))[-16:],2)
        time_now = rospy.get_time()

        if below_sen < 100: 
            if time_now-former_start_time > round_time: 
                former_start_time = time_now
                #print(cone_list_left)
                if len(cone_list_right) >= 2 and len(cone_list_left) >= 2:
                    r1 = dist(float((cone_list_right[-1][0]+cone_list_right[-2][0])/2))
                    print(cone_list_right)
                    #print(cone_list_left)
                    a1 = (cone_list_right[-1][1]+cone_list_right[-2][1])/2
                    r2 = dist(float((cone_list_left[0][0]+cone_list_left[1][0])/2))
                    a2 = (cone_list_left[0][1]+cone_list_left[1][1])/2
                    ang = 90 - deta(r1,a1,r2,a2)
                    if len(cone_list_right) >= 4 and len(cone_list_left) >= 4:
                        r1 = dist(float((cone_list_right[-3][0]+cone_list_right[-4][0])/2))
                        #print(cone_list_right)
                        #print(cone_list_left)
                        a1 = (cone_list_right[-3][1]+cone_list_right[-4][1])/2
                        r2 = dist(float((cone_list_left[2][0]+cone_list_left[3][0])/2))
                        a2 = (cone_list_left[2][1]+cone_list_left[3][1])/2
                        ang = (ang + 90 - deta(r1,a1,r2,a2))/2
                        if len(cone_list_right) >= 6 and len(cone_list_left) >= 6:
                            r1 = dist(float((cone_list_right[-5][0]+cone_list_right[-6][0])/2))
                            #print(cone_list_right)
                            #print(cone_list_left)
                            a1 = (cone_list_right[-5][1]+cone_list_right[-6][1])/2
                            r2 = dist(float((cone_list_left[4][0]+cone_list_left[5][0])/2))
                            a2 = (cone_list_left[4][1]+cone_list_left[5][1])/2
                            ang = (ang + 90 - deta(r1,a1,r2,a2))/2
                    cone_list_left =[]
                    cone_list_right = []
                    ang_list.append(ang)
                    print(ang)
                    move(time_now,ang)
                #print(former_start_time)

        number2 = rad_speed*(time_now-former_start_time)  
        if number2 < 0:
            number2 = 360+number2
        #print(number2)
        #print(below_sen)
        number3 = rad_speed*(time_now-former_start_time) -80 
        if number3 < 0:
            number3 = 360 + number3 

        scan(above_sen,number2,number3)

        #print(above_sen > 423)
        #if 88 < number2 < 92:
            #print(above_sen,number2)
        #print(above_sen>423,170<number2<190)


        if above_sen > 423 and 80<number3<100 and time_now>time_stop3:
            give_chance = False
            print('--------------------------something in front----------------------')
            print(not left_has_thing[0],not right_has_thing[0])
            #print(ang_list)
            if left_has_thing[0] and (not right_has_thing[0]):
                _ang = -90
                print("--------------------right-------------------")
            elif right_has_thing[0] and (not left_has_thing[0]):
                _ang = 90
                print("---------------------left-------------------")
            elif left_has_thing[0] and right_has_thing[0]:
                print("-----------douyoudongxi-------------------")
                if left_has_thing[1] > right_has_thing[1]:
                    print("r")
                    _ang = -90
                else:
                    _ang = 90
                    print("l")
                
            elif (not left_has_thing[0]) and (not right_has_thing[0]) :
                print('------------------------give me a chance----------------')
                print(ang_list)
                give_chance = True
                if ang_list == []:
                    _ang = 0
                elif ang_list[-1] > 0:
                    _ang = -30
                else:
                    _ang = 30
            time_stop3 = time_now+abs(_ang/(180/3.14159))
            emerg = True
            left_has_thing[0] = False
            right_has_thing[0] = False

        if time_now<time_stop2:
            if ang < 0:
                twist.angular.z = -1
            else:
                twist.angular.z = 1

        if emerg and time_now<time_stop3:
            if _ang < 0:
                twist.angular.z = -1
            else:
                twist.angular.z = 1
            twist.linear.x = 0.0

        if emerg and time_now>time_stop3:
            emerg == False
        '''
        twist.linear.x = 0
        twist.angular.z = 0
        '''
    else:
        twist.linear.x = 0
        twist.angular.z = 0
    kobuki_velocity_pub.publish(twist)      
    
    
def range_controller():
    
    # define the publisher globally
    global kobuki_velocity_pub
    #define the cone list globally
    global cone_list
    
    #define the former value globally
    global former,ang_list
    
    global former_start_time
    global rad_speed,round_time
    global time_stop2
    global time_stop1
    global time_stop3
    global cone_list_left,left_has_thing,right_has_thing
    global cone_list_right,emerg,bumper

    emerg = False
    ang_list = []
    cone_list = []
    former = 0
    former_start_time = 0
    cone_list_right = []
    cone_list_left = []
    time = 0.324
    rad_speed = 360/time
    round_time = 360/rad_speed - 0.05
    left_has_thing = [False,0]
    right_has_thing = [False,0]
    bumperstate = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, judge_bumper)
    buttonstate = rospy.Subscriber('mobile_base/events/button', ButtonEvent, judge_button)
    '''
    rospy.init_node('laser_scan_publisher')
    scan_pub = rospy.Publisher('scan', LaserScan, queue_size = 50)
 n  '''
    # initialize the node
    rospy.init_node('range_controller', anonymous=True)

    time_stop2 = rospy.get_time()
    time_stop1 = rospy.get_time()
    time_stop3 = rospy.get_time()


    # initialize the publisher - to publish Twist message on the topic below...
    kobuki_velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # subscribe to the topic '/ir_data' of message type Int32. The function 'ir_callback' will be called
    # every time a new message is received - the parameter passed to the function is the message
    rospy.Subscriber("/ir_data", Int32, ir_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

# start the line follow
if __name__ == '__main__':
    global bumper
    bumper = False
    range_controller()