#!/usr/bin/env python
import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Bool
motor_msg = xycar_motor()
distance = []

obstacle_order = 'first' # 1: obstacle frist: left second right

start = 50
end = 310

# start = 10
# end = 370


def callback(data):
	global distance
	distance = data.ranges

def drive_go():
	global motor_msg, pub
	motor_msg.speed = 3
	motor_msg.angle = 0
	pub.publish(motor_msg)

def drive_stop():
	global motor_msg, pub
	motor_msg.speed = 0
	motor_msg.angle = 0
	pub.publish(motor_msg)
        
# direction: 'left' 또는 'right' 문자열. 피해야 하는 방향을 나타냄.
def drive_avoid(direction):
    global motor_msg, pub, right_obstacle, left_obstacle, obstacle_order
    motor_msg.speed = 50  # 장애물 피하기 동작 시 보다 느린 속도로 설정

    current_time = time.time()  # 현재 시간 측정. 이후 시간 경과 계산에 사용.
    
    # 장애물이 감지된 순서에 따라 다르게 처리
    if obstacle_order == 'first':
        if(direction == 'left'):  # 장애물이 왼쪽에 있는 경우
            print("turn_left")

            # 현재 시간부터 0.37초 동안 차량을 오른쪽으로 조향
            while(time.time() - current_time < 0.37 ):
                motor_msg.angle = -40
                pub.publish(motor_msg)
            current_time = time.time()

            # 다음 1초 동안 차량을 왼쪽으로 조향
            while(time.time() - current_time < 1.0 ):
                motor_msg.angle = 40
                pub.publish(motor_msg)
            
            obstacle_order = 'second'  # 장애물을 피한 후 순서를 'second'로 변경
            print(obstacle_order)
            
        elif(direction == 'right'):  # 장애물이 오른쪽에 있는 경우
            print("turn_right")

            # 현재 시간부터 0.36초 동안 차량을 왼쪽으로 조향
            while(time.time() - current_time < 0.36 ):
                motor_msg.angle = 50
                obstacle_order = 'second'  # 장애물을 피한 후 순서를 'second'로 변경
                print(obstacle_order)
                pub.publish(motor_msg)

    else:
        if(direction == 'left'):  # 장애물이 왼쪽에 있는 경우
            print("turn_left")

            # 현재 시간부터 0.5초 동안 차량을 오른쪽으로 조향
            while(time.time() - current_time < 0.5 ):
                motor_msg.angle = -40
                pub.publish(motor_msg)
            current_time = time.time()

            # 다음 1초 동안 차량을 왼쪽으로 조향
            while(time.time() - current_time < 1.0 ):
                motor_msg.angle = 40
                pub.publish(motor_msg)
            
            obstacle_order = 'second'  # 장애물을 피한 후 순서를 'second'로 변경
            
        elif(direction == 'right'):  # 장애물이 오른쪽에 있는 경우
            print("turn_right")

            # 차량을 왼쪽으로 조향
            motor_msg.angle = 40
            obstacle_order = 'second'  # 장애물을 피한 후 순서를 'second'로 변경
            pub.publish(motor_msg)


rospy.init_node('lidar_avoid')
rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
pub_ob = rospy.Publisher('/obstacle', Bool,queue_size=1)

time.sleep(3)
start_time = time.time()
while not rospy.is_shutdown() and time.time() - start_time < 13:

	
	left_obstacle = sum(1 for i in distance[start:180] if i < 0.28)
	right_obstacle = sum(1 for i in distance[181 : end] if i < 0.28)
	print("-"*8)
	print("left", left_obstacle)
	print("right", right_obstacle)
	print("-"*8)

	
	if left_obstacle > 5:  # have to move right
		drive_avoid("right")
		obstacle_order = "second"
		# print("left", left_obstacle)
		# print("right", right_obstacle)
		# print("right")
		pub_ob.publish(True)
	elif right_obstacle > 5: # have to move left
		drive_avoid("left")

		obstacle_order = "second"
		#print("left", left_obstacle)
		#print("right", right_obstacle)
		# print("left")
		pub_ob.publish(True)

	else:
		pub_ob.publish(False)
