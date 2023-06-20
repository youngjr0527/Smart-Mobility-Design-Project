#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image
import time
from std_msgs.msg import Bool
import sys
import os
import signal

from std_msgs.msg import Int64, String
from ar_track_alvar_msgs.msg import AlvarMarkers

from tf.transformations import euler_from_quaternion

###############ar_tag#################
arData = {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0, "ID": 7.0}
roll, pitch, yaw = 0, 0, 0
time_count = 0
left_color = ""
right_color = ""
hough_flag = True
ar_flag = False
crosswalk_flag = 0


# 횡단보도에서의 행동을 조절하는 함수
# action: 수행할 동작 (stop, sprint, hold back 중 하나)
# time_count: 대기 시간
def desperado(action, time_count):
    global crosswalk_flag  # 횡단보도에서의 행동 상태를 나타내는 전역 변수

    if action == 'stop':  # 'stop' 동작인 경우 (신호등이 빨간색인 경우)
        drive(0, 0)  # 차량을 정지
        rospy.sleep(time_count+0.5)  # 설정한 시간만큼 대기
        crosswalk_flag = 1  # 횡단보도 행동 상태를 '행동 중'으로 변경
        return

    elif action == 'sprint':  # 'sprint' 동작인 경우
        if crosswalk_flag == 0:  # 횡단보도 행동 상태가 '대기 중'인 경우
            start_time = time.time()  # 현재 시간 저장
            while(time.time() - start_time < 3):  # 3초 동안 전력 질주
                drive(0, 6)  # 차량을 전진시킴 (허브 변환하면 그냥 주행이 안전할지도)
            crosswalk_flag = 1  # 횡단보도 행동 상태를 '행동 중'으로 변경
            return
        
    elif action == 'hold back':  # 'hold back' 동작인 경우
        drive(0, 0)  # 차량을 정지
        rospy.sleep(time_count+10)  # 설정한 시간에 10초를 더한 시간만큼 대기 (남은 노란불 + 빨간불 시간)
        crosswalk_flag = 1  # 횡단보도 행동 상태를 '행동 중'으로 변경
        return
        

def park_PID():
    global arData, start_park_pid_time
    end_time = time.time()
    dt = end_time - start_park_pid_time
    start_park_pid_time = end_time

    ki = 0.001
    i_error = 0
    

    kp = 3
    (roll, pitch, yaw) = euler_from_quaternion(
		(arData["AX"], arData["AY"], arData["AZ"], arData["AW"])) 
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    # print(yaw)
    error = yaw
    p_error = kp * error
    i_error = i_error + ki * error * dt
    angle = i_error + p_error

    print(angle)
    return angle


def Left_light_callback(msg):
	global left_color
	left_color = msg.data
	
def Right_light_callback(msg):
	global right_color
	right_color = msg.data
	
def light_time_callback(msg):
	global time_count
	time_count = msg.data



def AR_callback(msg):
	global arData
	# print("1")
	for i in msg.markers:
		arData["DX"] = i.pose.pose.position.x
		arData["DY"] = i.pose.pose.position.y
		arData["DZ"] = i.pose.pose.position.z
		arData["AX"] = i.pose.pose.orientation.x
		arData["AY"] = i.pose.pose.orientation.y
		arData["AZ"] = i.pose.pose.orientation.z
		arData["AW"] = i.pose.pose.orientation.w
		arData["ID"] = i.id


left_list =  [['R',10],['R',9],['R',8],['R',7],['R',6],['R',5],['R',4],['R',3],['R',2],['R',1],
              ['G',10],['G',9],['G',8],['G',7],['G',6],['G',5],['G',4],['Y',3],['Y',2],['Y',1]]

right_list = [['G',10],['G',9],['G',8],['G',7],['G',6],['G',5],['G',4],['Y',3],['Y',2],['Y',1],
              ['R',10],['R',9],['R',8],['R',7],['R',6],['R',5],['R',4],['R',3],['R',2],['R',1]]

def foresee(left_color, time_count, detect2stopline_time):
    time_indices = [i for i, x in enumerate(left_list) if x[1] == time_count] # time_count가 일치하는 2개의 인덱스를 찾는다
    
    current_index_list = [i for i in time_indices if left_list[i][0] == left_color] # 2개의 인덱스 중 color 까지 동일한 하나의 인덱스를 찾는다
    # print(current_index_list)
    current_index = current_index_list[0]
    future_index =(current_index + detect2stopline_time) % 20
    print("-"*10)
    print(left_list[future_index][0], left_list[future_index][1])
    print("-"*10)
    return left_list[future_index][0], left_list[future_index][1]


def decide_left_or_right(left_color,time_count):  
    detect2stopline_time = 7 # 실제로 재봐야 함
    # print(left_color)
    future_L_color, future_time = foresee(left_color, time_count, detect2stopline_time)
    if future_L_color == 'R' and future_time == 1:  # 애매한 경우 
		# 왼쪽으로 가야함
        return 'turn_left'
    elif future_L_color == 'Y' and future_time < 2:
        return 'turn_right'

    elif future_L_color == 'R' and future_time > 1:
		# 오른쪽으로 가야함
        return 'turn_right'
    else:
		# 왼쪽으로 가야함 
        return 'turn_left'

def drive_turn(Angle, Speed): 
    global pub
    rospy.sleep(2.5)
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    # print(msg.speed)
    rospy.sleep(1.5)
    pub.publish(msg)


###############ar_tag#################

ob_flag = False

def ob_callback(data):
    global ob_flag
    ob_flag = data.data

def PID(input_data, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror =  error - prev_error

    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
speed = 50
start_time = 0.0

# 횡단보도 탐지 함수
# image: 차량 카메라에서 가져온 이미지
def detect_crosswalk(image):
      
    roi = image[350:400, 150:500]  # 원본 이미지에서 관심 영역(ROI)을 설정. 이 값은 경험적으로 찾아내야 함.
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  # ROI를 그레이스케일로 변환
    _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # 그레이스케일 이미지를 이진 이미지로 변환. 여기서는 픽셀값 200 이상을 가진 픽셀은 255(흰색)으로, 그렇지 않은 픽셀은 0(검은색)으로 변환.
    white_pixels = np.count_nonzero(binary == 255)  # 이진 이미지에서 흰색 픽셀의 개수를 세기
    total_pixels = binary.shape[0] * binary.shape[1]  # 이진 이미지의 전체 픽셀 개수를 계산
    
    # 흰색 픽셀의 비율이 전체 픽셀 대비 0.1보다 크다면 횡단보도로 판단하고 True를 반환
    if float(white_pixels) / total_pixels > 0.1:
        return True
    else:
        return False  # 그렇지 않다면 횡단보도가 아니라고 판단하고 False를 반환


def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")    

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    # print(msg.speed)

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x1+x2 < Width):
            left_lines.append([Line.tolist()])

        elif (slope > 0) and (x1 < x2):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global pub_temp

    # frame: 웹캠 등으로부터 받아온 이미지를 말함.

    # 이미지를 그레이스케일로 변환한다. 이를 통해 이미지 처리의 복잡성을 줄인다.
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # 가우시안 블러를 이용해 그레이스케일 이미지를 흐릿하게 만든다. 이렇게 하면 노이즈와 세부적인 정보를 줄일 수 있다.
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # 캐니 엣지를 이용해 이미지에서 경계선을 찾는다. 이를 통해 차선과 같은 중요한 정보를 강조한다.
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # 차선을 찾기 위해 관심 영역을 설정하고, HoughLinesP 함수를 이용해 이 영역에서 직선을 찾는다.
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # 찾아낸 모든 선을 좌측 차선과 우측 차선으로 분류한다. 만약 차선이 없다면 해당 함수는 0과 640을 반환하며 종료한다.
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # 각 차선의 중심 위치를 구한다.
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # 차선을 그린다.
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)

    # 차량의 위치를 나타내는 사각형을 그린다.
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    frame = cv2.rectangle(frame, (150,350 ),(500,400),(0, 0, 255), 2)

    # 변환된 이미지를 OpenCV의 imshow 함수를 통해 화면에 출력한다.
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    image_msg.header.frame_id = "map"
    pub_temp.publish(image_msg)

    return lpos, rpos

def loadend():
    print("stop")
    drive(0,0)
    hough_flag = False
    start_stop = time.time()
    while(time.time() - start_stop < 0.5 ):
        angle = 0
        speed = 0
        drive(angle, speed)
    return angle

# 갈림길에서의 주행 방향 결정 함수
def twoway_decide():
    global ar_flag  # 전역 변수 ar_flag 선언
    direction = decide_left_or_right(left_color,time_count)  # 왼쪽 혹은 오른쪽 방향을 결정하는 함수 호출

    start_intersection = time.time()  # 현재 시간을 start_intersection에 저장
    ar_flag = True  # ar_flag를 True로 설정
    
    # 왼쪽으로 턴을 결정한 경우
    if direction == 'turn_left':
        start_intersection = time.time()  # 현재 시간을 start_intersection에 저장
        # 일정 시간(3.5초) 동안 직진
        while(time.time() - start_intersection < 3.5):
            drive(0,5)
                
        start_intersection = time.time()  # 현재 시간을 start_intersection에 저장
        # 일정 시간(1.2초) 동안 왼쪽으로 방향전환
        while(time.time() - start_intersection < 1.2):
            drive(-40,5)
        start_intersection = time.time()

        # 일정 시간(0.5초) 동안 다시 직진
        while(time.time() - start_intersection < 0.5):
            drive(40,5)
    else:
        print("error")  # 방향 결정에 에러가 발생한 경우 에러 메시지 출력

    # 오른쪽으로 턴을 결정한 경우
    if direction == 'turn_right':
        start_intersection = time.time()  # 현재 시간을 start_intersection에 저장
        # 일정 시간(2.8초) 동안 직진
        while(time.time() - start_intersection < 2.8):
            drive(0,5)
                
        start_intersection = time.time()  # 현재 시간을 start_intersection에 저장
        # 일정 시간(1.4초) 동안 오른쪽으로 방향전환
        while(time.time() - start_intersection < 1.4):
            drive(40,5)
        start_intersection = time.time()

        # 일정 시간(0.4초) 동안 다시 직진
        while(time.time() - start_intersection < 0.4):
            print(3)
            drive(-40,5)
    else:
        print("error")  # 방향 결정에 에러가 발생한 경우 에러 메시지 출력


# 평행 주차를 수행하는 함수
def parking():
    global start_park_pid_time  # 주차 시작 시간을 나타내는 전역 변수

    # 첫 번째 단계: 오른쪽으로 최대한 조향하면서 일정 시간 동안 후진
    start_park = time.time()  # 현재 시간 저장
    while(time.time() - start_park < 1.40 ):
        angle = 50  # 오른쪽으로 최대한 조향
        speed = -9  # 후진
        drive(angle, speed)  # 차량을 제어

    # 두 번째 단계: 왼쪽으로 최대한 조향하면서 같은 시간 동안 후진
    start_park = time.time()  # 현재 시간 저장
    while(time.time() - start_park < 1.40 ):
        angle = -50  # 왼쪽으로 최대한 조향
        speed = -9  # 후진
        drive(angle, speed)  # 차량을 제어

    # 세 번째 단계: 주차 AR 태그의 거리가 주차 영역까지 가까워질 때까지 전진
    start_park = time.time()  # 현재 시간 저장
    while(time.time() - start_park < 0.8):
        drive(0,0)  # 차량을 정지
    
    start_park = time.time()  # 현재 시간 저장
    while(time.time() - start_park < 10.8):
        start_park_pid_time = time.time()  # 현재 시간 저장
        drive(0,9)  # 차량을 전진시킴
        if(arData["ID"] == 6 and arData["DZ"] < 0.74):  # AR 태그의 ID가 6이고 거리가 0.74 미만인 경우
            drive(0,0)  # 차량을 정지
            rospy.sleep(0.5)  # 0.5초 동안 대기
            break  # 반복문을 종료


def start():
    global pub
    global image
    global cap
    global Width, Height, speed
    global pub_temp, ob_flag, hough_flag
    line_detect = False
    global left_color, time_count, start_park_pid_time, ar_flag
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    pub_temp = rospy.Publisher('/hough_result', Image, queue_size=10)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    ob_sub = rospy.Subscriber("/obstacle", Bool, ob_callback)

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, AR_callback, queue_size=1)
    rospy.Subscriber('/Left_color',String,Left_light_callback)
    rospy.Subscriber('/Right_color',String,Right_light_callback)
    rospy.Subscriber('/time_count',Int64,light_time_callback)
    print("---------- Xycar A2 v1.0 ----------")
    rospy.sleep(2)
     
    # hough_flag = False
    direction = "None"
    
    while (hough_flag == True):
        while not image.size == (640*480*3):
            continue
 
        if (ob_flag == False):
            line_detect = detect_crosswalk(image)

        if (line_detect):
            # motor_msg = xycar_motor()
            # motor_msg.speed = 0 
            # motor_msg.angle = 0
            # pub.publish(motor_msg)
            print(line_detect)

        #main code에서 실행되는 부분. cw_flag = 1이면 crosswalk를 만나기 전, 0이면 crosswalk를 지난 후 
        
        if line_detect and left_color == 'R' :
            desperado('stop', time_count)
        elif line_detect and left_color == 'G':
            desperado('sprint', time_count)
        elif line_detect and left_color == 'Y': 
            if time_count < 1.8:
                desperado('hold back',time_count)
            else:
                desperado('sprint',time_count)
        
        lpos, rpos = process_image(image)
        center = (lpos + rpos) / 2
        angle = PID(center, 0.50, 0.0, 0.0)
        # angle = PID(center,0.42, 0.0007, 0.05)

        if arData["ID"] == 2 and arData["DZ"] < 0.71 and ar_flag == False:  #카메라 찍고 미래를 그려봄
            twoway_decide()

        elif arData["ID"] == 4 and arData["DZ"] < 0.8:  #카메라 찍고 미래를 그려봄
            angle = loadend()
        

        drive(angle, speed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    ##### park #######
    parking()

    rospy.spin()
    
if __name__ == '__main__':

    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()

    start()
