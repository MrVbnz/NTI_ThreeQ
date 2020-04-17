import rospy
from clever import srv
from std_srvs.srv import Trigger
from clever.srv import SetLEDEffect
import cv2
from pyzbar import pyzbar
from cv_bridge import CvBridge
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Image
import numpy as np


#перемещение по заданным координатам
def navigate_wait(x, y, z):
    print navigate(x=x, y=y, z=z, speed=1, frame_id=frame_id_t)
    rospy.sleep(sleepTime)

#получение и обработка изображения с камеры
def image_callback(data):
    global needQRFlag
    global b_data
    global color_in_cam
    
    #получение изображения
    img = bridge.imgmsg_to_cv2(data, 'bgr8')
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #отбор двух границ красного
    red_1 = cv2.inRange(hsv, red_1_l, red_1_h)
    red_2 = cv2.inRange(hsv, red_2_l, red_2_h)

    #отбор остальных цветов и обьедининие границ красного
    red = cv2.bitwise_or(red_1, red_2)
    yellow = cv2.inRange(hsv, yellow_l, yellow_h)
    green = cv2.inRange(hsv, green_l, green_h)

    #подсчет количества пикселей нужноых цветов
    redInCam = red.sum()
    yellowInCam = yellow.sum()
    greenInCam = green.sum()

    #выбор результата распознавания
    if redInCam > yellowInCam and redInCam > greenInCam:
        color_in_cam = "red"
    if yellowInCam > redInCam and yellowInCam > greenInCam:
        color_in_cam = "yellow"
    if greenInCam > redInCam and greenInCam > yellowInCam:
        color_in_cam = "green"

    #публикация цветовых масок в каналы отладки
    debug_r_pub.publish(bridge.cv2_to_imgmsg(red, 'bgr8'))
    debug_y_pub.publish(bridge.cv2_to_imgmsg(yellow, 'bgr8'))
    debug_g_pub.publish(bridge.cv2_to_imgmsg(green, 'bgr8'))
    
    #распознавание и печать QR
    if needQRFlag:
        mask = cv2.inRange(img, (0, 0, 0), (200, 200, 200))
        thresholded = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        inverted = 255 - thresholded
        barcodes = pyzbar.decode(inverted)
        b_data = barcodes.data.encode("utf-8")
        print 'QR data: ' + str(b_data)

#определение цвета в заданных координатах
def detect_color(x, y, i):
    print 'TryToDetectColor'
    navigate_wait(x, y, flyz - 0.6)
    print 'Detected color: ' + str(color_in_cam)
    if color_in_cam == 'red' or color_in_cam == 'yellow':
        set_effect(r=148, g=0, b=211)
        rospy.sleep(5)  # blink with white color
        set_effect(r=0, g=0, b=0)
        redIndex.append(i)
    rospy.sleep(5)

#определение QR в заданных координатах
def detect_QR(x, y):
    global needQRFlag
    print 'TryToReadQR'
    navigate_wait(x, y, flyz - 0.6)
    needQRFlag = True
    rospy.sleep(1)
    needQRFlag = False
    print b_data
    if b_data == 'COVID - 19':
        set_effect(r=255, g=0, b=0)
        rospy.sleep(5)
        set_effect(r=0, g=0, b=0)  # blink with white color
    navigate_wait(x, y, flyz)

#перемещение к зараженному по индексу
def gotoreal_sick(listIndex):
    print 'GoesToReadQR'
    for i in range(len(listIndex)):
        currentX = sickList[listIndex[i]][0]
        currentY = sickList[listIndex[i]][1]
        navigate_wait(currentX, currentY, flyz)
        detect_QR(currentX, currentY)


rospy.init_node('flight')

#заполнение HSV границ цветов для cv2.inRange
trs = 15
green_d = np.array((58, 31, 80))
green_h = np.array((green_d[0] + trs, green_d[1] + trs, green_d[2] + trs))
green_l = np.array((green_d[0] - trs, green_d[1] - trs, green_d[2] - trs))

red_1_d = np.array((0, 101, 147))
red_1_h = np.array((red_1_d[0] + trs, red_1_d[1] + trs, red_1_d[2] + trs))
red_1_l = np.array((red_1_d[0], red_1_d[1] - trs, red_1_d[2] - trs))

red_2_d = np.array((255, 101, 147))
red_2_h = np.array((red_2_d[0], red_2_d[1] + trs, red_2_d[2] + trs))
red_2_l = np.array((red_2_d[0] - trs, red_2_d[1] - trs, red_2_d[2] - trs))

yellow_d = np.array((23, 101, 27))
yellow_h = np.array((yellow_d[0] + trs, yellow_d[1] + trs, yellow_d[2] + trs))
yellow_l = np.array((yellow_d[0] - trs, yellow_d[1] - trs, yellow_d[2] - trs))

redInCam = 0
greenInCam = 0
yellowInCam = 0

#создание каналов публикации
debug_r_pub = rospy.Publisher('~debug_color_r', Image)
debug_y_pub = rospy.Publisher('~debug_color_y', Image)
debug_g_pub = rospy.Publisher('~debug_color_g', Image)

#создание каналов публикации
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

#подписка на камеру с низким FPS
bridge = CvBridge()
image_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
rospy.spin() #СТРОКА СКОПИРОВАНА ИЗ МАНУАЛА. 3 раза помешала успешному запуску.

sickList = [[0, 2.27], [0.72, 3.94], [0.72, 1.5], [2.88, 1.5], [2.88, 3.94], [2.16, 0.28], [1.44, 2.72], [1.44, 1.5],
            [2.16, 2.72], [3.66, 0.28]]
redIndex = []
yellowIndex = []
redEnd = []
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
set_effect(r=0, g=0, b=0)
# if next line is not frame_id = 'aruco_map', please fix it
frame_id_t = 'aruco_map'
flyz = 1.2
sleepTime = 4
needQRFlag = False
b_data = ''
color_in_cam = ''

#взлет
flyz=1.2
print 'Try navigate body'
print navigate(z=flyz, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
print 'Try navigate aruco_map'
print navigate(z=flyz, speed=0.5, frame_id='aruco_map', auto_arm=True)
rospy.sleep(10)

startPoint = get_telemetry(frame_id=frame_id_t)

#проход по известным точкам
for i in range(len(sickList)):
    currentSickX = sickList[i][0]
    currentSickY = sickList[i][1]
    navigate_wait(currentSickX, currentSickY, flyz)
    detect_color(currentSickX, currentSickY, i)
    navigate_wait(currentSickX, currentSickY, flyz)
    print 'DoneWithCurrentSick x: ' + str(currentSickX) + 'y: ' + str(currentSickY)

#возврат в точку старта
print 'FliesHome'
print navigate(x=startPoint.x, y=startPoint.y, z=flyz, speed=1, frame_id=frame_id_t)
rospy.sleep(6)

#преземление + сон 2 мин
print land()
rospy.sleep(5)
print arming(False)

print 'Wait 2 min'
rospy.sleep(120)

#второй взлет
flyz=1.2
print 'Try navigate body'
print navigate(z=flyz, speed=0.5, frame_id='body', auto_arm=True)
rospy.sleep(2)
print 'Try navigate aruco_map'
print navigate(z=flyz, speed=0.5, frame_id='aruco_map', auto_arm=True)
rospy.sleep(5)

#посещение красных и желтых точек
print redIndex
print yellowIndex
gotoreal_sick(redIndex)
gotoreal_sick(yellowIndex)

#возврат в точку старта
print 'FliesHome'
print navigate(x=startPoint.x, y=startPoint.y, z=flyz, speed=1, frame_id=frame_id_t)
rospy.sleep(6)

#преземление
print land()
rospy.sleep(5)
print arming(False)
