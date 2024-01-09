import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

ksize = 5

def Get_Color_Point(img,rgb,under_thresh,upper_thresh):
    minBGR = np.array([rgb[2] - under_thresh, rgb[1] - under_thresh, rgb[0] - under_thresh])
    maxBGR = np.array([rgb[2] + upper_thresh, rgb[1] + upper_thresh, rgb[0] + upper_thresh])

    maskBGR = cv2.inRange(img,minBGR,maxBGR)

    resultRGB = cv2.bitwise_and(img, img, mask = maskBGR)

    gray_rgb = cv2.cvtColor(resultRGB,cv2.COLOR_BGR2GRAY)

    gray_rgb = cv2.medianBlur(gray_rgb,ksize)

    ret, th = cv2.threshold(gray_rgb, 90, 255, cv2.THRESH_BINARY)

    return th

px_rate = 8#distance of camera * px of object
ball_size = 0.205#meter
tan_view_angle = 11.4/23.8#horizon angle,Verticalangle

def Measure_Distance(center,r):
    coord = np.array([0,0,0])
    if int(r) < 5 or int(r) > 90:
        return coord
    distance = px_rate/r
    frame_coord = [center[0]-frame.shape[1]/2,-center[1]+frame.shape[0]/2]
    distance_xy = np.sqrt(center[0]**2 + center[1]**2)*circle_size/(2*r)

    coord = [
            distance_xy*frame_coord[0]/np.sqrt(center[0]**2 + center[1]**2),
            distance_xy*frame_coord[1]/np.sqrt(center[0]**2 + center[1]**2),
            np.sqrt(distance**2-distance_xy**2)
            ]

    return coord

def Sarch_Circle(contours,min_size,max_size,number_of_circles):

    margin = 100

    circle_data = np.zeros((margin,3))

    count = 0

    if len(contours) > 0:
        for i ,cnt in enumerate(contours):
            if count >= margin:
                break

            center ,r = cv2.minEnclosingCircle(cnt)

            if min_size < int(r) and int(r) < max_size:
                circle_data[count,:] += [center[0],center[1],r]
                count+=1

    circles = circle_data[np.argsort(circle_data[:, 2])]

    if count > number_of_circles:
        return circle_data[0:number_of_circles,:]

    return circle_data[0:count,:]




class ImgReceiver(Node):
    def __init__(self):

        super().__init__("receiver")
        self.br = CvBridge()
        self.subscription = self.create_subscription(Image,"image_raw",self.cb,qos_profile_sensor_data)
        self.publisher = self.create_publisher(Image,"processed",10)
    def cb(self,data):

        #process of opencv
        #self.get_logger().info("roop")

        frame = self.br.imgmsg_to_cv2(data,'bgr8')

        no_img = np.zeros(frame.shape)

        color_point= Get_Color_Point(frame,[254,209,65],80,80)#[244,54,76]

        contours, hierarchy = cv2.findContours(color_point,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i,cnt in enumerate(contours):
            area = cv2.contourArea(cnt,False)
            if area > 500 :#and area < 1000:
                cv2.drawContours(no_img,cnt,-1,(255,255,255),2)
                cv2.circle(no_img,(int(np.average(cnt[:,0,0])),int(np.average(cnt[:,0,1]))),2,(0,255,0),2)
                cv2.putText(no_img,text=str(area),org=(int(np.average(cnt[:,0,0])),int(np.average(cnt[:,0,1]))),fontFace=cv2.FONT_HERSHEY_SIMPLEX,fontScale=1.0,color=(0, 255, 0),thickness=2,lineType=cv2.LINE_AA)
        
        circles = Sarch_Circle(contours,0,90,10)

        for i in circles:
            cv2.circle(frame,(int(i[0]),int(i[1])),int(i[2]),(0,255,0),2)

        result = self.br.cv2_to_imgmsg(frame,'bgr8')
        self.publisher.publish(result)

        pass

def main():
    rclpy.init()
    img_receiver = ImgReceiver()
    #try:
    rclpy.spin(img_receiver)
    #except KeyboardInterrupt:
        #pass
    #rclpy.shutdown()

