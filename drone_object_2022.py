#!/usr/bin/env python3

#from numpy.core.defchararray import count
#from numpy.lib.financial import nper
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_matrix, translation_matrix
from tf import transformations
import math
import os
import sys
import copy

pub = rospy.Publisher('/object_pose', PointStamped, queue_size=10)
pub1 = rospy.Publisher('/camera_pose', PointStamped, queue_size=10)
rospy.init_node('drone_Object', anonymous=True)
rospy.loginfo("Start D435_Object_Distance")
cv_bridge = CvBridge()
done = False;
print('Try to get camera info...')

msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=None)
#     [fx'  0  cx' Tx]
# P = [ 0  fy' cy' Ty]
#     [ 0   0   1   0]
print('Get camera info')
fx = msg.P[0]
fy = msg.P[5]
cx = msg.P[2]
cy = msg.P[6]

transform_time = 0.0
transform = Odometry()

tvmonitor = np.zeros(3)
bottle = np.zeros(3)
teddy_bear = np.zeros(3)
keyboard = np.zeros(3)

tvmonitor_vec = []
bottle_vec = []
teddy_bear_vec = []
keyboard_vec = []

tag1_tvmonitor = []
tag1_bottle = []
tag1_teddy_bear = []
tag1_keyboard = []

tag2_tvmonitor = []
tag2_bottle = []
tag2_teddy_bear = []
tag2_keyboard = []

tag3_tvmonitor = []
tag3_bottle = []
tag3_teddy_bear = []
tag3_keyboard = []

tag4_tvmonitor = []
tag4_bottle = []
tag4_teddy_bear = []
tag4_keyboard = []

disapear_tag_counter = 0#how many frame we leave tag
disapear_tag_flag = 1   #determine whether we leave last tag
apear_tag_conter = 0    #how many frame tag appear, to decide change tag or not
tag_conter = 0          #show what tag is it now
record_threshold = 2    #if there is too less catched object, it may be a mistook frame
disapear_tag_threshold = 5 #threshold of leave tag
record_flag = False     #record this frame or not

def main():
    depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image) # ('???', ???)
    bb_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes) #('???', ???)
    # depth_image_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    ts = message_filters.ApproximateTimeSynchronizer([depth_image_sub, bb_sub], 10, 0.5) #(???, ???, ???)
    ts.registerCallback(callback) #(???)
    rospy.Subscriber("apriltag_localization", Odometry, transform_cb) #("???", ???, ???)
    rospy.spin()
    

def transform_cb(msg): 
    global transform_time
    global transform
    transform_time = msg.header.stamp.to_sec()
    transform = msg
    # print("Get transform time")
    # print(transform_time)

def publish_object_location(object_position, depth_img, org, obj, class_type, bb_size):
    global bottle_output
    global teddybear_output
    global keyboard_output
    global tvmonitor_output
    global record_flag
    global tag_conter

    global tag1_tvmonitor
    global tag1_bottle
    global tag1_teddy_bear
    global tag1_keyboard

    global tag2_tvmonitor
    global tag2_bottle
    global tag2_teddy_bear
    global tag2_keyboard

    global tag3_tvmonitor
    global tag3_bottle
    global tag3_teddy_bear
    global tag3_keyboard

    global tag4_tvmonitor
    global tag4_bottle
    global tag4_teddy_bear
    global tag4_keyboard
    
    global done
    # print(object_position/1000)
    point_message = PointStamped()
    point_message.header = depth_img.header
    point_message.header.frame_id = "origin"
    point_message.point.x = object_position[0]/1000 + org[0]
    point_message.point.y = object_position[1]/1000 + org[1]
    point_message.point.z = object_position[2]/1000 + org[2]
    # update obj
    obj[0] = object_position[0]/1000 + org[0]
    obj[1] = object_position[1]/1000 + org[1]
    obj[2] = object_position[2]/1000 + org[2]
    print('position:', obj[0], obj[1], obj[2])
    
    # print('tag1_bottle_len',len(tag1_bottle))
    # print('tag2_bottle_len',len(tag2_bottle))
    # print('tag3_bottle_len',len(tag3_bottle))
    print('tag:',tag_conter)
    tmp = copy.deepcopy(obj)
    if record_flag:
        if class_type == 'bottle' and tag_conter == 1:
            tag1_bottle.append(tmp)
        elif class_type == 'teddy bear' and tag_conter == 1:
            tag1_teddy_bear.append(tmp)
        elif class_type == 'keyboard' and tag_conter == 1:
            tag1_keyboard.append(tmp)
        elif class_type == 'tvmonitor' and tag_conter == 1:
            tag1_tvmonitor.append(tmp)
        
        elif class_type == 'bottle' and tag_conter == 2:
            tag2_bottle.append(tmp)
        elif class_type == 'teddy bear' and tag_conter == 2:
            tag2_teddy_bear.append(tmp)
        elif class_type == 'keyboard' and tag_conter == 2:
            tag2_keyboard.append(tmp)
        elif class_type == 'tvmonitor' and tag_conter == 2:
            tag2_tvmonitor.append(tmp)
        
        elif class_type == 'bottle' and tag_conter == 3:
            tag3_bottle.append(tmp)
        elif class_type == 'teddy bear' and tag_conter == 3:
            tag3_teddy_bear.append(tmp)
        elif class_type == 'keyboard' and tag_conter == 3:
            tag3_keyboard.append(tmp)
        elif class_type == 'tvmonitor' and tag_conter == 3:
            tag3_tvmonitor.append(tmp)
        
        elif class_type == 'bottle' and tag_conter == 4:
            tag4_bottle.append(tmp)
        elif class_type == 'teddy bear' and tag_conter == 4:
            tag4_teddy_bear.append(tmp)
        elif class_type == 'keyboard' and tag_conter == 4:
            tag4_keyboard.append(tmp)
        elif class_type == 'tvmonitor' and tag_conter == 4:
            tag4_tvmonitor.append(tmp)

    tag1_keyboard_pos = np.zeros(3)
    tag1_bottle_pos = np.zeros(3)
    tag1_teddy_bear_pos = np.zeros(3)
    tag1_tvmonitor_pos = np.zeros(3)
    
    tag2_keyboard_pos = np.zeros(3)
    tag2_bottle_pos = np.zeros(3)
    tag2_teddy_bear_pos = np.zeros(3)
    tag2_tvmonitor_pos = np.zeros(3)
    
    tag3_keyboard_pos = np.zeros(3)
    tag3_bottle_pos = np.zeros(3)
    tag3_teddy_bear_pos = np.zeros(3)
    tag3_tvmonitor_pos = np.zeros(3)
    
    tag4_keyboard_pos = np.zeros(3)
    tag4_bottle_pos = np.zeros(3)
    tag4_teddy_bear_pos = np.zeros(3)
    tag4_tvmonitor_pos = np.zeros(3)
    
    
    for i in tag1_bottle:
        tag1_bottle_pos[0] += i[0]
        tag1_bottle_pos[1] += i[1]
        tag1_bottle_pos[2] += i[2]
    tag1_bottle_pos /= len(tag1_bottle)
    
    for i in tag1_tvmonitor:
        tag1_tvmonitor_pos[0] += i[0]
        tag1_tvmonitor_pos[1] += i[1]
        tag1_tvmonitor_pos[2] += i[2]
    tag1_tvmonitor_pos /= len(tag1_tvmonitor)
        
    for i in tag1_keyboard:
        tag1_keyboard_pos[0] += i[0]
        tag1_keyboard_pos[1] += i[1]
        tag1_keyboard_pos[2] += i[2]
    tag1_keyboard_pos /= len(tag1_keyboard)
        
    for i in tag1_teddy_bear:
        tag1_teddy_bear_pos[0] += i[0]
        tag1_teddy_bear_pos[1] += i[1]
        tag1_teddy_bear_pos[2] += i[2]
    tag1_teddy_bear_pos /= len(tag1_teddy_bear)


    for i in tag2_bottle:
        tag2_bottle_pos[0] += i[0]
        tag2_bottle_pos[1] += i[1]
        tag2_bottle_pos[2] += i[2]
    tag2_bottle_pos /= len(tag2_bottle)
        
    for i in tag2_tvmonitor:
        tag2_tvmonitor_pos[0] += i[0]
        tag2_tvmonitor_pos[1] += i[1]
        tag2_tvmonitor_pos[2] += i[2]
    tag2_tvmonitor_pos /= len(tag2_tvmonitor)
        
    for i in tag2_keyboard:
        tag2_keyboard_pos[0] += i[0]
        tag2_keyboard_pos[1] += i[1]
        tag2_keyboard_pos[2] += i[2]
    tag2_keyboard_pos /= len(tag2_keyboard)
        
    for i in tag2_teddy_bear:
        tag2_teddy_bear_pos[0] += i[0]
        tag2_teddy_bear_pos[1] += i[1]
        tag2_teddy_bear_pos[2] += i[2]
    tag2_teddy_bear_pos /= len(tag2_teddy_bear)


    for i in tag3_bottle:
        tag3_bottle_pos[0] += i[0]
        tag3_bottle_pos[1] += i[1]
        tag3_bottle_pos[2] += i[2]
    tag3_bottle_pos /= len(tag3_bottle)
        
    for i in tag3_tvmonitor:
        tag3_tvmonitor_pos[0] += i[0]
        tag3_tvmonitor_pos[1] += i[1]
        tag3_tvmonitor_pos[2] += i[2]
    tag3_tvmonitor_pos /= len(tag3_tvmonitor)
        
    for i in tag3_keyboard:
        tag3_keyboard_pos[0] += i[0]
        tag3_keyboard_pos[1] += i[1]
        tag3_keyboard_pos[2] += i[2]
    tag3_keyboard_pos /= len(tag3_keyboard)
        
    for i in tag3_teddy_bear:
        tag3_teddy_bear_pos[0] += i[0]
        tag3_teddy_bear_pos[1] += i[1]
        tag3_teddy_bear_pos[2] += i[2]
    tag3_teddy_bear_pos /= len(tag3_teddy_bear)

    for i in tag4_bottle:
        tag4_bottle_pos[0] += i[0]
        tag4_bottle_pos[1] += i[1]
        tag4_bottle_pos[2] += i[2]
    tag4_bottle_pos /= len(tag4_bottle)
        
    for i in tag4_tvmonitor:
        tag4_tvmonitor_pos[0] += i[0]
        tag4_tvmonitor_pos[1] += i[1]
        tag4_tvmonitor_pos[2] += i[2]
    tag4_tvmonitor_pos /= len(tag4_tvmonitor)
        
    for i in tag4_keyboard:
        tag4_keyboard_pos[0] += i[0]
        tag4_keyboard_pos[1] += i[1]
        tag4_keyboard_pos[2] += i[2]
    tag4_keyboard_pos /= len(tag4_keyboard)
        
    for i in tag4_teddy_bear:
        tag4_teddy_bear_pos[0] += i[0]
        tag4_teddy_bear_pos[1] += i[1]
        tag4_teddy_bear_pos[2] += i[2]
    tag4_teddy_bear_pos /= len(tag4_teddy_bear)


    all_position = []
    
    all_position.append(tag1_teddy_bear_pos)    
    all_position.append(tag1_bottle_pos)
    all_position.append(tag1_tvmonitor_pos)
    all_position.append(tag1_keyboard_pos)

    all_position.append(tag2_teddy_bear_pos)    
    all_position.append(tag2_bottle_pos)
    all_position.append(tag2_tvmonitor_pos)
    all_position.append(tag2_keyboard_pos)
    
    all_position.append(tag3_teddy_bear_pos)    
    all_position.append(tag3_bottle_pos)
    all_position.append(tag3_tvmonitor_pos)
    all_position.append(tag3_keyboard_pos)
    
    all_position.append(tag4_teddy_bear_pos)    
    all_position.append(tag4_bottle_pos)
    all_position.append(tag4_tvmonitor_pos)
    all_position.append(tag4_keyboard_pos)
    
    record(all_position)

    # out = open("/home/hcc_wsoutput.txt",'a+')
    # out.write(class_type+str(obj[0])+str(obj[1])+str(obj[2]))
    #print("answer recorded")
    # out.close()
    # append to array
    """if class_type == "bottle":
        bottle_output = np.append(bottle_output,obj)
    elif class_type == "tedddybear":
        teddybear_output = np.append(teddybear_output,obj)
    elif class_type == "keyboard":
        keyboard_output = np.append(keyboard_output,obj)
    elif class_type == "tvmonitor":
        tvmonitor_output = np.append(tvmonitor_output,obj)
    # print(Green_bottle_output.reshape(-1,3))
    # usage: reshape(-1,3) --> [[o,o,o][o,o,o][o,o,o]]
    submission_path = os.path.realpath('..') + "/output/drone_pkg"
    np.savez(submission_path ,
             a = bottle_output,
             b = teddybear_output,
             c = keyboard_output,
             d = tvmonitor_output)"""
    # print("npsave {0}",class_type)

def callback(depth_img, bb):
    global transform_time
    global transform
    # print(bb.bounding_boxes)
    local_time = depth_img.header.stamp.to_sec()
    print("Get local_time")
    print(local_time)
    print(transform_time)

    # print("Get local_time")
    # print(local_time)
    # you could set the time error2, 3, 4, 5 (local_time - transform_time) by yourself    
    if abs(local_time - transform_time) < 1 and transform_time != 0: #??? and transform_time != 0:
        print("Time error")
        print(local_time - transform_time)
        
        # hint: http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
        # You could use "quaternion_matrix" function to find the 4x4 transform matrix
        global_transform = quaternion_matrix(np.array(
                                            [transform.pose.pose.orientation.x, 
                                             transform.pose.pose.orientation.y, 
                                             transform.pose.pose.orientation.z, 
                                             transform.pose.pose.orientation.w])) #(???)
        global_transform[0][3] = transform.pose.pose.position.x #???
        global_transform[1][3] = transform.pose.pose.position.y #???
        global_transform[2][3] = transform.pose.pose.position.z #???
        # print("transform")
        # print(global_transform)
        try:
            cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_img, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
        except CvBridgeError as e:
            print(e)
        
        # publish camera pos in origin frame
        v1 = np.array([0,0,0,1])
        org = np.matmul(global_transform, v1)
        # print("camera_link")
        # print(object_position)
        point_message = PointStamped()
        point_message.header = depth_img.header
        point_message.header.frame_id = "origin"
        point_message.point.x = org[0]
        point_message.point.y = org[1]
        point_message.point.z = org[2]
        pub1.publish(point_message)
        catch_object_counter = 0
        
        for i in bb.bounding_boxes:

            if i.Class == "bottle":
                catch_object_counter = catch_object_counter + 1
            elif i.Class == "keyboard":
                catch_object_counter = catch_object_counter + 1
            elif i.Class == "teddy bear":
                catch_object_counter = catch_object_counter + 1
            elif i.Class == "tvmonitor":
                catch_object_counter = catch_object_counter + 1

        count_tag(catch_object_counter)  

        for i in bb.bounding_boxes:

            x_mean = (i.xmax + i.xmin) / 2
            y_mean = (i.ymax + i.ymin) / 2
            bb_size = (i.xmax - i.xmin)*(i.ymax - i.ymin)
            thr = 10
            """if (i.xmax>(640-thr) or i.xmin<thr or i.ymax>(480-thr) or i.ymin<thr) and i.Class != "umbrella":
                # if i.Class == "umbrella":
                #     print("umbrella")
                continue"""
            """BCI = open("~/hcc_2022/Final_competition/hcc_ws/BCI.txt")
            lines = BCI.readlines()
            if i.Class == lines[0]:
                rospy.loginfo("see "+ lines[0])
                zc = cv_depthimage2[int(y_mean) - 30][int(x_mean)]
                v1 = np.array(getXYZ(x_mean - 30, y_mean, zc, fx, fy, cx, cy))
                object_position = np.matmul(global_transform, v1)
                publish_object_location(object_position,depth_img, org, bottle, "bottle", bb_size)
            BCI.close()"""
            # print(i.Class, type(i.Class))
            catch_object_counter = 0
                            
                            
            if i.Class == "bottle":
                rospy.loginfo("see bottle")
                zc = cv_depthimage2[int(y_mean) - 30][int(x_mean)]
                v1 = np.array(getXYZ(x_mean - 30, y_mean, zc, fx, fy, cx, cy))
                object_position = np.matmul(global_transform, v1)
                publish_object_location(object_position,depth_img, org, bottle, "bottle", bb_size)
                
            elif i.Class == "teddy bear": #and i.probability >= 0.4 and bb_size > 90000
                rospy.loginfo("see teddybear")
                zc = cv_depthimage2[int(y_mean) + 65][int(x_mean)]
                v1 = np.array(getXYZ(x_mean, y_mean + 65, zc, fx, fy, cx, cy))
                object_position = np.matmul(global_transform, v1)
                publish_object_location(object_position,depth_img, org, teddy_bear, "teddy bear", bb_size)
                
            elif i.Class == "keyboard":
                rospy.loginfo("see keyboard")
                zc = cv_depthimage2[int(y_mean) + 10][int(x_mean)]
                v1 = np.array(getXYZ(x_mean, y_mean + 10, zc, fx, fy, cx, cy))
                object_position = np.matmul(global_transform, v1)
                publish_object_location(object_position,depth_img, org, keyboard, "keyboard", bb_size)
                                
            elif i.Class == "tvmonitor":
                rospy.loginfo("see tvmonitor")
                zc = cv_depthimage2[int(y_mean)][int(x_mean)]
                v1 = np.array(getXYZ(x_mean, y_mean, zc, fx, fy, cx, cy))
                object_position = np.matmul(global_transform, v1)
                publish_object_location(object_position,depth_img, org, tvmonitor, "tvmonitor", bb_size)
            
    else:
        count_tag(0)


            # if catch_object_counter >= 3:
                # record to list
            ############################
            #  Student Implementation  #
            ############################

def getXYZ(xp, yp, zc, fx, fy, cx, cy):
    #### Definition:
    # cx, cy : image center(pixel)
    # fx, fy : focal length
    # xp, yp: index of the depth image
    # zc: depth
    inv_fx = 1.0/fx
    inv_fy = 1.0/fy
    x = (xp-cx) *  zc * inv_fx
    y = (yp-cy) *  zc * inv_fy
    z = zc
    return (x,y,z,1.0)

def count_tag(catch_object_num):
    global tag_conter
    global disapear_tag_counter
    global disapear_tag_flag
    global apear_tag_conter
    global record_flag
    global record_threshold
    global disapear_tag_threshold
    if catch_object_num == 0:
        disapear_tag_counter += 1

        if disapear_tag_counter >= disapear_tag_threshold:
            disapear_tag_flag = 1
            disapear_tag_counter = 0
            apear_tag_conter = 0
    
    if catch_object_num > 0:
        disapear_tag_counter = 0
        apear_tag_conter += 1
        
        if apear_tag_conter >= 2 and disapear_tag_flag == 1:
            tag_conter += 1
            disapear_tag_flag = 0
            apear_tag_conter = 0

    if catch_object_num >= record_threshold:
        record_flag = True
    else :
        record_flag = False

def record(all_position):
    'LJK todo'
    print('tag1-1 pos',all_position[0])
    print('tag1-2 pos',all_position[1])
    print('tag1-3 pos',all_position[2])
    print('tag1-4 pos',all_position[3])
    print()
    print('tag2-1 pos',all_position[4])
    print('tag2-2 pos',all_position[5])
    print('tag2-3 pos',all_position[6])
    print('tag2-4 pos',all_position[7])
    print()
    print('tag3-1 pos',all_position[8])
    print('tag3-2 pos',all_position[9])
    print('tag3-3 pos',all_position[10])
    print('tag3-4 pos',all_position[11])
    print()
    print('tag4-1 pos',all_position[12])
    print('tag4-2 pos',all_position[13])
    print('tag4-3 pos',all_position[14])
    print('tag4-4 pos',all_position[15])
    
    BCI = open("BCI.txt")
    lines = BCI.readlines()
    out = open("out.txt",'w')
    print("****************BCI**************  " + lines[0])

    for i in range(4):		
        if lines[i] == "teddybear" or lines[i] == "teddybear\n":
            if not np.isnan(all_position[i + 0].all()):
                print("*******out.write********")
                out.write("teddybear " + str(all_position[i + 0][0]) + " " + str(all_position[i + 0][1]) + " " + str(all_position[i + 0][2]) + '\n')
        elif lines[i] == "bottle" or lines[i] == "bottle\n":
            if not np.isnan(all_position[i + 1].all()):
                print("*******out.write********")
                out.write("bottle " + str(all_position[i + 1][0]) + " " + str(all_position[i + 1][1]) + " " + str(all_position[i + 1][2]) +  '\n')
        elif lines[i] == "tvmonitor" or lines[i] == "tvmonitor\n":
            if not np.isnan(all_position[i + 2].all()):
                print("*******out.write********")
                out.write("tvmonitor " + str(all_position[i + 2][0]) + " " + str(all_position[i + 2][1]) + " " + str(all_position[i + 2][2]) + '\n')
        elif lines[i] == "keyboard" or lines[i] == "keyboard\n":
            if not np.isnan(all_position[i + 3].all()):
                print("*******out.write********")
                out.write("keyboard " + str(all_position[i + 3][0]) + " " + str(all_position[i + 3][1]) + " " + str(all_position[i + 3][2]) + '\n')
    BCI.close()
    out.close()

if __name__ == '__main__':

    print("modified1")
    main()
