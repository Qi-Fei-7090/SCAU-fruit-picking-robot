#!/usr/bin/python3
#coding:utf-8

import ctypes
import rospy
import os  
import time
import sys
import math
import numpy as np
np.bool = bool
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,CameraInfo,PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32
from yolo5_detector.srv import *
#from yolo5_detector.srv import ControlCmd

import message_filters

import pycuda.driver as cuda
#import matplotlib
#matplotlib.use('Agg') #prevent GTK error
from tensorRT.yolov5_trt import YoLov5TRT
from tensorRT.yolov5_seg_trt import YoLov5segTRT
from com.common import * #通用功能
from com.datastruct import resultinfo 
from com.camerapara import * #相机内参存储

##################################
#带网络结构的权重文件名,存储在scripts/tensorRT/下
pluginlibname = "libmyplugins.so"
wfname="apple.engine"  # tensorRT识别模型文件
categories = ["apple"]  #识别的类别，和网络同步
#categories = ["apple","blackscrew","blueblock","greenblock","orange","purpleblock","redblock","screwdriver","whitescrew","yellowblock","milk",]


#使用UI显示识别结果的开关 True or False
showresult=False
#showresult=True

#相机回调函数帧率统计  字典
frame_count = {}
ttime = {}

#相机参数
#本地存储的摄像头内参.
cam_params = {}
is_getcamparams = {} #是否获取到的相机的内参


#数据队列FIFO 字典
dataqueues = {} 
resultqueues = {} 

#ROS输出对象
yolo5_pubs={} # 识别目标的topic
 
yolo5_tf=0 #一个目标的TF坐标



###多进程
isalive = None #用于关闭多进程
isworking = None #用于进程休眠
proc = None #子进程
#输入:进程关闭控制,输入数据队列,输出数据队列,摄像头内参K值,网络类别名称列表,rt库名,engine文件名
def PredictionProcess(isalive,isworking,inqueuedict, outqueuedict,cam_paramdict=None,classname=["apple",],pluginlibname="libmyplugins.so",wfname="apple_best_s.engine"):
    print("!initialization for TensorRT....")
    print(wfname,classname)
    #初始化cuda
    cuda.init()
    #tensorRT
    # load custom plugin and engine
    dirname, filename = os.path.split(os.path.abspath(sys.argv[0]))
        
    #初始化推理类
    yolov5_wrapper =None

    detectMode='' #'seg'为实例分割模式(可以用，但暂无技术支持)  ''为目标检测模式

    # a YoLov5TRT instance
    if(detectMode == 'seg'):
        PLUGIN_LIBRARY = dirname+"/tensorRT/"+'libmyplugins_seg.so'  #tensorrt相关的网络库文件
        print(PLUGIN_LIBRARY)
        engine_file_path = dirname+"/tensorRT/"+'desktop_seg_s.engine'    #网络权值文件
        ctypes.CDLL(PLUGIN_LIBRARY)
        yolov5_wrapper = YoLov5segTRT(engine_file_path)
    else:
        PLUGIN_LIBRARY = dirname+"/tensorRT/"+pluginlibname  #tensorrt相关的网络库文件
        print(PLUGIN_LIBRARY)
        engine_file_path = dirname+"/tensorRT/"+wfname    #网络权值文件
        ctypes.CDLL(PLUGIN_LIBRARY)

        yolov5_wrapper = YoLov5TRT(engine_file_path)
    print("!RT Done")

    print("start dector!\n")
    cimg = np.array([1])
    cname = ""
    depth = np.array([1])

    oldsortdict={} #对应相机上一帧输出的目标顺序
    

    #获取字典key名称
    innames = list(inqueuedict.keys())
    total_img = len(innames)
    #print(innames,total_img)
    name_index = 0  #当前轮询到了哪一个
    while not rospy.is_shutdown() or isalive:
        #print("isalive:",isalive.value)
        if not isworking.value:
            print("Sleeping......")
            #print("isworking:",isworking.value)
            time.sleep(1)
            continue

        t0 = time.time()
        #获取深度相机数据
        #轮询所有输入队列获取图像
        nowname = innames[name_index]
        name_index +=1
        if(name_index >= total_img):
            name_index = 0
        
        #获取对应相机内参
        cam_param = cam_intrinsics1()
        cam_param.D = cam_paramdict[nowname]['D']
        cam_param.K = cam_paramdict[nowname]['K']
        cam_param.R = cam_paramdict[nowname]['R']
        cam_param.P = cam_paramdict[nowname]['P']
        
        #如果相机上一帧对应的顺序列表不存在，则初始化
        if  nowname not in oldsortdict: 
            oldsortdict[nowname] = []
            print(oldsortdict[nowname])
        
        oldsort = oldsortdict[nowname]

        #获取数据队列对象
        inqueue = inqueuedict[nowname]
        outqueue = outqueuedict[nowname]

        if(inqueue.empty()):
            print("没有获取到新图像!等待图像到来!")
            time.sleep(0.5) ##########################################0.05
            continue
        else:
            data=inqueue.get()
                 
            cimg = data[0]
            depth = data[1]
            cname = data[2]
            print("\n\n##############从队列中获取了1帧图像###########\n\n")
        if(cimg.shape[0] == 1 or depth.shape[0]==1):
            print("图像初始化中......")
            continue    
            
        #预测一帧
        results, use_time = yolov5_wrapper.infer(cimg)
        print("!!!tensorRT_TIME:%.4f"%(use_time))
            
        l=len(results)
        sorted_results=[] #存储排序好的数据
        #print(results[0].bbox.xyxy)
        t11 = time.time()
        #处理结果方框
        if l == 0:
            print('!no object!')
            print("Just show img.")
            #continue
        
        print("\n!!!#######OBJECTS############:{}".format(l))
        for i in range(l):
            t22 = time.time()
            xyxy = results[i].bbox.xyxy  
            #过滤不满足条件的结果
            conf = results[i].prob #置信度
            cls = int(results[i].id) #分类
            #if(cls != 1 ): #只关心0.苹果  1.橘子。默认状态下
            #    continue
            if(conf < 0.8):
                continue
                
            #看box范围是不是贴着摄像头边缘 10px
            if(is_obj_in_border(xyxy,cimg,th=10) == True):
                print("发现一个目标在图像边缘!",xyxy)
                continue
            
            # ========== 添加中心坐标计算和打印 ==========
            # 计算边界框中心坐标（像素坐标）
            x_center = (xyxy[0] + xyxy[2]) / 2
            y_center = (xyxy[1] + xyxy[3]) / 2
            print("!------------1个有效目标:%s"%(classname[cls]))
            print("!------------2D中心坐标: (%.1f, %.1f)像素, 置信度:%.3f"%(x_center, y_center, conf))
            print("!------------边界框: [%.1f, %.1f, %.1f, %.1f]"%(xyxy[0], xyxy[1], xyxy[2], xyxy[3]))
                 
            #从深度图计算xyz坐标
            xyz,thetaxy = [0,0]
            if(detectMode == 'seg'):
                xyz,thetaxy = get_one_XYZ(depth,results[i].mask,cam_param,cimg,usemask=True) 
            else:
                xyz,thetaxy = get_one_XYZ(depth,xyxy,cam_param,cimg) 

            # ========== 添加3D坐标打印 ==========
            print("!------------3D世界坐标: X=%.4fm, Y=%.4fm, Z=%.4fm"%(xyz[0], xyz[1], xyz[2]))
            print("!------------角度: theta_x=%.4f, theta_y=%.4f"%(thetaxy[0], thetaxy[1]))
            # ========== 坐标打印结束 ==========

            #打包信息
            datapack = resultinfo(results[i],xyz,thetaxy) 
            
            #队首插入排序
            sorted_results = do_target_sort(datapack,sorted_results)
            
            print("单次处理时间:%.3fs"%(time.time()-t22))
        print("##########OBJECTS_END#################\n")
        
        #判断2帧内是否有重合的目标，有则修正顺序 
        sorted_results = check_object_order(sorted_results,oldsort,r_th=25,s_th=0.1)
        oldsortdict[nowname] =  sorted_results #下帧处理前记录当前帧的目标排序
            
        #bbox画到输出图像
        t33=time.time()
        if(detectMode == 'seg'):
            drawbox2image(sorted_results,cimg,drawmask=True)
        else:
            drawbox2image(sorted_results,cimg)
        print("画所有结果时间:%.3fs"%(time.time()-t33))
        print("所有结果处理时间:%.3fs"%(time.time()-t11))
            
            
        if( outqueue.full()):
            outqueue.get()#此为阻塞函数
        outqueue.put([sorted_results,cimg,cname])#将结果写入输出队列。此为阻塞函数
            
        print('#############ALL Done. (%.3fs)###################' % (time.time() - t0))
                    
        if (showresult == True):
            #将深度信息转换成heatmap 合并输出
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
            #images = np.hstack((cimg_copy, depth_colormap))         
            images = cimg
            #showimg
            cv2.imshow("results", images)
            cv2.waitKey(1) 
    print("child process closed!")

def callback_campara(para,args):
    cam = args[0]
    #获取摄像机内参,只需要执行一次即可
    global cam_params,is_getcamparams
    if(is_getcamparams[cam] == False):
        cam_param = cam_intrinsics()
        cam_param.initByRosCaminfo(para.K,para.D,para.R,para.P)
        #转为字典
        cam_params[cam] = {}
        cam_params[cam]['K'] = cam_param.K
        cam_params[cam]['D'] = cam_param.D
        cam_params[cam]['R'] = cam_param.R
        cam_params[cam]['P'] = cam_param.P
        rospy.loginfo("\n-------------------------------------------\n" + \
                  "相机[{}] 内参 D,R,K,P:\n".format(cam) + \
                  "D:\n{}\nR:\n{}\n".format(cam_params[cam]['D'],cam_params[cam]['R']) + \
                  "K:\n{}\nP:\n{}\n".format(cam_params[cam]['K'],cam_params[cam]['P']) + \
                  "\n-------------------------------------------\n")
        is_getcamparams[cam] = True
    
         
#彩图压缩，深度图不压缩(realsen官方深度压缩数据有问题，只适用于1字节的深度数据,D435i为 2字节,不适用)
#15帧带宽占用:700K + 9M ≈ 10M左右
def callback_colorAnddepthcompressed(colorImage,depthImage):
    global frame_count,ttime
    global dataqueues,resultqueues
    global yolo5_pubs,yolo5_tf
    global isworking
    if not isworking.value:
        #print("isworking:",isworking.value)
        time.sleep(1)
        return
    #print("aaa")
    #识别相机ID
    #0.获取RGB摄像头帧名称
    cname = colorImage.header.frame_id
    camprefix = None
    for key in frame_count.keys():
        if cname.startswith(key):
            camprefix = key
    if camprefix is None:
        return
    count = frame_count[camprefix]
    t1 = ttime[camprefix]

    #获取对应的数据对象
    dataqueue = dataqueues[camprefix]
    resultqueue = resultqueues[camprefix]
    ##################1s实际帧数统计####################
    if(t1 != None):
      tdiff = time.time() - t1
      #print('\033[92m'+"Time diff:",tdiff)
      if(tdiff >= 1.0):
        framerate= int(count/tdiff)
        print("! {} :  1s 同步了 {} 帧".format(camprefix,framerate))
        t1 = time.time()
        count = 0
    else:
      t1 = time.time()
    count += 1
    #参数更新
    frame_count[camprefix] = count
    ttime[camprefix] = t1
    ########################################################
    tyy=time.time()
    
    #1.处理色彩和深度数据
    depth_image = np.frombuffer(depthImage.data, dtype=np.uint16).reshape(480, 640)
    color_image = np.frombuffer(colorImage.data, dtype=np.uint8)
    color_image = cv2.imdecode(color_image,cv2.IMREAD_COLOR)

    #2.写入新数据前先取出旧数据
    if( dataqueue.full()):
        t10 = time.time()
        dataqueue.get()#此为阻塞函数 
        #print("图像输入队列已满，丢弃老图。Time:",time.time()- t10)  
    dataqueue.put([color_image,depth_image,cname])#此为阻塞函数
    #print("img pre procee time:",time.time() - tyy)
    
    #3.获取YOLO5识别结果并发送
    if(not resultqueue.empty()):
        print("  {} 取到了1个处理结果".format(camprefix))
        ret=resultqueue.get()
        sorted_results = ret[0]
        rimg = ret[1]
        cname = ret[2]  #cname相当于到tensorrt进程走了一圈
        
        #发送数据到topic
        send_topic(yolo5_pubs[camprefix][0],sorted_results,cname)
        
        #发送结果图像到topic
        send_resultimg(yolo5_pubs[camprefix][1],rimg,cname,iscompress=False)
        send_resultimg(yolo5_pubs[camprefix][2],rimg,cname,iscompress=True)

        #广播物体实际坐标到tf
        send_tf(yolo5_tf,sorted_results,cname,camprefix)

#彩图不压缩，深度图不压缩  (###代码还未修改适配多相机)
#15帧带宽占用:12M + 9M ≈ 21M左右            
def callback_colorAnddepth(colorImage,depthImage):
    global frame_count,ttime  
    global dataqueues,resultqueues
    global yolo5_pubs,yolo5_tf
    global isworking
    if not isworking.value:
        #print("isworking:",isworking.value)
        time.sleep(1)
        return
    #print("aaa")
    #识别相机ID
    #0.获取RGB摄像头帧名称
    cname = colorImage.header.frame_id

    camprefix = None
    for key in frame_count.keys():
        if cname.startswith(key):
            camprefix = key
    if camprefix is None:
        return
    count = frame_count[camprefix]
    t1 = ttime[camprefix]

    #获取对应的数据对象
    dataqueue = dataqueues[camprefix]
    resultqueue = resultqueues[camprefix]
    ##################1s实际帧数统计####################
    if(t1 != None):
      tdiff = time.time() - t1
      #print('\033[92m'+"Time diff:",tdiff)
      if(tdiff >= 1.0):
        framerate= int(count/tdiff)
        print("! {} ： 1s 同步了 {} 帧".format(camprefix,framerate))
        t1 = time.time()
        count = 0
    else:
      t1 = time.time()
    count += 1
    #参数更新
    frame_count[camprefix] = count
    ttime[camprefix] = t1
    ########################################################
    tyy=time.time()
    #1.处理色彩和深度数据
    depth_image = np.frombuffer(depthImage.data, dtype=np.uint16).reshape(480, 640)
    color_image = np.frombuffer(colorImage.data, dtype=np.uint8).reshape(480, 640, 3)
    color_image = color_image[:,:,::-1]#RGB->BGR
    
    #2.写入新数据前先取出旧数据
    if( dataqueue.full()):
        t10 = time.time()
        dataqueue.get()#此为阻塞函数 
        #print("图像输入队列已满，丢弃老图。Time:",time.time()- t10)  
    dataqueue.put([color_image,depth_image,cname])#此为阻塞函数
    #print("img pre procee time:",time.time() - tyy)
    
    #3.获取YOLO5识别结果并发送
    if(not resultqueue.empty()):
        print("  {} 取到了1个处理结果".format(camprefix))
        ret=resultqueue.get()
        sorted_results = ret[0]
        rimg = ret[1]
        cname = ret[2]  #cname相当于到tensorrt进程走了一圈
        
        #发送数据到topic
        send_topic(yolo5_pubs[camprefix][0],sorted_results,cname)
        
        #发送结果图像到topic
        send_resultimg(yolo5_pubs[camprefix][1],rimg,cname,iscompress=False)
        send_resultimg(yolo5_pubs[camprefix][2],rimg,cname,iscompress=True)

        #广播物体实际坐标到tf
        send_tf(yolo5_tf,sorted_results,cname,camprefix)
    


#管理service
def yolo5_manage_service(req):
    global proc,isworking
    print(req.command)  #std_msgs.msg.string has param named data

    if(req.command == "sleep"):
        isworking.value = False
        return 100,"OK"
    elif(req.command == "recover"):
        isworking.value = True
        return 100,"OK"



#from tf.transformations import *
#############################################################################
def main():
    import tf #此处用的是python3的tf
    global frame_count,ttime
    global cam_params,is_getcamparams,showresult,wfname,categories
    global dataqueues,resultqueues
    global yolo5_pubs,yolo5_tf
    global proc,isalive,isworking

    #设置多进程模式
    #获取的对应的参数设置多进程的启动模式
    #fork报错使用spawn，一般x86平台用spwan ， aarch用fork
    process_mode = rospy.get_param('~process_mode', "fork")
    #cuda.init()
    import multiprocessing as mp
    mp.set_start_method(process_mode, force = True)
    #初始化数据传输的变量
    rospy.loginfo("Now Multiprocessing Mode: {}".format(mp.get_start_method()))

    ######################################################################
    #                      本程序数据节点部分                            #
    ######################################################################
    rospy.init_node('yolo5_detector') 
    rate = rospy.Rate(30)
    is_compressed_img = rospy.get_param('~compress_img', True)
    cam_num = rospy.get_param('~camera_num', 0)
    cam_prefix = rospy.get_param('~camera_prefix', "")
    cam_colors_f = rospy.get_param('~colors_frame', "")
    cam_colors_compress_f = rospy.get_param('~colors_compressed_frame', "")
    cam_depth_f = rospy.get_param('~depth_frame', "")
    cam_caminfo_f  = rospy.get_param('~caminfo_frame', "")

    cams_pxs=[]
    if(cam_num == 1):
        #初始化所有相机前缀
        cams_pxs=[cam_prefix,] 
        #初始化帧率计算参数
        frame_count[cam_prefix] = 0
        ttime[cam_prefix] = 0
        #初始化数据输入输出队列
        dataqueues[cam_prefix] = mp.Queue(3) #输入数据队列 >=2
        resultqueues[cam_prefix] = mp.Queue(3) #输入数据队列 >=2
        print(frame_count)
    else:
        for i in range(cam_num):
            #初始化所有相机前缀
            c= cam_prefix+str(i+1)
            cams_pxs.append(c)
            #初始化帧率计算参数
            frame_count[c] = 0
            ttime[c] = 0
            #初始化数据输入输出队列
            dataqueues[c] = mp.Queue(3) #输入数据队列 >=2
            resultqueues[c] = mp.Queue(3) #输入数据队列 >=2
    
    rospy.loginfo("!creating yolo5 topic nodes...")
     
    #为每个相机分别建立3个输出topic
    for p in cams_pxs:
        #发送识别到的对象信息的数据节点
        #PointCloud是为了兼容数据类型，不用额外定义msg类型。
        #数据类型定义:http://wiki.ros.org/sensor_msgs?distro=melodic  
        pub = rospy.Publisher('yolo5/'+ p +'/detected_objects', PointCloud, queue_size=1)
        pub_img = rospy.Publisher('yolo5/'+ p +'/resultimg',Image, queue_size=1)
        pub_img_cpd = rospy.Publisher('yolo5/'+ p + '/resultimg/compressed', CompressedImage, queue_size=1)
        yolo5_pubs[p]=[pub,pub_img,pub_img_cpd]

        

    #tf广播节点
    yolo5_tf = tf.TransformBroadcaster()
    
    ######################################################################
    #                            深度相机部分                            #
    ######################################################################
    #订阅相机参数节点，获取内参，用于定位识别的物体
    #注意，需要深度图对齐到rgb图
    #订阅每个相机的内参
    #cam_params = [0 for n in range(cam_num)] #初始化占位
    #is_getcamparam = [False for n in range(cam_num)] 
    
    csubs=[]
    for prefix in cams_pxs:
        is_getcamparams[prefix] = False #初始化
        camera_n="/" + prefix + cam_caminfo_f
        csub = rospy.Subscriber(camera_n, CameraInfo,callback_campara,(prefix,))
        csubs.append(csub)
        rospy.loginfo(camera_n)

    #等待所有相机内参全部获取完成
    while(True):
        ct = 0
        for p in cams_pxs: 
            b = is_getcamparams[p]
            if(b == True):
                ct += 1
        if(ct == cam_num):
            break
        else:
            rospy.loginfo("waiting for get ALL camera intrinsics info.")
            time.sleep(0.4) #wait get cam_paramK
    rospy.loginfo("Get ALL Camera Intrinsics Info Done")
    #注销掉内参节点订阅
    for u in csubs:
        u.unregister()

    #订阅ROS深度相机的2个数据节点，并将数据使用message_filters同步，1个回调函数
    if(is_compressed_img == False):
        #raw color data , raw depth data
        rospy.loginfo("using raw color data , raw depth data.")

        for id in range(cam_num):
            n="/" + cams_pxs[id] + cam_colors_f
            n1="/" + cams_pxs[id] + cam_depth_f
            scolor = message_filters.Subscriber(n, Image)
            sdepth = message_filters.Subscriber(n1, Image)
            #同步时间戳slop=间隔时间，单位秒。 queue_size=缓存大小
            sync = message_filters.ApproximateTimeSynchronizer([scolor,sdepth],queue_size=1,slop=0.0001)
            sync.registerCallback(callback_colorAnddepth)
    elif(is_compressed_img == True):
        #compressed color data , raw depth data
        rospy.loginfo("using compressed color data , raw depth data.")
        for id in range(cam_num):
            n="/" + cams_pxs[id] + cam_colors_compress_f
            n1="/" + cams_pxs[id] + cam_depth_f
            scolor = message_filters.Subscriber(n, CompressedImage)
            sdepth = message_filters.Subscriber(n1, Image)
            #同步时间戳slop=间隔时间，单位秒。 queue_size=缓存大小
            sync = message_filters.ApproximateTimeSynchronizer([scolor,sdepth],queue_size=1,slop=0.0001)
            sync.registerCallback(callback_colorAnddepthcompressed)

    ######################################################################
    #                            YOLO5控制服务初始化部分                   #
    ######################################################################
    
    rospy.Service('/yolo5/controlservice', ControlCmd, yolo5_manage_service)
    rospy.loginfo("ROS Done.")
    ######################################################################
    #                            YOLO5tensorRT进程初始化部分              #
    ######################################################################
    

    
    '''
    cam_PARAM={}  
    cam_PARAM['K']=cam_param.K
    cam_PARAM['D']=cam_param.D
    cam_PARAM['R']=cam_param.R
    cam_PARAM['P']=cam_param.P
    '''
    #启动YOlO5 tensorRT 进程
    rospy.loginfo("start TensorRT Process!!!")

    isalive = mp.Value('b', True) #通过value对象完成进程关闭控制
    isworking = mp.Value('b', False) #通过value对象完成进程休眠控制

    proc = mp.Process(target = PredictionProcess,args=(isalive,isworking,dataqueues,resultqueues,cam_params,categories,pluginlibname,wfname))
    proc.start()
    isworking.value = True   #休眠控制测试
    
    rospy.spin()


def myshutdown():
    global isalive
    #先给子进程发送关闭指令
    isalive.value = False

    while not rospy.is_shutdown():
        rospy.signal_shutdown("close!")
        import signal
        os.kill(os.getpid(),signal.SIGKILL) #suicide


if __name__ == '__main__':
    try:
        rospy.loginfo("cv2 version:{}".format(cv2.__version__))
        
        rospy.on_shutdown(myshutdown) #注册退出回调,用于彻底杀掉自己
        main()
        print("END!")
    except rospy.ROSInterruptException:
        pass
