#!/usr/bin/python3
#coding:utf-8

import ctypes
import rospy
import os  
import time
import sys
import math
import numpy as np
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo,PointCloud,ChannelFloat32
from geometry_msgs.msg import Point32

from numpy import random

#####################################
###                from yolo5_v6.0
#####################################
class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))
        
colors = Colors()  # create instance for 'from utils.plots import colors'

def is_ascii(s=''):
    # Is string composed of all ASCII (no UTF) characters? (note str().isascii() introduced in python 3.7)
    s = str(s)  # convert list, tuple, None, etc. to str
    return len(s.encode().decode('ascii', 'ignore')) == len(s)




#draw bbox to image
class Annotator:
    # YOLOv5 Annotator for train/val mosaics and jpgs and detect/hub inference annotations
    def __init__(self, im, line_width=None, font_size=None, font='Arial.ttf', pil=False, example='abc'):
        assert im.data.contiguous, 'Image not contiguous. Apply np.ascontiguousarray(im) to Annotator() input images.'
        non_ascii = not is_ascii(example)  # non-latin labels, i.e. asian, arabic, cyrillic
        self.pil = pil or non_ascii
        if self.pil:  # use PIL
            self.im = im if isinstance(im, Image.Image) else Image.fromarray(im)
            self.draw = ImageDraw.Draw(self.im)
            self.font = check_pil_font(font='Arial.Unicode.ttf' if non_ascii else font,
                                       size=font_size or max(round(sum(self.im.size) / 2 * 0.035), 12))
        else:  # use cv2
            self.im = im
        self.lw = line_width or max(round(sum(im.shape) / 2 * 0.003), 2)  # line width

    def box_label(self, box, label='', color=(128, 128, 128), txt_color=(255, 255, 255)):
        # Add one xyxy box to image with label
        if self.pil or not is_ascii(label):
            self.draw.rectangle(box, width=self.lw, outline=color)  # box
            if label:
                w, h = self.font.getsize(label)  # text width, height
                outside = box[1] - h >= 0  # label fits outside box
                self.draw.rectangle(
                    (box[0], box[1] - h if outside else box[1], box[0] + w + 1,
                     box[1] + 1 if outside else box[1] + h + 1),
                    fill=color,
                )
                # self.draw.text((box[0], box[1]), label, fill=txt_color, font=self.font, anchor='ls')  # for PIL>8.0
                self.draw.text((box[0], box[1] - h if outside else box[1]), label, fill=txt_color, font=self.font)
        else:  # cv2
            p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
            cv2.rectangle(self.im, p1, p2, color, thickness=self.lw, lineType=cv2.LINE_AA)
            if label:
                tf = max(self.lw - 1, 1)  # font thickness
                w, h = cv2.getTextSize(label, 0, fontScale=self.lw / 3, thickness=tf)[0]  # text width, height
                outside = p1[1] - h - 3 >= 0  # label fits outside box
                p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
                cv2.rectangle(self.im, p1, p2, color, -1, cv2.LINE_AA)  # filled
                cv2.putText(self.im,
                            label, (p1[0], p1[1] - 2 if outside else p1[1] + h + 2),
                            0,
                            self.lw / 3,
                            txt_color,
                            thickness=tf,
                            lineType=cv2.LINE_AA)

    def rectangle(self, xy, fill=None, outline=None, width=1):
        # Add rectangle to image (PIL-only)
        self.draw.rectangle(xy, fill, outline, width)

    def text(self, xy, text, txt_color=(255, 255, 255)):
        # Add text to image (PIL-only)
        w, h = self.font.getsize(text)  # text width, height
        self.draw.text((xy[0], xy[1] - h + 1), text, fill=txt_color, font=self.font)

    def result(self):
        # Return annotated image as array
        return np.asarray(self.im)
###################################
###                 END
###################################


#获取yolo5根目录
def get_yolo5_rootPath():
    dirname, filename = os.path.split(os.path.abspath(sys.argv[0]))
    dirname = dirname + "/yolo5"
    return dirname


#识别的目标是否在图像边缘  #默认阈值为离4个边界10个像素
def is_obj_in_border(xyxy,cimg,th=10):
    #2.再看box范围是不是贴着摄像头边缘
    sw=cimg.shape[1]
    sh=cimg.shape[0]
    #左上角点和左上边缘的间距 px
    delta_x1 = xyxy[0]
    delta_y1 = xyxy[1]
    #右下角点和右下边缘的间距 px
    delta_x2 = sw - xyxy[2]
    delta_y2 = sh - xyxy[3] 
    #根据间距判断bbox是否处在摄像机视角图像边缘，
    #是就过滤掉，防止由于视角限制导致识别的目标不完整
    delta_offx = th #单位px
    delta_offy = th
    #处于边缘返回True
    if(delta_x1 < delta_offx or delta_x2 < delta_offx or delta_y1 < delta_offy or delta_y2 < delta_offy):
        return True



#获取目标中心点的xyz坐标
#默认indata输入bbox的xyxy，表示直接从bbox获取目标中点
#若usemask使能，则indata输入mask[shape(480,640)]，表示根据mask计算目标中点
def get_one_XYZ(ddata,indata,camparam,timg=np.array([0],dtype="int"),usemask=False):
    #摄像头畸变系数
    D = camparam.D
    #摄像头内参K
    fx = camparam.K[0,0]
    fy = camparam.K[1,1]
    ppx = camparam.K[0,2]
    ppy = camparam.K[1,2]
    #print("ppx....:",fx,fy,ppx,ppy)

    mid_pos=[0.0,0.0] #目标中点
    distance_list=[] #目标深度值候选队列

    if(usemask == False):
        xyxy=indata
        #确定目标bbox 中心像素位置,左上角和右下角相加在/2
        mid_pos= [int((int(xyxy[0]) + int(xyxy[2])) / 2), int((int(xyxy[1]) + int(xyxy[3])) / 2)]
        
        #确定选取深度值的范围(bbox范围内),放缩系数0.5
        rw = abs(int(xyxy[2]) - int(xyxy[0])) * 0.5
        rh = abs(int(xyxy[3]) - int(xyxy[1])) * 0.5
        
        #随机在box中心点附近采样计算距离
        randnum = 40  #选取参考点，用于计算最终距离
        biash = 0
        biasw = 0
        for i in range(randnum):
            #realsensor 畸变很小，所以尝试不用去畸变计算目标距离
            #根据点的深度值获取距离
            dist = ddata[int(mid_pos[1] + biash), int(mid_pos[0] + biasw)] * 0.001 # mm -> m
            #0值处理，非0存储，0丢弃 
            if(dist != 0.0):#非0值存储
                if(timg.shape[0] != 1):   
                    #把采样点都画上去
                    poss=np.array(mid_pos)
                    poss[1]+=biasw
                    poss[0]+=biash       
                    cv2.circle(timg,tuple(poss.astype("int")),3,(130,130,0),1)

                distance_list.append(dist)
            #else: 
            #    print("!!!distance list has 0.")
                
            #生成随机偏移 
            biasw = random.randint(-rw // 4, rw // 4) 
            biash = random.randint(-rh // 4, rh // 4) 

        #画bbox的中心点
        cv2.circle(timg,tuple(mid_pos),3,(0,255,0),1)

    elif(usemask == True):
        #1.获取mask的最大轮廓
        mask = np.array(indata,dtype='uint8')
        #cv2.imwrite("/home/robuster/1.jpg",mask*255)
        contours, _ = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=cv2.contourArea, reverse=True) #True:从大到小排序  False:从小到大排序
        
        #print("\nlunkuo:",contours[0].shape,contours[0])
        #print("X:",contours[0][:,0,0])
        #print("Y:",contours[0][:,0,1])
        #2.通过轮廓求取质心
        M = cv2.moments(contours[0])#选择最大的轮廓
        #print(M)
        if(M["m00"] == 0.0):
            print("invaild contours data.")
            return [0.0,0.0,0.0],[0.0,0.0]
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        mid_pos = [cX,cY]
        #画质心
        cv2.circle(timg, mid_pos, 5, (130,255,130), -1)
        #3.通过mask获取到物体的深度值
        depthdata=ddata*mask #通过mask掩膜提取对应区域的深度值
        distance_list = depthdata[depthdata > 0.0] * 0.001 #提取大于0的深度点 并将距离单位mm -> m

    if(len(distance_list) == 0):#无法获得有效距离
        print("invaild distance!!")
        return [0.0,0.0,0.0],[0.0,0.0]

    distance_list = np.array(distance_list,dtype="float64")
    #排序,然后去掉两头的部分值
    distance_list = np.sort(distance_list)#冒泡排序
    #砍掉头尾各20%的数据
    l =len(distance_list)
    ll = l *0.2
    s= int(ll)
    e= int(l - ll)
    distance_list = distance_list[s:e]

    if(len(distance_list) == 0):
        print("invaild start[%d] end[%d]?"%(s,e))
        print(len(distance_list))
        print(distance_list)
        return [0.0,0.0,0.0],[0.0,0.0]

    #异常值处理  传入数组不能为空
    distance_list = irr_value_check(distance_list, th =0.01)  #数组内允许浮动的最大值0.01m
    
    #print("dlist:\n",distance_list)   
    #均值为最终距离 real_z  
    real_z = np.mean(distance_list)  
    real_x = 0
    real_y = 0        
    if(real_z != 0.0):
        #去摄像头畸变
        pts = np.zeros((1,1,2), dtype=np.float32) #(个数,1,xy坐标)
        pts[0][0]=([int(mid_pos[0]), int(mid_pos[1])]) #x,y
        undistort_pts = cv2.undistortPoints(pts, camparam.K, camparam.D, P=camparam.K) #对点去畸变
        print("old_center:{}  new_center:{}".format(pts,undistort_pts))
        mid_pos = undistort_pts[0][0]
        #计算所有点的3D坐标
        real_x = (mid_pos[0] - ppx) / fx * real_z
        real_y = (mid_pos[1] - ppy) / fy * real_z
        real_z = real_z
        #print(real_x,real_y)
        #debug
        #real_z = 0.25  #相机坐标系，平行抓取时，固定抓取深度，用于检测定位点和抓取点是否一致
        #real_y -= 0.07   #相机坐标系，平行抓取时，y轴升高7cm ，检测x轴的偏移
        
    
    theta_x = math.atan(real_x / real_z) #偏转角度x
    theta_y = math.atan(real_y / real_z) #偏转角度y
    return [real_x,real_y,real_z],[theta_x,theta_y]



def irr_value_check(distance_np, th =0.02):
    '''
    1.记录数组的最大值和最小值。
    2.判断最大值和最小值的差值是否超过阈值(默认0.02m)
    #否:数组无需处理
    #是:数组中所有值分别减去最大值和最小值，然后分别求和，取和最小的作为基准值，过滤不符合条件的距离。
        然后输出新数组
    '''
    vmax=np.max(distance_np)
    vmin=np.min(distance_np)
    vdiff = np.abs(vmax -vmin)
    if(vdiff <= th):#否
        return distance_np
    #是
    dmax = np.sum(np.abs(distance_np - vmax))
    dmin = np.sum(np.abs(distance_np - vmin))
    #print("!!!dmax-min:",dmax,dmin)
    if(dmax <= dmin):
        #print("vmax:",vmax)
        #dmax为准,过滤超过 dmax - th 范围的点
        distance_np = distance_np - (vmax-th) 
        #print("emm:",distance_np)
        return distance_np[distance_np > 0] + (vmax -th) #还原   
    else:
        #print("vmin:",vmin)
        #dmin为准,过滤超过 dmin +th范围的点
        distance_np = (vmin+th) - distance_np
        #print("emm:",distance_np)
        return (vmin + th) - distance_np[distance_np > 0] #还原   
    
    
    

#将yolo5结果排序后打包发送到topic
def send_topic(pub,yolo5data,cname):
    data=PointCloud()
    data.header.stamp = rospy.Time.now() #时间戳
    data.header.frame_id = cname #相机rgb摄像头前缀
    link_index = 0
    for i in yolo5data:
        link_index += 1
        #xyz实际坐标封包
        p = Point32(i.x, i.y, i.z)
        data.points.append(p)
        #标签和link(frame)信息封包,link信息和tf的link name对应 link-1 link-2 ......
        info = ChannelFloat32(name=i.labelname,values=[i.labelnum,link_index,i.prob])
        data.channels.append(info)
        
    ''' example data  
    #Point32存储坐标xyz
    p1=Point32(1.1,2.2,3.3)
    p2=Point32(4.4,5.5,6.6)
    p3=Point32(7.7,8.85,9.95)
    a.points = [p1,p2,p3]
    #使用ChannelFloat32的name传送tf的标签的名字,value传送link的序列号
    b1=ChannelFloat32(name="apple",values=[1.0,])
    b2=ChannelFloat32(name="cellphone",values=[2.0,])
    b3=ChannelFloat32(name="panda",values=[3.0,])
    a.channels=[b1,b2,b3]
    '''  
    pub.publish(data)

#画mask
def draw_mask(masks, colors_, im_src, alpha=0.5):
        """
        description: Draw mask on image ,
        param: 
            masks  : result_mask    shape(480,640)
            colors_: color to draw mask   shape(3,)
            im_src : original image    shape(480,640,3)
            alpha  : scale between original  image and mask
        return:
            no return
        """
        if len(masks) == 0:
            return
        masks = np.asarray(masks, dtype=np.uint8)
        masks = np.expand_dims(masks,axis=0)#添加一个行维度

        masks = np.ascontiguousarray(masks.transpose(1, 2, 0))
        masks = np.asarray(masks, dtype=np.float32)
        colors_ = np.asarray(colors_, dtype=np.float32)
        s = masks.sum(2, keepdims=True).clip(0, 1)
        #print(masks.shape,colors_.shape,im_src.shape)
        colors_ = np.expand_dims(colors_,axis=0)#添加一个行维度
        masks = (masks @ colors_).clip(0, 255)
        im_src[:] = masks * alpha + im_src * (1 - s * alpha)


#将需要的yolo5结果画到图像上
#直接对图像内存操作，无需返回画好的图像
def drawbox2image(yolo5data,img,drawmask=False):
    #画图
    annotator = Annotator(img, line_width=1)
    for i in range(len(yolo5data)):
        #画框
        label_xyz = '%d-%.2f:(%.4f,%.4f,%.4f),(%.2f,%.2f)'%(yolo5data[i].labelnum,yolo5data[i].prob,yolo5data[i].x,yolo5data[i].y,yolo5data[i].z,yolo5data[i].thx,yolo5data[i].thy)
        if(i == 0): #第一个目标为锁定的目标
            annotator.box_label(yolo5data[i].xyxy, label_xyz, color=(0,255,0)) #green
        else:
            annotator.box_label(yolo5data[i].xyxy, label_xyz, color=(0,0,255)) #red
        if(drawmask == True):
            t33=time.time()
            draw_mask(yolo5data[i].mask, colors_=colors(int(yolo5data[i].labelnum), True),im_src=img)
            print("画mask时间:%.3fs"%(time.time()-t33))
        #按类别选择颜色
        #annotator.box_label(yolo5data[i].xyxy, label_xyz, color=colors(int(yolo5data[i].labelnum), True))
    

#话题句柄+要输出的图像+rgb摄像头帧名称
#发送识别结果图片
def send_resultimg(pub,img,cname,iscompress=False):
    from cv_bridge import CvBridge, CvBridgeError #注意此为手动编译的noetic版本的cv_bridge
    data = None
    if(iscompress):
        #data=CompressedImage()
        CvB = CvBridge()
        data = CvB.cv2_to_compressed_imgmsg(img)
        data.header.stamp = rospy.Time.now() #时间戳
        data.header.frame_id = cname #frame前缀
        #print(data)
        #color_image = CvB.compressed_imgmsg_to_cv2(color_image,"bgr16")
    else:
        #封装Image
        data=Image()
        data.header.stamp = rospy.Time.now() #时间戳
        data.header.frame_id = cname #frame前缀
        data.height = img.shape[0]
        data.width = img.shape[1]
        data.encoding = "rgb8"
        data.is_bigendian = 0   #如果不是nx下运行,注意检查大小端
        data.step = 1920
        img = img[:,:,::-1]#BGR->RGB
        data.data = img.tobytes()
    
    #发送
    pub.publish(data)
    
#发送一个tf
def send_tf(tfbr,yolo5data,cname,prefix):
    import tf
    target_len = len(yolo5data)
    if target_len == 0:
        return
        
    #frame序号只发送1
    print("send tf_Len:", target_len)

    link_index = 1
    tfbr.sendTransform((yolo5data[0].x, yolo5data[0].y, yolo5data[0].z), #xyz位移 相对父frame
                    tf.transformations.quaternion_from_euler(0, 0, 0),#旋转四元数坐标，目前为0
                    rospy.Time.now(),#时间戳
                    "yolo5_obj_"+prefix+"_" +str(link_index),#自己frame
                    cname) #父frame


#根据yolo5得出的结果的置信度以及距离的远近进行排序
#近在前，远在后

def do_target_sort(data,ret=[]):
    #ret为空则直接添加返回
    if(len(ret) == 0):
        ret.append(data)
        return ret
    data_prob = data.prob
    data_xyxy = data.xyxy
    data_mid_pos= [int((int(data_xyxy[0]) + int(data_xyxy[2])) / 2), int((int(data_xyxy[1]) + int(data_xyxy[3])) / 2)]#box中点
    
    for i in range(len(ret)):
        ret_prob = ret[i].prob
        ret_xyxy = ret[i].xyxy
        ret_mid_pos= [int((int(ret_xyxy[0]) + int(ret_xyxy[2])) / 2), int((int(ret_xyxy[1]) + int(ret_xyxy[3])) / 2)]#box中点
        
        '''
                  排序策略
        1.置信度最高的最优先
        2.物体到相机的距离最近的优先
        
        '''
        pdiff = data_prob - ret_prob
        ddiff = data.z - ret[i].z
        if(pdiff > 0.1):#置信度高出0.1以上直接插入
            ret.insert(i, data)#在i位插入,原数据后移
            return ret
            
        elif(abs(pdiff) <=0.1):#置信度处在0.1区间(-0.1 <-> 0.1)，继续判断物体到相机的距离
            if(ddiff <= 0): #距离小时插入
                ret.insert(i, data) #在i位插入
                return ret
        
    #遍历完后不满足所有条件则添加到最后        
    ret.append(data)
    return ret
    
    
#为了避免两个目标距离跳变导致输出目标跳来跳去的现象，基于2帧添加弱追踪能力。
#     目标稳定识别的情况下。缓慢移动的物体可以稳定追踪
#若2个bbox的2帧位置大致相同，则参考上一帧的顺序调整输出。
def check_object_order(nowsort=[],oldsort=[],r_th=5,s_th=0.1):
    #r_th: 半径阈值
    #s_th: 面积差阈值
    ln=len(nowsort)
    lo=len(oldsort)
    #print("emm:(%d,%d)"%(lo,ln))
    if( ln== 0 or lo == 0):
        return nowsort
        
    newsort = [0 for x in range(0,ln)] #初始化一个list数组
    order = np.empty((ln,),dtype="int") #记录输出新顺序
    order.fill(-1)
    complete_list=[] #当前帧已经排过序的点
    
    #先按上一帧的顺序找一下有没有重合的bbox
    oldindex=0
    for old_i in range(lo):
        #计算bbox中心点和面积
        xyxy = oldsort[old_i].xyxy
        old_c = [int((int(xyxy[0]) + int(xyxy[2])) / 2), int((int(xyxy[1]) + int(xyxy[3])) / 2)]#box中点(x,y)
        old_s = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
        #使用中点为原点(old_c)和阈值为半径(p_th)建立圆方程
        for now_i in range(ln):
            if now_i in complete_list :
                #print("PASS this position:{}".format(now_i))
                continue
            #计算bbox中心点和面积
            xyxy = nowsort[now_i].xyxy
            now_c = [int((int(xyxy[0]) + int(xyxy[2])) / 2), int((int(xyxy[1]) + int(xyxy[3])) / 2)]#box中点(x,y)
            #已知xy和圆心坐标，求半径
            #y=sqrt(r^2 -(x-a)^2) + b
            x = now_c[0] - old_c[0]
            y = now_c[1] - old_c[1]
            r = np.sqrt(x*x +y*y)

            #判断新点是否在半径阈值范围内
            if(r <= r_th):  
                #判断面积差是否在阈值范围内
                now_s = (xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1])
                diff_s = abs(now_s - old_s) / old_s #  下一帧和当前帧面积差的比例
                #print("diff_s:",diff_s)
                if(diff_s <= s_th):
                    #符合条件说明2个目标匹配，则将老顺序覆盖到新顺序中
                    #print("old-new:",old_c,now_c)

                    complete_list.append(now_i) #下轮循环不会再看这个点
                    order[now_i] = oldindex #保存编号 #当前帧列表的第now_i号为oldindex
                    #print(order)
                    oldindex += 1
                    break #相同位置的bbox只能存在一个
    
    nowindex=np.max(order)
    if(nowindex != -1): #不为-1说明顺序有改动，为-1则说明顺序无改动
        #补充完整的顺序
        for new_i in range(ln):
            #print(nowindex)
            if(order[new_i] == -1):
                nowindex +=1 
                order[new_i] = nowindex
            #重组当前帧的排序
            pos = order[new_i]
            #print("{} newsort-{}:{} nowsort-{}:{}".format(order,len(newsort),pos,len(nowsort),new_i))
            newsort[pos] = nowsort[new_i]
            
    else:
        newsort = nowsort
    return newsort
