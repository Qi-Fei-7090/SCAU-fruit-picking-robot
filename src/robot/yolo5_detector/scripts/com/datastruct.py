


class bbox2d:
    xyxy=[0.0,0.0,0.0,0.0]

class result:
    label=" "
    id=-1
    prob=0.0
    bbox=0
    mask=None
    def __init__(self):
        self.bbox = bbox2d() #不能静态创建



#YOLO5返回结果信息存储结构
class resultinfo():
    xyxy = 0 #bbox区域
    labelname = "" #标签名字
    labelnum = 0 #标签序号
    prob = 0 #置信度
    mask = None #mask
    #实际3D坐标 
    x = 0
    y = 0
    z = 0  
    #相对相机的偏转角度
    thx =0
    thy = 0
    def __init__(self,yolo5Presult,position=[0,0,0],rxy=[0.0,0.0]):
        self.xyxy = yolo5Presult.bbox.xyxy
        self.labelname = yolo5Presult.label
        self.labelnum = yolo5Presult.id
        self.prob = yolo5Presult.prob
        self.mask = yolo5Presult.mask
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self.thx = rxy[0]
        self.thy = rxy[1]