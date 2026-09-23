import numpy as np
#相机内参

class cam_intrinsics:  
    K = np.array([[0.0,0.0,0.0],[0.0, 0.0, 0.0],[0.0,0.0,1.0]])
    D = np.array([0.0,0.0,0.0, 0.0])
    R = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
    P = np.array([
        [0.0,0.0,0.0,0.0],
        [0.0,0.0,0.0,0.0],
        [0.0,0.0,1.0,0.0]])

    
    #init
    def initByCaminfo(self,caminfo=None):
        if(caminfo != None):
            self.K=caminfo.K
            self.D=caminfo.D
            self.R=caminfo.R
            self.P=caminfo.P
    
    #通过ros获取的realsensor的摄像头内参初始化
    def initByRosCaminfo(self,K=None,D=None,R=None,P=None):
        if(K!=None):
            K = np.array(K,dtype=np.float64)
            self.K[0] = K[0:3]
            self.K[1] = K[3:6]
            self.K[2] = K[6:9]
        if(D!=None):
            self.D = np.array(D,dtype=np.float64)
        if(R!=None):
            R = np.array(R,dtype=np.float64)
            self.R[0] = R[0:3]
            self.R[1] = R[3:6]
            self.R[2] = R[6:9]
        if(P!=None):
            P = np.array(P,dtype=np.float64)
            self.P[0] = P[0:4]
            self.P[1] = P[4:8]
            self.P[2] = P[8:12]


#opencv标定的内参
class cam_intrinsics1:  
    K = np.array([[
              585.2138764517942, 0.0, 313.6354339001962
          ],
          [
              0.0, 587.1675397373571, 255.45084513727065
          ],
          [
            0.0,
            0.0,
            1.0
          ]])
    D = np.array([
        0.11894169146956333, -0.26539759085133463, 0.0017711027787027439, -0.0005668001720892185, 0.0
          ])
    R = np.array([
                    [1.0,0.0,0.0],
                    [0.0,1.0,0.0],
                    [0.0,0.0,1.0]
                    ])

    P = np.array([
        [
            591.3494262695312, 0.0, 312.7006483234727, 0.0
        ],
        [
            0.0, 593.7122192382812, 255.49506833631676, 0.0
        ],
        [
            0.0,
            0.0,
            1.0,
            0.0
        ]
        ])
