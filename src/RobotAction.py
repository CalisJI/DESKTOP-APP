import sys
import os
current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "Module"
module_path = os.path.join(current_dir, relative_path)
sys.path.append(module_path)

# import modeul frrpc from path file src\Module\frrpc.pyd
import frrpc
import time
class RobotAction:

    def __init__(self):
        self.robot = frrpc.RPC('192.168.58.2')
        ret = self.robot.GetSDKVersion()    # Query SDK version number
        if ret[0] == 0:
            # 0-No fault, return format:[errcode,data],errcode-Fault code,data-Data
            print("SDK version is:",ret[1])
            print("Connected")
        else:
            print("the errcode is: ", ret[0])

    def Change_Mode(self, mode):
        # 0: Autom Mode, 1: Manual Mode 2: Teaching Mode 3: Non Teaching Mode
        if mode == 1:
            self.robot.Mode(1)
            time.sleep(1)
            self.robot.DragTeachSwitch(1) #When the robot enters the drag teaching mode, it can only enter the drag teaching mode in manual mode
            time.sleep(1)
            ret = self.robot.IsInDragTeach()
            if ret[0] == 0:
                print("Drag State Is:", ret[1])
            else:
                print("the errcode is: ", ret[0])
        elif mode == 2:
            self.robot.DragTeachSwitch(0)
            time.sleep(1)
            ret = self.robot.IsInDragTeach()
            if ret[0] == 0:
                print("Drag State Is:", ret[1])
            else:
                print("the errcode is: ", ret[0])
    
    def RobotEnable(self):
        self.robot.RobotEnable(0) # 0: Enable 1: Disable
        time.sleep(3)
        self.robot.RobotEnable(1) # 0: Enable 1: Disable
    
    def Obtain_Controller_IP(self):
        ret = self.robot.GetControllerIP()
        if ret[0] == 0:
            print("Controller IP Is:", ret[1])
        else:
            print("the errcode is: ", ret[0])

#region Robot Motion

    def Start_JOG(self,ref:int,nb:int,dir:int,vel:int,acc:int,max_dis:float):
        """
        Returns [0] Success [errorcode0] Failed
        -------
        ref:0-joint jogging, 2-base coordinate system jogging, 4-tool coordinate system jogging, 8-workpiece coordinate system jogging;

        nb:1-1joint(x-axis), 2-2joint(y-axis), 3-3join(z-axis), 4-4joint(rx), 5-5joint (ry), 6-6joint(rz);

        dir:0-negative direction, 1-positive direction;

        el:Speed percentage,[0~100];

        acc:Acceleration percentage,[0~100];

        max_dis:Maximum angle/distance for a single jog,unit[° or mm]
        """
        return self.robot.StartJOG(ref,nb,dir,vel,acc,max_dis)
    def Stop_Jog(self,ref:int):
        """
        deceleratioin stops
        ref:1-joint jog stop,
            3-base coordinate system jog stop, 
            5-tool coordinate system jog stop, 
            9-workpiece coordinate system jog stop
        """
        return self.robot.StopJOG(ref)
    
    def ImmStopJog(self):
        """
        jog jog immediately stops
        """
        return self.robot.ImmStopJOG()

    def MoveJ(self,joint_pos,desc_pos,tool,user,vel,acc,ovl,exaxis_pos,blendT,offset_flag,offset_pos):
        """
        Returns [0] Success [errorcode0] Failed
        -------
        joint_pos:Target joint position, unit[°];
        desc_pos:Target Cartesian pose,unit[mm][°];
        tool:Tool number,[0~14];
        user:Workpiece number,[0~14];
        vel:Speed percentage,[0~100];
        acc:Acceleration percentage,[0~100],temporarily closed;
        ovl:Speed scaling factor,[0~100];
        exaxis_pos:External axis 1 position to external axis 4 position;
        blendT:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked),unit[ms];
        offset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;
        offset_pos:Pose offset,unit[mm][°]
        """

        #region Example
        
            # import frrpc
            # import time
            # # A connection is established with the robot controller. A successful connection returns a robot object
            # robot = frrpc.RPC('192.168.58.2')
            # J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
            # P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
            # eP1=[0.000,0.000,0.000,0.000]
            # dP1=[1.000,1.000,1.000,1.000,1.000,1.000]
            # J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
            # P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
            # eP2=[0.000,0.000,0.000,0.000]
            # dP2=[1.000,1.000,1.000,1.000,1.000,1.000]
            # robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #Joint space motionPTP,Tool number1,the actual test is based on field data and Tool number
            # robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
            # time.sleep(2)
            # j1 = robot.GetInverseKin(0,P1,-1)       #In the case of Cartesian space coordinates only, the inverse kinematic interface can be used to solve the joint position
            # print(j1)
            # j1 = [j1[1],j1[2],j1[3],j1[4],j1[5],j1[6]]
            # robot.MoveJ(j1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
            # j2 = robot.GetInverseKin(0,P2,-1)
            # print(j2)
            # j2 = [j2[1],j2[2],j2[3],j2[4],j2[5],j2[6]]
            # robot.MoveJ(j2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
            # time.sleep(2)
            # p1 = robot.GetForwardKin(J1)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
            # print(p1)
            # p1 = [p1[1],p1[2],p1[3],p1[4],p1[5],p1[6]]
            # robot.MoveJ(J1,p1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
            # p2 = robot.GetForwardKin(J2)
            # print(p2)
            # p2 = [p2[1],p2[2],p2[3],p2[4],p2[5],p2[6]]
            # robot.MoveJ(J2,p2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
        
        #endregion
        return self.robot.MoveJ(joint_pos,desc_pos,tool,user,vel,acc,ovl,exaxis_pos,blendT,offset_flag,offset_pos)

    #Linear motion in Cartesian space
    def MoveL(self,joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,exaxis_pos,search,offset_flag,offset_pos):
        """
            Linear motion in Cartesian space

            joint_pos:Target joint position, unit[°];

            desc_pos:Target Cartesian pose,unit[mm][°];

            tool:Tool number,[0~14];

            user:Workpiece number,[0~14];

            vel:Speed percentage,[0~100];

            acc:Acceleration percentage,[0~100],temporarily closed;

            ovl:Speed scaling factor,[0~100];

            blendR:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked),unit[mm];

            exaxis_pos:Position of external axis 1~position of external axis 4;

            search:[0]-non welding wire positioning, [1]-welding wire positioning;

            offset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;

            offset_pos:Pose offset,unit[mm][°]
        """

            # import frrpc
            # # A connection is established with the robot controller. A successful connection returns a robot object
            # robot = frrpc.RPC('192.168.58.2')
            # J1=[95.442,-101.149,-98.699,-68.347,90.580,-47.174]
            # P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
            # eP1=[0.000,0.000,0.000,0.000]
            # dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
            # J2=[123.709,-121.190,-82.838,-63.499,90.471,-47.174]
            # P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
            # eP2=[0.000,0.000,0.000,0.000]
            # dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
            # J3=[167.066,-95.700,-123.494,-42.493,90.466,-47.174]
            # P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
            # eP3=[0.000,0.000,0.000,0.000]
            # dP3=[0.000,0.000,0.000,0.000,0.000,0.000]
            # robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1 ,dP1)   #Rectilinear motion in Cartesian space
            # robot.MoveL(J2,P2,0,0,100.0,180.0,100.0,-1.0,eP2,0,0,dP2)
            # robot.MoveL(J3,P3,0,0,100.0,180.0,100.0,-1.0,eP3,0,0,dP3)
        return self.robot.MoveL(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,exaxis_pos,search,offset_flag,offset_pos)

    def MoveC(self,joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,poffset_flag,offset_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t ,toffset_flag,offset_pos_t,ovl,blendR):
        
        """ Circular arc motion in Cartesian space
        
            joint_pos_p:Path point joint position,unit[°];

            desc_pos_p:Path point Cartesian pose,unit[mm][°];

            ptool:Tool number,[0~14];

            puser:Workpiece number,[0~14];

            pvel:Speed percentage,[0~100];

            pacc:Acceleration percentage,[0~100],temporarily closed;

            exaxis_pos_p:Position of external axis 1~position of external axis 4;

            poffset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;

            offset_pos_p:Offset,unit[mm][°];

            joint_pos_t:Target point joint position,unit[°];

            desc_pos_t:Cartesian pose of the target point,unit[mm][°];

            ttool:Tool number,[0~14];

            tuser:Workpiece number,[0~14];

            tvel:Speed percentage,[0~100];

            tacc:Acceleration percentage,[0~100],temporarily closed;

            exaxis_pos_t:Position of external axis 1~position of external axis 4;

            toffset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;

            offset_pos_t:Offset,unit[mm][°]。

            ovl:Speed scaling factor,[0~100];

            blendR:[-1.0]-motion in place (blocked), [0-1000]-smooth radius(non blocked),unit[mm]
        """

        self.robot.MoveC(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,poffset_flag,offset_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t ,toffset_flag,offset_pos_t,ovl,blendR)


    def Circle(self,joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t,ovl,offset_flag,offset_pos):
        """Circular motion in Cartesian space
            joint_pos_p:Path point joint position,unit[°];

            desc_pos_p:Path point Cartesian pose,unit[mm][°];

            ptool:Tool number,[0~14];

            puser:Workpiece number,[0~14];

            pvel:Speed percentage,[0~100];

            pacc:Acceleration percentage,[0~100],temporarily closed;

            exaxis_pos_p:Position of external axis 1~position of external axis 4;

            joint_pos_t:Target point joint position,unit[°];

            desc_pos_t:Cartesian pose of the target point,unit[mm][°];

            ttool:Tool number,[0~14];

            tuser:Workpiece number,[0~14];

            tvel:Speed percentage,[0~100];

            tacc:Acceleration percentage,[0~100],temporarily closed;

            exaxis_pos_t:Position of external axis 1~position of external axis 4;

            ovl:Speed scaling factor,[0~100%];

            offset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;

            offset_pos:Offset,unit[mm][°]

        """

                # import frrpc
                # # A connection is established with the robot controller. A successful connection returns a robot object
                # robot = frrpc.RPC('192.168.58.2')
                # J1=[121.381,-97.108,-123.768,-45.824,89.877,-47.296]
                # P1=[-127.772,459.534,221.274,-177.850,-2.507,78.627]
                # eP1=[0.000,0.000,0.000,0.000]
                # dP1=[10.000,10.000,10.000,10.000,10.000,10.000]
                # J2=[138.884,-114.522,-103.933,-49.694,90.688,-47.291]
                # P2=[-360.468,485.600,196.363,-178.239,-0.893,96.172]
                # eP2=[0.000,0.000,0.000,0.000]
                # dP2=[10.000,10.000,10.000,10.000,10.000,10.000]
                # pa2=[0.0,0.0,100.0,180.0]
                # J3=[159.164,-96.105,-128.653,-41.170,90.704,-47.290]
                # P3=[-360.303,274.911,203.968,-176.720,-2.514,116.407]
                # eP3=[0.000,0.000,0.000,0.000]
                # dP3=[10.000,10.000,10.000,10.000,10.000,10.000]
                # pa3=[0.0,0.0,100.0,180.0]
                # dP=[10.000,10.000,10.000,10.000,10.000,10.000]
                # robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)    #Joint space motionPTP
                # robot.Circle(J2,P2,pa2,eP2,J3,P3,pa3,eP3,100.0,0,dP)    #Circular motion in Cartesian space
        return self.robot.Circle(joint_pos_p,desc_pos_p,ptool,puser,pvel,pacc,exaxis_pos_p,joint_pos_t,desc_pos_t,ttool,tuser,tvel,tacc,exaxis_pos_t,ovl,offset_flag,offset_pos)

    #Spiral motion in Cartesian space
    def NewSpiral(self,joint_pos,desc_pos,tool,user,vel,acc,exaxis_pos,ovl,offset_flag,offset_pos,param):
        """ Spiral motion in Cartesian space

        joint_pos:Target joint position, unit[°];

        desc_pos:Target Cartesian pose,unit[mm][°];

        tool:Tool number,[0~14];

        user:Workpiece number,[0~14];

        vel:Speed percentage,[0~100];

        acc:Acceleration percentage,[0~100],temporarily closed;

        exaxis_pos:Position of external axis 1~position of external axis 4;

        ovl:Speed scaling factor,[0~100];

        offset_flag:[0]-no offset, [1]-offset under workpiece/base coordinate system, [2]-offset under tool coordinate system;

        offset_pos:Pose offset,unit[mm][°]

        param:[circle_num,circle_angle,rad_init,rad_add,rotaxis_add,rot_direction],
                circle_num: number of coils, 
                circle_angle: helix angle, 
                rad_init: initial radius of the helix, 
                rad_add: radius increment, 
                rotaxis_add: axis direction increment, 
                rot_direction: rotation direction, 
                0-clockwise, 1-counterclockwise
        """
            # import frrpc
            # # A connection is established with the robot controller. A successful connection returns a robot object
            # robot = frrpc.RPC('192.168.58.2')
            # J1=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
            # eP1=[0.000,0.000,0.000,0.000]
            # dP1=[50.0,0.0,0.0,-30.0,0.0,0.0]
            # J2=[127.888,-101.535,-94.860,17.836,96.931,-61.325]
            # eP2=[0.000,0.000,0.000,0.000]
            # dP2=[50.0,0.0,0.0,-5.0,0.0,0.0]
            # Pa = [5.0,5.0,50.0,10.0,10.0,0.0]
            # P1 = robot.GetForwardKin(J1)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
            # print(P1)
            # P1 = [P1[1],P1[2],P1[3],P1[4],P1[5],P1[6]]
            # robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,0.0,2,dP1)
            # P2 = robot.GetForwardKin(J2)       #The forward kinematic interface can be used to solve Cartesian space coordinates with only joint positions
            # print(P2)
            # P2 = [P2[1],P2[2],P2[3],P2[4],P2[5],P2[6]]
            # robot.NewSpiral(J2,P2,0,0,100.0,180.0,eP2,100.0,2,dP2,Pa)   #Helical motion
        return self.robot.NewSpiral(joint_pos,desc_pos,tool,user,vel,acc,exaxis_pos,ovl,offset_flag,offset_pos,param)
    
    #Joint space servo mode motion
    def ServoJ(self,joint_pos,acc,vel,cmdT,filterT,gain):

        """
            Joint space servo mode motion
            
            joint_pos:Target joint position, unit[°];

            acc:Acceleration, range[0~100],temporarily closed,default to 0;

            vel: Speed, range[0~100],temporarily closed,default to 0;

            cmdT:Instruction Cycle,unit[s],[0.001~0.016];

            filterT:Filtering time,unit[s],temporarily closed;

            gain:Proportional amplifier for target position,temporarily closed
        
        """
            # import frrpc
            # import time
            # # A connection is established with the robot controller. A successful connection returns a robot object
            # robot = frrpc.RPC('192.168.58.2')
            # joint_pos = robot.GetActualJointPosDegree(0)
            # print(joint_pos)
            # joint_pos = [joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5],joint_pos[6]]
            # acc = 0.0
            # vel = 0.0
            # t = 0.008
            # lookahead_time = 0.0
            # P = 0.0
            # count = 100
            # while(count):
            #     robot.ServoJ(joint_pos, acc, vel, t, lookahead_time, P)
            #     joint_pos[0] = joint_pos[0] + 0.1
            #     count = count - 1
            #     time.sleep(0.008)

        return self.robot.ServoJ(joint_pos,acc,vel,cmdT,filterT,gain)
    
    #Point-to-point motion in Cartesian space
    def MoveCart(self,desc_pos,tool,user,vel,acc,ovl,blendT,config):
        """
            Point-to-point motion in Cartesian space
            desc_pos:Target Cartesian position;

            tool:Tool number,[0~14];

            user:Workpiece number,[0~14];

            vel: Speed, range[0~100],temporarily closed,default to 0;

            acc:Acceleration, range[0~100],temporarily closed,default to 0;

            ovl:Speed scaling factor,[0~100];

            blendT:[-1.0]-Motion in place (blocked), [0-500]-Smoothing time (non blocked),unit[ms];

            config:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration
        
        """
            # import frrpc
            # import time
            # # A connection is established with the robot controller. A successful connection returns a robot object
            # robot = frrpc.RPC('192.168.58.2')
            # P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
            # P2=[-273.856,643.260,259.235,-177.972,-1.494,80.866]
            # P3=[-423.044,229.703,241.080,-173.990,-5.772,123.971]
            # robot.MoveCart(P1,0,0,100.0,100.0,100.0,-1.0,-1)       #Point-to-point motion in Cartesian space
            # robot.MoveCart(P2,0,0,100.0,100.0,100.0,-1.0,-1)
            # robot.MoveCart(P3,0,0,100.0,100.0,100.0,0.0,-1)
            # time.sleep(1)
            # robot.StopMotion()    #Stop moving
        return self.robot.MoveCart(desc_pos,tool,user,vel,acc,ovl,blendT,config)
    
    #Robot spline motion
    def SplineStart(self):
        return self.robot.SplineStart()
    
    #Spline motion PTP
    def SplinePTP(self,joint_pos,desc_pos,tool,user,vel,acc,ovl):
     
        """ Spline motion PTP
            joint_pos:Target joint position, unit[°];

            desc_pos:Target Cartesian pose,unit[mm][°];

            tool:Tool number,[0~14];

            user:Workpiece number,[0~14];

            vel: Speed, range[0~100],temporarily closed,default to 0;

            acc:Acceleration, range[0~100],temporarily closed,default to 0;

            ovl:Speed scaling factor,[0~100];

        """

        return self.robot.SplinePTP(joint_pos,desc_pos,tool,user,vel,acc,ovl)

    # Spline motion end
    def SplineEnd(self):
        return self.robot.SplineEnd()

    #region Spline Example

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
        # P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
        # eP1 = [0.000,0.000,0.000,0.000]
        # dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
        # J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
        # P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
        # J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
        # P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
        # J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
        # P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
        # robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
        # robot.SplineStart()    #Spline motion start
        # robot.SplinePTP(J1,P1,0,0,100.0,180.0,100.0)    #Spline motion PTP
        # robot.SplinePTP(J2,P2,0,0,100.0,180.0,100.0)
        # robot.SplinePTP(J3,P3,0,0,100.0,180.0,100.0)
        # robot.SplinePTP(J4,P4,0,0,100.0,180.0,100.0)
        # robot.SplineEnd()     #Spline motion end
    
    #endregion

    #Robot New Spline Motion
    #New spline motion start

    def NewSplineStart(self,type):
        """type:0-arc transition, 1-given point position path point"""
        return self.robot.NewSplineStart(type)

    #New spline motion end
    def NewSplineEnd(self):
        return self.robot.NewSplineEnd()
    
    # New Spline Instruction Points
    def NewSpiralPoint(self,joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,lastFlag):
        """
            New Spline Instruction Points
            joint_pos:Target joint position, unit[°];

            desc_pos:Target Cartesian pose,unit[mm][°];

            tool:Tool number,[0~14];

            user:Workpiece number,[0~14];

            vel: Speed, range[0~100],temporarily closed,default to 0;

            acc:Acceleration, range[0~100],temporarily closed,default to 0;

            ovl:Speed scaling factor,[0~100];

            blendR: [0-1000]-smooth radius,unit[mm];

            lastFlag:Is it the last point, 0-No, 1-Yes
        """
        return self.robot.NewSpiralPoint(joint_pos,desc_pos,tool,user,vel,acc,ovl,blendR,lastFlag)
    
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # J1 = [114.578,-117.798,-97.745,-54.436,90.053,-45.216]
        # P1 = [-140.418,619.351,198.369,-179.948,0.023,69.793]
        # eP1 = [0.000,0.000,0.000,0.000]
        # dP1 = [0.000,0.000,0.000,0.000,0.000,0.000]
        # J2 = [115.401,-105.206,-117.959,-49.727,90.054,-45.222]
        # P2 = [-95.586,504.143,186.880,178.001,2.091,70.585]
        # J3 = [135.609,-103.249,-120.211,-49.715,90.058,-45.219]
        # P3 = [-252.429,428.903,188.492,177.804,2.294,90.782]
        # J4 = [154.766,-87.036,-135.672,-49.045,90.739,-45.223]
        # P4 = [-277.255,272.958,205.452,179.289,1.765,109.966]
        # robot.MoveJ(J1,P1,0,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
        # robot.NewSplineStart(1)    #The spline motion begins
        # robot.NewSplinePoint(J1,P1,0,0,50.0,50.0,50.0,0.0,0)    #Spline control point
        # robot.NewSplinePoint(J2,P2,0,0,50.0,50.0,50.0,0.0,0)
        # robot.NewSplinePoint(J3,P3,0,0,50.0,50.0,50.0,0.0,0)
        # robot.NewSplinePoint(J4,P4,0,0,50.0,50.0,50.0,0.0,1)
        # robot.NewSplineEnd()

    #Robot terminates motion
    def StopMotion(self):
        """To terminate motion, use the termination motion instructions as non-blocking state"""
        return self.robot.StopMotion()
    
    #Overall displacement of robot points
    #Starting point overall offset
    def PointsOffsetEnable(self,flag,offset_pos):
        """
        flag:0-offset under base coordinate or workpiece coordinate system, 2-offset under tool coordinate system;

        offset_pos:Offset,unit[mm][°]
        """
        return self.robot.PointsOffsetEnable(flag,offset_pos)
    
    #The overall offset of the point ends
    def PointsOffsetDisable(self):
        """The overall offset of the point ends"""
        return self.robot.PointsOffsetDisable()

    #region Example New Line Motion
    # import frrpc
    # import time
    # # A connection is established with the robot controller. A successful connection returns a robot object
    # robot = frrpc.RPC('192.168.58.2')
    # #Overall shift of robot point position
    # J1=[-168.847,-93.977,-93.118,-80.262,88.985,11.831]
    # P1=[-558.082,27.343,208.135,-177.205,-0.450,89.288]
    # eP1=[0.000,0.000,0.000,0.000]
    # dP1=[10.000,10.000,10.000,0.000,0.000,0.000]
    # J2=[168.968,-93.977,-93.118,-80.262,88.986,11.831]
    # P2=[-506.436,236.053,208.133,-177.206,-0.450,67.102]
    # eP2=[0.000,0.000,0.000,0.000]
    # dP2=[0.000,0.000,0.000,0.000,0.000,0.000]
    # robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    # robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    # time.sleep(2)
    # flag = 0
    # offset = [100.0,5.0,6.0,0.0,0.0,0.0]   #Pose offset
    # robot.PointsOffsetEnable(flag, offset)   #Global offset start
    # robot.MoveJ(J1,P1,1,0,100.0,180.0,100.0,eP1,-1.0,0,dP1)
    # robot.MoveJ(J2,P2,1,0,100.0,180.0,100.0,eP2,-1.0,0,dP2)
    # robot.PointsOffsetDisable()  #End of global shift
    #endregion
    

#endregion

#region IO



    #Set the digital output of the control box
    def SetDigitalOut(self,id,status,smooth,block):
        """
        id:IO number,range[0~15];

        status:0-off, 1-on;

        smooth:0-unsmooth, 1-smooth;

        block:0-blocking, 1-non blocking.
        """
        import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # for i in range(0,16):
        #     robot.SetDO(i,1,0,0)   #Open the control box DO
        # robot.WaitMs(1000)
        # for i in range(0,16):
        #     robot.SetDO(i,0,0,0)   #Close the control box DO
        # robot.WaitMs(1000)

        return self.robot.SetDO(id,status,smooth,block)
    
    #Set tool digital output
    def SetToolDO(self,id,status,smooth,block):
        """Set tool digital output
        id:IO number,range[0~15];

        status:0-off, 1-on;

        smooth:0-unsmooth, 1-smooth;

        block:0-blocking, 1-non blocking.
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # for i in range(0,2):
        #     robot.SetToolDO(i,1,0,0)    #Open the control box DO
        # robot.WaitMs(1000)
        # robot.WaitMs(1000)
        # for i in range(0,2):
        #     robot.SetToolDO(i,0,0,0)    #Close the control box DO
        return self.robot.SetToolDO(id,status,smooth,block)
    
    #Set the analog output of the control box
    def SetAO(self,id,value,block):
        """
        Set the analog output of the control box
        id:IO number,range[0~1];

        value:electricity or voltage value percentage, range [0-100%] corresponds to electricity value [0-20mA] or voltage [0-10V];

        block:[0]-blocking, [1]-non blocking
        """
        return self.robot.SetAO(id,value,block)
    
    #Set tool analog output
    def SetToolAO(self,id,value,block):

        """
        Set tool analog output
        id:IO number,range[0];

        value:electricity or voltage value percentage, range [0-100%] corresponds to electricity value [0-20mA] or voltage [0-10V];

        block:[0]-blocking, [1]-non blocking
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetToolAO(0,100.0,0)   # Set tool analog output
        # robot.WaitMs(1000)
        # robot.SetToolAO(0,0.0,0)
        return self.robot.SetToolAO(id,value,block)
    
    #Obtain the digital input of the control box
    def GetDI(self,id,block):
        """
            Obtain the digital input of the control box
            id:IO number,range[0~15];

            block:[0]-blocking, [1]-non blocking
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # di = robot.GetDI(0,0)   # Obtain the digital input of the control box
        # print(di)
        return self.robot.GetDI(id,block)
    
    #Obtain tool digital input
    def GetToolDI(self,id,block):
        """
            Obtain tool digital input
            id:IO number,range[0~1];

            block:[0]-blocking, [1]-non blocking
            Success:[0,di],di: 0-Low level,1-High level

            Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # tool_di = robot.GetToolDI(1,0)   # Obtain tool digital input
        # print(tool_di)
        return self.robot.GetToolDI(id,block)

    #Waiting for digital input from the control box
    def WaitDI(self,id,status,maxtime,opt):
        """
            Waiting for digital input from the control box
            id:IO number,range[0~15];

            status:0-off,1-on;

            maxtime:Maximum waiting time, unit[ms];

            opt:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting
            Success:[0]

            Failed:[errcode]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitDI(0,1,0,2)    # Waiting for the control box digital input
        return self.robot.WaitDI(id,status,maxtime,opt)
    
    #Waiting for multiple digital inputs from the control box
    def WaitMultiDI(self,mode,id,status,maxtime,opt):
        """
        Waiting for multiple digital inputs from the control box
        mode:[0]-Multiplex AND, [1]-Multiplex OR;

        id:IO number, bit0~bit7 corresponds to DI0~DI7, bit8~bit15 corresponds to CI0~CI7;

        status(uint16_t):bit0~bit7 corresponds to DI0~DI7 status, bit8~bit15 corresponds to the states of the CI0~CI7 status bits [0]-off, [1]-on;

        maxtime:Maximum waiting time, unit[ms];

        opt:After timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting。
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitMultiDI(1,3,3,10000,2)   #  Waiting for control box multiplex digital input

        return self.robot.WaitMultiDI(mode,id,status,maxtime,opt)
    
    #waiting for tool digital input
    def WaitToolDI(self,id,status,maxtime,opt):
        """
            Waiting for the end digital input
            id:IO number,range[0~1];

            status:0-off,1-on;

            maxtime:Maximum waiting time, unit[ms];

            opt:after timeout strategy, 0-program stops and prompts for timeout, 1-ignore timeout prompt to continue executing the program, 2-keep waiting
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitToolDI(1,1,0,2)    #  Wait for the tool number to enter
        return self.robot.WaitToolDI(id,status,maxtime,opt)
    
    #Waiting for terminal digital input
    def GetAI(self,id,block):
        """
            Returns 
            Success:[0,value], 
            value:Input current or voltage value percentage, 
            range[0-100] corresponds to current value[0-20mA] or voltage[0-10V];

            Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ai = robot.GetAI(0,1)   #  Obtain control box analog input
        # print(ai)

        return self.robot.GetAI(id,block)
    #Obtain tool analog input
    def GetToolAI(self,id,block):
        """
            Returns
            Success:[0,value], 
            value:Input current or voltage value percentage, 
            range[0-100] corresponds to current value[0-20mA] or voltage[0-10V];

            Failed:[errcode,]

            Obtain terminal analog input
            id:IO number,range[0];

            block:[0]-blocking, [1]-non blocking
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # tool_ai = robot.GetToolAI(0,1)    #   Obtain tool analog input
        # print(tool_ai)
        return self.robot.GetToolAI(id,block)
    
    #Waiting for the control box simulation input
    def WaitAI(self,id,sign,value,maxtime,opt):
        """Waiting for the control box simulation inputid:IO number,range[0~1];

            sign:0-Greater than,1-Less than

            value:Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V];

            maxtime:Maximum waiting time, unit[ms];

            opt:After timeout strategy,
              0-program stops and prompts for timeout,
                1-ignore timeout prompt to continue executing the program, 
                2-keep waiting
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitAI(0,0,50,0,2)   # Always waiting for tool analog input        
        return self.robot.WaitAI(id,sign,value,maxtime,opt)
    
    #Waiting for tool analog input
    def WaitToolAI(self,id,sign,value,maxtime,opt):
        """Waiting for the end analog input
        id:IO number,range[0];

        sign:0-Greater than,1-Less than

        value: Input current or voltage value percentage, range[0-100] corresponds to current value[0-20mA] or voltage[0-10V];

        maxtime:Maximum waiting time, unit[ms];

        opt:After timeout strategy, 
        0-program stops and prompts for timeout, 
        1-ignore timeout prompt to continue executing the program, 
        2-keep waiting
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitToolAI(0,0,50,0,2)   #  Always waiting for tool analog input
        return self.robot.WaitToolAI(id,sign,value,maxtime,opt)

#endregion

#region Common Settings Functions

    #Set Gloabl Speed
    def SetSpeed(self,vel):
        """
            Set global speed

            vel:Speed percentage, range[0~100]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetSpeed(20)   # Set the global speed. Manual mode and automatic mode are set independently
        return self.robot.SetSpeed(vel)
    
    # Setting System Variable Value

    def SetSysVarValue(self,id, value):
        """Setting System Variable Values
        id:Variable number, range[1~20];

        value:Variable value
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # for i in range(1,21):
        #     robot.SetSysVarValue(i,i+0.5)    #  Setting System Variable Values
        # robot.WaitMs(1000)
        # for i in range(1,21):
        #     sys_var = robot.GetSysVarValue(i)  #  Example Query the values of system variables
        #     print(sys_var)        
        return self.robot.SetSysVarValue(id,value)
    
    #Set Tool Coordinate System

    def SetToolCoord(self,id,t_coord,type,install):

        """Set Tool Coordinate System
        id:Coordinate system number, range[0~14];

        t_coord:Position of tool center point relative to end flange center, unit[mm][°];

        type:0-Tool coordinate system,1-Sensor coordinate system;

        install:Installation position,0-Robot end,1-Robot external
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
        # robot.SetToolCoord(10,t_coord,0,0)  #  Set tool coordinate system

        return self.robot.SetToolCoord(id,t_coord,type,install)
    
    #Set Tool Coordinate Series Table

    def SetToolList(self,id,t_coord,type,install):
        """
        Set Tool Coordinate Series Table
        id:Coordinate system number, range[0~14];

        t_coord:Position of tool center point relative to end flange center, unit[mm][°];

        type:0-Tool coordinate system,1-Sensor coordinate system;

        install:Installation position,0-Robot end,1-Robot external
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # t_coord = [1.0,2.0,3.0,4.0,5.0,6.0]
        # robot.SetToolList(10,t_coord,0,0)  #  Set tool coordinate system
        return self.robot.SetToolList(id,t_coord,type,install)
    
    #Set the external tool coordinate system

    def SetExToolCoord(self,id,etcp ,etool):
        """Set the external tool coordinate system
        id:Coordinate system number, range[0~14];

        etcp:External tool coordinate system, unit[mm][°];

        etool:End tool coordinate system, unit[mm][°];
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
        # etool = [21.0,22.0,23.0,24.0,25.0,26.0]
        # robot.SetExToolCoord(10,etcp,etool)
        return self.robot.SetExToolCoord(id,etcp ,etool)
    

    #Set external tool coordinate series table

    def SetExToolList(self,id,etcp ,etool):
        """
        Set external tool coordinate series table
        id:Coordinate system number, range[0~14];

        etcp:External tool coordinate system, unit[mm][°];

        etool:End tool coordinate system, unit[mm][°];
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # etcp = [1.0,2.0,3.0,4.0,5.0,6.0]
        # etool = [21.0,22.0,23.0,24.0,25.0,26.0]
        # robot.SetExToolList(10,etcp,etool)
        return self.robot.SetExToolList(id,etcp ,etool)
    
    #Set the workpiece coordinate system
    def SetWObjCoord(self,id,w_coord):
        """Set the workpiece coordinate system
        id:Coordinate system number, range[0~14];

        w_coord:Relative pose of coordinate system, unit[mm][°];
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
        # robot.SetWObjCoord(11,w_coord)
        return self.robot.SetWObjCoord(id,w_coord)

    #Set the workpiece coordinate series table
    def SetWObjList(self,id,w_coord):
        """
            Set the workpiece coordinate series table
            id:Coordinate system number, range[0~14];

            w_coord:Relative pose of coordinate system, unit[mm][°];
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # w_coord = [11.0,12.0,13.0,14.0,15.0,16.0]
        # robot.SetWObjList(11,w_coord)
        return self.robot.SetWObjList(id,w_coord)
    
    # Set end load weight

    def SetLoadWeight(self,weight):
        """Set end load weight
        weight:unit[kg]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetLoadWeight(3.0)   # Set load weight
        return self.robot.SetLoadWeight(weight)

    #Set the robot installation method - fixed installation
    def SetRobotInstallPos(self,method):
        """Set the robot installation method - fixed installation
            method:0-Flat installation, 
            1-Side installation, 
            2-Hanging installation
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetRobotInstallPos(0)    #   Set the robot installation mode
        return self.robot.SetRobotInstallPos(method)
    
    #Set robot installation angle - free installation

    def SetRobotInstallAngle(self,yangle,zangle):
        """Set robot installation angle - free installation
        yangle:Angle of roll

        zangle:Rotation angle
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetRobotInstallAngle(0.0,0.0)    #   Set the robot installation Angle
        return self.robot.SetRobotInstallAngle(yangle,zangle)
    
    # Set the centroid coordinates of the end load

    def SetLoadCoord(self,x,y,z):

        """
            Set the centroid coordinates of the end load
            x, y, z: Barycentric coordinate,unit[mm]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetLoadCoord(3.0,4.0,5.0)    #   Set the load centroid coordinates
        return self.robot.SetLoadCoord(x,y,z)
    
    # Waiting for specified time
    def WaitMs(self,t_ms):
        """
            waiting for specified time
            t_ms:unit[ms]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.WaitMs(1000)    #  Wait 1000ms
        return self.robot.WaitMs(t_ms)
    
#endregion

#region Security settings

    #Set collision level
    def SetAnticollision (self,mode,level,config):

        """
        Set collision level
        mode:0-level, 1-percentage;;

        level=[j1,j2,j3,j4,j5,j6]:collision threshold;

        config:0-do not update configuration file, 1-update configuration file
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # level = [1.0,2.0,3.0,4.0,5.0,6.0]
        # robot.SetAnticollision(0,level,1)     #  Set collision level
        # level = [50.0,20.0,30.0,40.0,50.0,60.0]
        # robot.SetAnticollision(1,level,1)     #  Set collision percentage
        return self.robot.SetAnticollision(mode,level,config)

    #Set the strategy after collision

    def SetCollisionStrategy(self,strategy):
        """
        Set the strategy after collision
        strategy:0-Error Pause, 1-Continue Running
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.SetCollisionStrategy(1)    # Set post collision strategy,1-Continue Running
        return self.robot.SetCollisionStrategy(strategy)
    
    #Set positive limit
    def SetLimitPositive(self,p_limit):
        """
            Set positive limit
            p_limit=[j1,j2,j3,j4,j5,j6]:six joint positions
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
        # robot.SetLimitPositive(p_limit)   #  Set positive limit
        return self.robot.SetLimitPositive(p_limit)

    #Set negative limit

    def SetLimitNegative(self,n_limit):
        """
        Set negative limit

        n_limit=[j1,j2,j3,j4,j5,j6]:six joint positions
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]
        # robot.SetLimitNegative(n_limit)   # Set negative limit
        return self.robot.SetLimitNegative(n_limit)
    
    #Error status cleared

    def ResetAllError(self):
        """
            Error status cleared,only resettable errors can be cleared
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.ResetAllError()    #  Error status cleared        
        return self.robot.ResetAllError()

    #Joint friction compensation switch
    def FrictionCompensationOnOff(self,state):
        """Joint friction compensation switch
            state:0-off,1-on
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
        return self.robot.FrictionCompensationOnOff(state)
    
    #Set joint friction compensation coefficient formal installation

    def SetFrictionValue_level(self,coeff):
        """Set joint friction compensation coefficient - formal installation
            coeff=[j1,j2,j3,j4,j5,j6]:six joint compensation coefficients
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
        # lcoeff = [0.9,0.9,0.9,0.9,0.9,0.9]
        # robot.SetFrictionValue_level(lcoeff)   #  Set joint friction compensation coefficient

        return self.robot.SetFrictionValue_level(coeff)
    
    #Set joint friction compensation coefficient-Inverted

    def SetFrictionValue_ceiling(self,coeff):
        """Set joint friction compensation coefficient-Inverted
            coeff=[j1,j2,j3,j4,j5,j6]:six joint compensation coefficients
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
        # ccoeff = [0.6,0.6,0.6,0.6,0.6,0.6]
        # robot.SetFrictionValue_ceiling(ccoeff)  #  Set joint friction compensation coefficient
        return self.robot.SetFrictionValue_ceiling(coeff)

    #Set joint friction compensation coefficient-free installation

    def SetFrictionValue_freedom(self,coeff):
        """Set joint friction compensation coefficient-free installation
            coeff=[j1,j2,j3,j4,j5,j6]:six joint compensation coefficients
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # robot.FrictionCompensationOnOff(1)   #  Joint friction compensation open
        # fcoeff = [0.5,0.5,0.5,0.5,0.5,0.5]
        # robot.SetFrictionValue_freedom(fcoeff)   #  Set joint friction compensation coefficient

        return self.robot.SetFrictionValue_freedom(coeff) 
#endregion

#region

    #Obtain robot installation angle
    def GetRobotInstallAngle(self):
        """
        Returns 
        Success:[0,yangle,zangle],yangle-angle of roll,zangle-rotation angle

        Failed:[errcode,]
        
        """
    # import frrpc
    # # A connection is established with the robot controller. A successful connection returns a robot object
    # robot = frrpc.RPC('192.168.58.2')
    # ret = robot.GetRobotInstallAngle()  # Obtain robot installation angle
    # print(ret)
        return self.robot.GetRobotInstallAngle()
    
    #Obtain system variable values
    def GetSysVarValue(self,id):
        """ Return
            Obtain system variable values
            id:System variable number, range[1~20]
        
        Success:[0,var_value]
        Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # for i in range(1,21):
        #     robot.SetSysVarValue(i,i+0.5)    #  Setting System Variable Values
        # robot.WaitMs(1000)
        # for i in range(1,21):
        #     sys_var = robot.GetSysVarValue(i)  #  Query system variable values
        #     print(sys_var)

        self.robot.GetSysVarValue(id)

    #Obtain the current joint position (angle)

    def GetActualJointPosDegree(self,flag):
        """Description: Obtain the current joint position (angle))

        Parameter: flag:0-blocking, 1-non blocking

        Return value: Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]

        Failed:[errcode,]"""

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetActualJointPosDegree(0)  # Obtain the current joint position of the robot
        # print(ret)

        return self.robot.GetActualJointPosDegree(flag)
    
    #Obtain the current joint position(radian)
    def GetActualJointPosRadian(self,flag):
        """Description: Obtain the current joint position(radian)

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]

                            Failed:[errcode,]"""
        
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetActualJointPosRadian(0)  # Obtain the current joint position of the robot
        # print(ret)

        return self.robot.GetActualJointPosRadian(flag)
    
    #Obtain the current tool pose

    def GetActualTCPPose(self,flag):
        """Description: Obtain the current tool pose

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,tcp_pose],tcp_pose=[x,y,z,rx,ry,rz]
                            Failed:[errcode,]
        
            Example:
                # import frrpc
                # # A connection is established with the robot controller. A successful connection returns a robot object
                # robot = frrpc.RPC('192.168.58.2')
                # ret = robot.GetActualTCPPose(0)  # Obtain the current tool pose of the robot
                # print(ret)

        """
        
        return self.robot.GetActualTCPPose(flag)
    
    #Obtain the current tool coordinate system number

    def GetActualTCPNum(self,flag):
        """ Description: Obtain the current tool coordinate system number

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,tool_id]

                            Failed:[errcode,]
        
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetActualTCPNum(0)  # Obtain the current tool coordinate system number
        # print(ret)     

        return self.robot.GetActualTCPNum(flag)
    #Obtain the current workpiece coordinate system number
    def GetActualWObjNum(self,flag):
        """ Description: Obtain the current workpiece coordinate system number

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,wobj_id]

                            Failed:[errcode,]
        
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetActualWObjNum(0)  # Obtain the current workpiece coordinate system number
        # print(ret)
        return self.robot.GetActualWObjNum(flag)
    
    
    #Obtain the current end flange pose
    
    def GetActualToolFlangePose(self, flag):

        """
            Description: Obtain the current end flange pose

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,flange_pose],flange_pose=[x,y,z,rx,ry,rz]

            Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetActualToolFlangePose(0)  # Obtain the current end flange pose
        # print(ret)     
        return self.robot.GetActualToolFlange(flag)
    
    #Inverse kinematics solution
    def GetInverseKin(self,type,desc_pos,config):
        """
            Description

            Inverse kinematics, Cartesian pose to solve joint position

            Parameter

            type:0-absolute pose (base coordinate system), 1-relative pose (base coordinate system), 2-relative pose (tool coordinate system)

            desc_pose:[x,y,z,rx,ry,rz],tool posture,unit[mm][°]

            config:Joint configuration, [-1]-refer to the current joint position for solution, [0-7]-solve based on joint configuration

            Return value

            Success:[0,joint_pos],joint_pos=[j1,j2,j3,j4,j5,j6]

            Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # P1=[75.414,568.526,338.135,-178.348,-0.930,52.611]
        # ret = robot.GetInverseKin(0,P1,-1)
        # print(ret)       

        return self.robot.GetInverseKin(type,desc_pos,config)
    
    #Obtain the weight of the current load
    def GetTargetPayload(self,flag):
        """
            Description: Obtain the weight of the current load

            Parameter: flag:0-blocking, 1-non blocking

            Return value

            Success:[0,weight],unit[kg]

            Failed:[errcode,]
        """
    
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetTargetPayload(0)  # Obtain the weight of the current load
        # print(ret)

        return self.robot.GetTargetPayload(flag)
    
    # Obtain the current tool coordinate system
    def GetTCPOffset(self,flag):
        """
            Description: Obtain the current tool coordinate system

            Parameter:  flag:0-blocking, 1-non blocking

            Return value: Success:[0,tcp_offset], tcp_offset=[x,y,z,rx,ry,rz]:relative pose,unit[mm][°]

                            Failed:[errcode,]
        """

        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetTCPOffset(0)  # Obtain the current tool coordinate system
        # print(ret)

        return self.robot.GetTCPOffset(flag)
    
    #Obtain the current workpiece coordinate system
    def GetWObjOffset(self,flag):
        """
            Description : Obtain the current workpiece coordinate system

            Parameter: flag:0-blocking, 1-non blocking

            Return value: Success:[0,wobj_offset], wobj _offset=[x,y,z,rx,ry,rz]:relative pose,unit[mm][°]

            Failed:[errcode,]
        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetWObjOffset(0)  # Obtain the current workpiece coordinate system
        # print(ret)   

        return self.robot.GetWObjOffset(flag)
    
    #Check if the robot motion is complete
    def GetRobotMotionDone(self):
        """Description: Check if the robot motion is complete

            Parameter: Nothing

            Return value: Success:[0,state],state:0-incomplete,1-complete

            Failed:[errcode,]

        """
        # import frrpc
        # # A connection is established with the robot controller. A successful connection returns a robot object
        # robot = frrpc.RPC('192.168.58.2')
        # ret = robot.GetRobotMotionDone()    #Query the motion completion status of the robot
        # if ret[0] == 0:
        #     print(ret[1])
        # else:
        #     print("the errcode is: ", ret[0])

        return self.robot.GetRobotMotionDone()
#endregion