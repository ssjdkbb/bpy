bl_info = {
    "name": "Robot_ZDFX0808",
    "author": "zzk",
    "version": (0, 0, 1),
    "blender": (2, 80, 0),
    "location": "",
    "description": "",
    "category": "传习机械臂",
}

import sys
import bpy
import math
import numpy as np
import socket
import json
import time
import threading

def R_x(theta,pos:tuple):
    theta = theta/180*math.pi
    matrix =[[1,0              ,0               ,pos[0]],
             [0,math.cos(theta),-math.sin(theta),pos[1]],
             [0,math.sin(theta),math.cos(theta) ,pos[2]],
             [0,0,              0               ,1]]
    return np.mat(matrix)

def R_y(theta,pos:tuple):
    theta = theta/180*math.pi
    matrix =[[math.cos(theta),0,math.sin(theta),pos[0]],
            [0               ,1,0              ,pos[1]],
            [-math.sin(theta),0,math.cos(theta),pos[2]],
            [0               ,0,0              ,1]]
    return np.mat(matrix)

def R_z(theta,pos:tuple):
    theta = theta/180*math.pi
    matrix =[[math.cos(theta),-math.sin(theta),0,pos[0]],
             [math.sin(theta), math.cos(theta),0,pos[1]],
             [0              , 0              ,1,pos[2]],
             [0              , 0              ,0,1]]
    return np.mat(matrix)

def rad2angle(rad:tuple):
    angle = []
    def map_to_angle_range(angle):
        while angle <= -180:
            angle += 360
        while angle > 180:
            angle -= 360
        return angle
    for r in rad:
        angle.append(map_to_angle_range(r/math.pi*180))
    return angle

def angle2rad(angle:tuple):
    rad = tuple()
    for a in angle:
        rad+=(a/180*math.pi,)
    return rad


class BPY_EX():
    def __init__(self):
        self.obj_name = ["00"+str(i) for i in range(1,7)]
        self.eluer={
            '001':2,
            '002':1,
            '003':1,
            '004':0,
            '005':1,
            '006':0,
        }
    def choose_object(self,object):
        bpy.ops.object.select_all(action='DESELECT')
        bpy.context.view_layer.objects.active=None
        bpy.data.objects[object].select_set(True)
        bpy.context.view_layer.objects.active=bpy.data.objects[object]
    def get_world_matrix(self,obj_name:str):
        obj = bpy.data.objects[obj_name]
        return obj.matrix_world

    def get_world_pos(self,obj_name:str):
        obj = bpy.data.objects[obj_name]
        x = obj.matrix_world.translation.x
        y = obj.matrix_world.translation.y
        z = obj.matrix_world.translation.z
        print(f"world_p{x,y,z}")
        return x, y, z
    
    def get_world_oritention(self,obj_name:str):
        matrix=self.get_world_matrix(obj_name)
        print(f"world_matrix:{matrix}")

    def get_euler(self,*args):
        euler = []
        for name in args:
            if name in self.obj_name:
                obj = bpy.data.objects[name]
                euler.append(obj.rotation_euler)
        t1,t2,t3,t4,t5,t6 = euler[0].z,euler[1].y,euler[2].y,euler[3].x,euler[4].y,euler[5].x
        return t1,t2,t3,t4,t5,t6
    
    def set_euler(self,obj:tuple,rad:tuple):
        for idx,name in enumerate(obj):
            for na,dir in self.eluer.items():
                if name in self.obj_name:
                    if name==na:
                        bpy.data.objects[na].rotation_euler[dir]=rad[idx]
                        # if name=="004" and rad[idx]==0:
                        #     rad[idx] = 0.01
                        #     bpy.data.objects[na].rotation_euler[dir]=rad[idx]
        # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    def move2safe(self,isupdate):
        """回到安全点"""
        return self.robot_move(((0,)*4+(np.pi/2,)+(0,)),5000,isupdate,check_hit=False)
        # time.sleep(1)  

    def check_jn(self,goal_jn):
        """检查弧度"""
        for jn in goal_jn:
            for na,dir in self.eluer.items():
                if bpy.data.objects[na].rotation_euler[dir]!=jn:
                    return True
        return 
    def check_hit(self,obj_name,hit_name:tuple):
        """碰撞检测"""
        self.choose_object(obj_name)
        for name in hit_name:
            bpy.ops.object.modifier_add(type='BOOLEAN')      
            bpy.data.objects[obj_name].modifiers["布尔"].operation = 'INTERSECT'
            bpy.data.objects[obj_name].modifiers["布尔"].solver = 'FAST'
            bpy.data.objects[obj_name].modifiers["布尔"].double_threshold = 0
            bpy.data.objects[obj_name].modifiers["布尔"].object = bpy.data.objects[name]      
            # bpy.ops.object.modifier_apply(modifier="布尔")    # 应用修改
            bpy.ops.object.modifier_set_active(modifier="布尔") # 激活修改
            vetor = bpy.context.object.dimensions
            print(f"vetor:{vetor}")
            if vetor[0]>0 or vetor[1]>0 or vetor[2]>0:
                bpy.ops.object.modifier_remove(modifier="布尔")
                return True,name
            bpy.ops.object.modifier_remove(modifier="布尔")

    def robot_move(self,jn,t=2000,isupdate=0,check_hit=True):
        """机械臂移动"""
        times = int(t*24/1000)
        if not isupdate:
            self.ttt = time.time()
            self.old_jn = self.get_euler("001","002","003","004","005","006")
            print(f"计算初始角度:{rad2angle(self.old_jn)}")
            print(f"目标角度:{rad2angle(jn)}")
            print(f"迭代次数:{times}")
        deata_jn = [nj-oj for oj,nj in zip(self.old_jn,jn)]
        count = 0
        tol=1e-4
        for s in deata_jn:
            if abs(s)<tol:
                count+=1
            if count>5:
                print("不需要移动")
                return True
        if self.check_jn(jn):
            if (times-isupdate)==1:
                print("最后一次迭代")
                bpy.data.objects['001'].rotation_euler.z=jn[0]
                bpy.data.objects['002'].rotation_euler.y=jn[1]
                bpy.data.objects['003'].rotation_euler.y=jn[2]
                bpy.data.objects['004'].rotation_euler.x=jn[3]
                bpy.data.objects['005'].rotation_euler.y=jn[4]
                bpy.data.objects['006'].rotation_euler.x=jn[5]
                print(self.get_euler("001","002","003","004","005","006"),isupdate)
                print(time.time()-self.ttt)
                return True
            else:
                bpy.data.objects['001'].rotation_euler.z+=deata_jn[0]/times
                bpy.data.objects['002'].rotation_euler.y+=deata_jn[1]/times
                bpy.data.objects['003'].rotation_euler.y+=deata_jn[2]/times
                bpy.data.objects['004'].rotation_euler.x+=deata_jn[3]/times
                bpy.data.objects['005'].rotation_euler.y+=deata_jn[4]/times
                bpy.data.objects['006'].rotation_euler.x+=deata_jn[5]/times
            if check_hit:
                if self.check_hit("005",("000","001","002")) or self.check_hit("004",("000","001","002")):
                    print("发生碰撞")
                    return "hitself"
            bpy.ops.object.select_all(action='DESELECT')
            bpy.context.view_layer.objects.active=bpy.data.objects["006"]
            return False
            # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        return True

class Robot_Kinematics(BPY_EX):
    def __init__(self):
        BPY_EX.__init__(self)  
        self.DH_context_p_xz = [(0,0.206),(0.07,0.126),(0,0.39),(0.091,0.037),(0.256,0)]
        self.L7 = 0.11
        self.a1x,self.a1z,self.a3x,self.a3z,self.d0,self.d2,self.d4,self.d5=0.07,0.126,0.091,0.037,0.206,0.39,0.256,0.11

    def fk(self,jiont:tuple):
        """Forward Kinematics"""
        P5 = np.array([self.L7,0,0,1]).reshape(-1,1)
        P4 = R_y(jiont[4],(self.DH_context_p_xz[4][0],0,self.DH_context_p_xz[4][1]))@P5
        P3 = R_x(jiont[3],(self.DH_context_p_xz[3][0],0,self.DH_context_p_xz[3][1]))@P4
        P2 = R_y(jiont[2],(self.DH_context_p_xz[2][0],0,self.DH_context_p_xz[2][1]))@P3
        P1 = R_y(jiont[1],(self.DH_context_p_xz[1][0],0,self.DH_context_p_xz[1][1]))@P2
        P0 = R_z(jiont[0],(self.DH_context_p_xz[0][0],0,self.DH_context_p_xz[0][1]))@P1
        P = P0.tolist()
        return (P[0][0],P[1][0],P[2][0])

    def get_oritention6(self):
        """带theata 6"""
        t1,t2,t3,t4,t5,t6 = self.get_euler("001","002","003","004","005","006")
        # t1,t2,t3,t4,t5,t6 = euler[0].z,euler[1].y,euler[2].y,euler[3].x,euler[4].y,euler[5].x
        nx = - math.sin(t5)*(math.sin(t1)*math.sin(t4) + math.cos(t4)*(math.cos(t1)*math.cos(t2)*math.sin(t3) + math.cos(t1)*math.cos(t3)*math.sin(t2))) - math.cos(t5)*(math.cos(t1)*math.sin(t2)*math.sin(t3) - math.cos(t1)*math.cos(t2)*math.cos(t3))
        ny = math.sin(t5)*(math.cos(t1)*math.sin(t4) - math.cos(t4)*(math.cos(t2)*math.sin(t1)*math.sin(t3) + math.cos(t3)*math.sin(t1)*math.sin(t2))) - math.cos(t5)*(math.sin(t1)*math.sin(t2)*math.sin(t3) - math.cos(t2)*math.cos(t3)*math.sin(t1))
        nz = - math.sin(t2 + t3)*math.cos(t5) - math.cos(t2 + t3)*math.cos(t4)*math.sin(t5)
        oz = math.cos(t2 + t3)*math.cos(t6)*math.sin(t4) - math.sin(t6)*(math.sin(t2 + t3)*math.sin(t5) - math.cos(t2 + t3)*math.cos(t4)*math.cos(t5))
        az = - math.cos(t6)*(math.sin(t2 + t3)*math.sin(t5) - math.cos(t2 + t3)*math.cos(t4)*math.cos(t5)) - math.cos(t2 + t3)*math.sin(t4)*math.sin(t6)
        tz0 = math.atan2(ny,nx)
        tx0 = math.atan2(oz,az)
        if -0.0001<=math.sin(tz0)<=0.0001 :
            cty=nx
        else:
            cty=ny/math.sin(tz0)
        ty0 = math.atan2(-nz,cty)
        return tx0,ty0,tz0
    
    def get_oritention(self):
        t1,t2,t3,t4,t5,t6 = self.get_euler("001","002","003","004","005","006")
        # t1,t2,t3,t4,t5,t6 = euler[0].z,euler[1].y,euler[2].y,euler[3].x,euler[4].y,euler[5].x
        # no theata6
        nx=-math.sin(t5)*(math.sin(t1)*math.sin(t4) + math.cos(t4)*(math.cos(t1)*math.cos(t2)*math.sin(t3) + math.cos(t1)*math.cos(t3)*math.sin(t2))) - math.cos(t5)*(math.cos(t1)*math.sin(t2)*math.sin(t3) - math.cos(t1)*math.cos(t2)*math.cos(t3))
        ny= math.sin(t5)*(math.cos(t1)*math.sin(t4) - math.cos(t4)*(math.cos(t2)*math.sin(t1)*math.sin(t3) + math.cos(t3)*math.sin(t1)*math.sin(t2))) - math.cos(t5)*(math.sin(t1)*math.sin(t2)*math.sin(t3) - math.cos(t2)*math.cos(t3)*math.sin(t1))
        nz= -math.sin(t2 + t3)*math.cos(t5) - math.cos(t2 + t3)*math.cos(t4)*math.sin(t5)
        oz=math.cos(t2 + t3)*math.sin(t4)
        az=math.cos(t2 + t3)*math.cos(t4)*math.cos(t5) - math.sin(t2 + t3)*math.sin(t5)

        tz0 = math.atan2(ny,nx)
        tx0 = math.atan2(oz,az)
        if -0.0001<=math.sin(tz0)<=0.0001 :
            cty=nx
        else:
            cty=ny/math.sin(tz0)
        ty0 = math.atan2(-nz,cty)
        print(f"(tx0,ty0,tz0)={tx0,ty0,tz0}")
        # (tx1,ty1,tz01) = self.get_oritention6()
        return (tx0,ty0,tz0)
    
    def get_theta1(self):
        theta1 = tuple()
        for t in (1,-1):
            theta1 +=([math.atan2(0,t)-math.atan2(self.d5*self.ny-self.py,self.px-self.d5*self.nx)],)
        return theta1
    
    def get_theta4(self,theta1:tuple):
        theta4 = tuple()
        for t in theta1:
            c4 = self.oy*math.cos(t[0]) - self.ox*math.sin(t[0])
            theta4 +=([t[0],math.atan2(math.sqrt(1-c4**2),c4)],)
            theta4 +=([t[0],math.atan2(-math.sqrt(1-c4**2),c4)],)
        return theta4

    def get_theta5(self,theta4:tuple):
        theta5 = tuple()
        for t in theta4:
            if -0.0001<=math.sin(t[1])<=0.0001:
                return 
            else:
                s5 = (self.ny*math.cos(t[0]) - self.nx*math.sin(t[0]))/math.sin(t[1])
                c5 =-(self.ay*math.cos(t[0]) - self.ax*math.sin(t[0]))/math.sin(t[1])
                theta5 +=([t[0],t[1],math.atan2(s5,c5)],)
        return theta5

    
    def get_theta23(self,theta5:tuple):
        theta23 = tuple()
        for t in theta5:
            s23 = (self.ox*math.cos(t[0]) + self.oy*math.sin(t[0]))/math.sin(t[1]) 
            c23 = self.oz/math.sin(t[1])
            theta23 +=([t[0],t[1],t[2],math.atan2(s23,c23)],)
        return theta23

    def get_theta2(self,theta23:tuple):
        theta2 = tuple()
        for t in theta23:
            temp_s2_1 = self.a1x + self.a3x*math.cos( t[3]) + self.d4*math.cos( t[3]) + self.a3z*math.sin( t[3])  - (self.d5*math.sin( t[3])*math.sin(t[1]+t[2]))/2 + self.d5*math.cos( t[3])*math.cos(t[2]) + (self.d5*math.sin(t[1]-t[2])*math.sin( t[3]))/2
            temp_c2_1 = self.a1z + self.a3z*math.cos( t[3])-self.a3x*math.sin( t[3])-self.d4*math.sin( t[3])-(self.d5*math.cos( t[3])*math.sin(t[1]+t[2]))/2-self.d5*math.sin( t[3])*math.cos(t[2])+(self.d5*math.sin(t[1]-t[2])*math.cos( t[3]))/2
            ts_2_1 = (self.px*math.cos(t[0])+self.py*math.sin(t[0])-temp_s2_1)/self.d2
            tc_2_1=(self.pz-self.d0-temp_c2_1)/self.d2
            t2 = math.atan2(ts_2_1,tc_2_1)
            theta2 +=([t[0],t2,t[3]-t2,t[1],t[2]],)
        return theta2

    def ik(self,tx,ty,tz,dx,dy,dz):
        """Inverse Kinematics"""
        (tx0,ty0,tz0) = (0,0,0)
        self.nx = math.cos(ty)*math.cos(ty0)*math.cos(tz)*math.cos(tz0) + math.cos(tx)*math.cos(tx0)*math.sin(tz)*math.sin(tz0) + math.sin(tx)*math.sin(tx0)*math.sin(tz)*math.sin(tz0) + math.cos(tx)*math.cos(tz)*math.sin(tx0)*math.sin(ty)*math.sin(tz0) - math.cos(tx)*math.cos(tz0)*math.sin(tx0)*math.sin(ty0)*math.sin(tz) - math.cos(tx0)*math.cos(tz)*math.sin(tx)*math.sin(ty)*math.sin(tz0) + math.cos(tx0)*math.cos(tz0)*math.sin(tx)*math.sin(ty0)*math.sin(tz) + math.cos(tx)*math.cos(tx0)*math.cos(tz)*math.cos(tz0)*math.sin(ty)*math.sin(ty0) + math.cos(tz)*math.cos(tz0)*math.sin(tx)*math.sin(tx0)*math.sin(ty)*math.sin(ty0)
        self.ny=math.cos(ty)*math.cos(ty0)*math.cos(tz0)*math.sin(tz) - math.cos(tx)*math.cos(tx0)*math.cos(tz)*math.sin(tz0) - math.cos(tz)*math.sin(tx)*math.sin(tx0)*math.sin(tz0) + math.cos(tx)*math.cos(tz)*math.cos(tz0)*math.sin(tx0)*math.sin(ty0) - math.cos(tx0)*math.cos(tz)*math.cos(tz0)*math.sin(tx)*math.sin(ty0) + math.cos(tx)*math.sin(tx0)*math.sin(ty)*math.sin(tz)*math.sin(tz0) - math.cos(tx0)*math.sin(tx)*math.sin(ty)*math.sin(tz)*math.sin(tz0) + math.cos(tx)*math.cos(tx0)*math.cos(tz0)*math.sin(ty)*math.sin(ty0)*math.sin(tz) + math.cos(tz0)*math.sin(tx)*math.sin(tx0)*math.sin(ty)*math.sin(ty0)*math.sin(tz)
        self.nz=math.cos(tx)*math.cos(ty)*math.sin(tx0)*math.sin(tz0) - math.cos(ty0)*math.cos(tz0)*math.sin(ty) - math.cos(tx0)*math.cos(ty)*math.sin(tx)*math.sin(tz0) + math.cos(tx)*math.cos(tx0)*math.cos(ty)*math.cos(tz0)*math.sin(ty0) + math.cos(ty)*math.cos(tz0)*math.sin(tx)*math.sin(tx0)*math.sin(ty0)
        self.ox=math.cos(ty)*math.cos(ty0)*math.cos(tz)*math.sin(tz0) - math.cos(tx)*math.cos(tx0)*math.cos(tz0)*math.sin(tz) - math.cos(tz0)*math.sin(tx)*math.sin(tx0)*math.sin(tz) - math.cos(tx)*math.cos(tz)*math.cos(tz0)*math.sin(tx0)*math.sin(ty) + math.cos(tx0)*math.cos(tz)*math.cos(tz0)*math.sin(tx)*math.sin(ty) - math.cos(tx)*math.sin(tx0)*math.sin(ty0)*math.sin(tz)*math.sin(tz0) + math.cos(tx0)*math.sin(tx)*math.sin(ty0)*math.sin(tz)*math.sin(tz0) + math.cos(tx)*math.cos(tx0)*math.cos(tz)*math.sin(ty)*math.sin(ty0)*math.sin(tz0) + math.cos(tz)*math.sin(tx)*math.sin(tx0)*math.sin(ty)*math.sin(ty0)*math.sin(tz0)
        self.oy=math.cos(tx)*math.cos(tx0)*math.cos(tz)*math.cos(tz0) + math.cos(tz)*math.cos(tz0)*math.sin(tx)*math.sin(tx0) + math.cos(ty)*math.cos(ty0)*math.sin(tz)*math.sin(tz0) + math.cos(tx)*math.cos(tz)*math.sin(tx0)*math.sin(ty0)*math.sin(tz0) - math.cos(tx)*math.cos(tz0)*math.sin(tx0)*math.sin(ty)*math.sin(tz) - math.cos(tx0)*math.cos(tz)*math.sin(tx)*math.sin(ty0)*math.sin(tz0) + math.cos(tx0)*math.cos(tz0)*math.sin(tx)*math.sin(ty)*math.sin(tz) + math.cos(tx)*math.cos(tx0)*math.sin(ty)*math.sin(ty0)*math.sin(tz)*math.sin(tz0) + math.sin(tx)*math.sin(tx0)*math.sin(ty)*math.sin(ty0)*math.sin(tz)*math.sin(tz0)
        self.oz=math.cos(tx0)*math.cos(ty)*math.cos(tz0)*math.sin(tx) - math.cos(tx)*math.cos(ty)*math.cos(tz0)*math.sin(tx0) - math.cos(ty0)*math.sin(ty)*math.sin(tz0) + math.cos(tx)*math.cos(tx0)*math.cos(ty)*math.sin(ty0)*math.sin(tz0) + math.cos(ty)*math.sin(tx)*math.sin(tx0)*math.sin(ty0)*math.sin(tz0)
        self.ax=math.cos(tx0)*math.cos(ty0)*math.sin(tx)*math.sin(tz) - math.cos(tx)*math.cos(ty0)*math.sin(tx0)*math.sin(tz) - math.cos(ty)*math.cos(tz)*math.sin(ty0) + math.cos(tx)*math.cos(tx0)*math.cos(ty0)*math.cos(tz)*math.sin(ty) + math.cos(ty0)*math.cos(tz)*math.sin(tx)*math.sin(tx0)*math.sin(ty)
        self.ay=math.cos(tx)*math.cos(ty0)*math.cos(tz)*math.sin(tx0) - math.cos(ty)*math.sin(ty0)*math.sin(tz) - math.cos(tx0)*math.cos(ty0)*math.cos(tz)*math.sin(tx) + math.cos(tx)*math.cos(tx0)*math.cos(ty0)*math.sin(ty)*math.sin(tz) + math.cos(ty0)*math.sin(tx)*math.sin(tx0)*math.sin(ty)*math.sin(tz)
        self.az=math.sin(ty)*math.sin(ty0) + math.cos(tx)*math.cos(tx0)*math.cos(ty)*math.cos(ty0) + math.cos(ty)*math.cos(ty0)*math.sin(tx)*math.sin(tx0)
        self.px=dx
        self.py=dy
        self.pz=dz
        list = [
            [self.nx,self.ox,self.ax,self.px],
            [self.ny,self.oy,self.ay,self.py],
            [self.nz,self.oz,self.az,self.pz],
            [0,0,0,1],
        ]
        print("matrix of self:")
        for m in list:
            print(f"\t{m}")
        # theta1  
        ans1 = self.get_theta1()
        # print(f"theta1:{ans1}")
        # theta5 
        ans4 = self.get_theta4(ans1)
        ans5 = self.get_theta5(ans4)
        if ans5 is not None:
            ans5 = self.get_theta23(ans5)
            ans5 = self.get_theta2(ans5)
        else:
            print("no theata")
            return
        return ans5
    
    def check_ik(self,P:tuple,ans:list):
        for list in ans:
            print(f"theta:{rad2angle(tuple(list))}")
            ans = self.fk(rad2angle(tuple(list)))
            tol=1e-5
            if abs(P[0]-ans[0,0])<tol and abs(P[1]-ans[1,0])<tol and abs(P[2]-ans[2,0])<tol:
                return list
            


class Mind(Robot_Kinematics,socket.socket):
    def __init__(self,host,port):
        Robot_Kinematics.__init__(self)  
        socket.socket.__init__(self,socket.AF_INET, socket.SOCK_STREAM)
        self.status = True
        self.payload = None
        self.bind((host,port))
        self.listen(1)
        self.settimeout(1)
        self.test()

    def test(self):
        def run():
            while self.status:
                try:
                    self.conn, addr = self.accept()
                    print("连接地址:",addr)
                    self.socket_receive(self.conn,addr)
                    # threading.Thread(target=self.socket_receive, args=(conn,addr)).start()
                except:
                    pass
            self.close()
            print("关闭Mind+连接")
        threading.Thread(target=run).start()

    def close_connect(self):
        self.status = False

    def socket_receive(self,conn,addr):
        # 接受消息
        while self.status:
            data = conn.recv(1024)
            if not data:
                print("Mind+连接断开")          
                break
            self.payload = json.loads(data.decode())
            conn.send(data)
            print("收到Mind+消息:", self.payload)
       

button_game = False
class BPY_timer(bpy.types.Operator,Robot_Kinematics):

    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    bl_options={"REGISTER","UNDO"}

    jn = tuple()
    isupdate = 0 # 

    def execute(self, context):  
        global button_game
        button_game = True   
        print(self)   
        print(context.window_manager)   
        self._timer = context.window_manager.event_timer_add(1/24, window=context.window) # 将计时器添加到窗口中
        context.window_manager.modal_handler_add(self) # 将模态处理函数添加到窗口中
        self.md = Mind("",5011)  # 服务端
        return {'RUNNING_MODAL'} # 返回到模态，保持运算符在blender上一直处于运行状态
    
    def recall(self,data=None):
        self.md.payload = None
        content = {'type':'finish',
                   'data':data,}
        try:
            self.md.conn.send(json.dumps(content).encode())
        except:
            pass
    # 模态函数
    def modal(self, context, event):       
        # 按ESC键退出运算符
        if event.type =='ESC':
            self.md.close_connect()
            self.cancel(context) # 运行"退出"函数
                        
            # 更新3d视图(如果有)
            for area in bpy.context.screen.areas:
                area.tag_redraw()
                                
            return {'CANCELLED'} # 退出模态运算符

        # 计时器事件(实时检测)
        if event.type == 'TIMER': # 如果是计时器事件，即操作物体
            if self.md.payload is not None:
                if self.md.payload["type"]=="get_oritention":
                    (tx0,ty0,tz0)= self.get_oritention()
                    (dx0,dy0,dz0) = self.get_world_pos("006")
                    self.recall((tx0,ty0,tz0,dx0,dy0,dz0))
                elif self.md.payload["type"]=="fk":
                    angle = tuple(self.md.payload["data"])
                    P = self.fk(angle)
                    self.recall(P)
                elif self.md.payload["type"]=="ik":
                    tx0,ty0,tz0,dx0,dy0,dz0 = tuple(self.md.payload["data"][1])
                    ans = self.ik(tx0,ty0,tz0,dx0,dy0,dz0)
                    rad = None
                    if ans:
                        rad = self.check_ik((dx0,dy0,dz0),ans)
                    self.recall(rad)
                elif self.md.payload["type"]=="move":
                    jn = angle2rad(self.md.payload["data"])
                    res = self.robot_move(jn,t=self.md.payload["t"],isupdate=self.isupdate,check_hit=self.md.payload["check_hit"])
                    if res:
                        self.isupdate=0
                        print("移动完成")
                        self.recall(res)
                    else:
                        self.isupdate+=1
            jn = self.get_euler("001","002","003","004","005","006")
            self.set_euler(("001","002","003","004","005","006"),jn)
        return {'PASS_THROUGH'} # 如果没有做事情，则绕过模态(这样在除了上面按键外，保留blender自身快捷键)

    # "退出"函数(命令取消时)
    def cancel(self, context):
        global button_game
        context.window_manager.event_timer_remove(self._timer) # 将给定的窗口中的计时器移除
        button_game = False 
    
# ui_type;固定各式
class ui_type:
    bl_space_type,bl_region_type,bl_context="VIEW_3D","UI","objectmode"     
# 面板1
class rpu_config(ui_type,bpy.types.Panel):  
    bl_idname="rpu_config"
    bl_label="传习机器人"                                                   
    bl_category="机械臂"                                                

    def draw(self,context):
        layout=self.layout
        scene=context.scene
        layout.label(text="configure",icon="BLENDER")
        row=layout.row()
        row1=layout.row()
        #生成按钮
        if not button_game:
            row.operator("wm.modal_timer_operator",text="连接mind+",icon="CUBE",depress=False)
        row1.operator("",text="连接mind+",icon="CUBE",depress=False)


classes=[
    BPY_timer,
    rpu_config,
]
def register():
    global button_game
    button_game=False
    for item in classes:
        bpy.utils.register_class(item)
    
def unregister():
    global button_game
    for item in classes:
        bpy.utils.unregister_class(item)
    button_game=False


if __name__=="__main__":
    register()

