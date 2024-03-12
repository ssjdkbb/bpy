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
    rad = []
    for a in angle:
        rad.append(a/180*math.pi)
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
    
    def set_euler(self,obj:tuple,theta:tuple):
        for idx,name in enumerate(obj):
            for na,dir in self.eluer.items():
                if name in self.obj_name:
                    if name==na:
                        rad = theta[idx]/180*math.pi
                        bpy.data.objects[na].rotation_euler[dir]=rad
                        if name=="004" and theta[idx]==0:
                            rad = 0/180*math.pi
                            bpy.data.objects[na].rotation_euler[dir]=rad
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        
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

    def robot_move(self,jn,t=2000):
        """机械臂移动"""
        old_jn = self.get_euler("001","002","003","004","005","006")
        times = int(t/60)    # ms
        speed = [nj-oj for oj,nj in zip(old_jn,jn)]
        for i in range(times):
            bpy.data.objects['001'].rotation_euler.z+=speed[0]/times
            bpy.data.objects['002'].rotation_euler.y+=speed[1]/times
            bpy.data.objects['003'].rotation_euler.y+=speed[2]/times
            bpy.data.objects['004'].rotation_euler.x+=speed[3]/times
            bpy.data.objects['005'].rotation_euler.y+=speed[4]/times
            if self.check_hit("005",("000","001","002")) or self.check_hit("004",("000","001","002")):
                print("发生碰撞")
                return 
            bpy.ops.object.select_all(action='DESELECT')
            bpy.context.view_layer.objects.active=bpy.data.objects["006"]
            bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        print(f"目标角度:{rad2angle(jn)}")
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
        return P0

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
        (tx1,ty1,tz01) = self.get_oritention6()
        return (tx0,ty0,tz0),tx1-tx0
    
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
            print("no theata",tz,ty)
            p5 = ((dx/np.cos(tz)-self.d5*np.cos(ty))*np.cos(tz),dy,dz+self.d5*np.sin(ty))
            print(f"p5:{p5}")
            p5=((dx/np.cos(tz)-self.d5*np.cos(ty))-self.a1x,dz+self.d5*np.sin(ty)-self.d0-self.a1z)
            sqrt_r = p5[0]**2+p5[1]**2
            sqrt_b =(self.a3x+self.d4)**2+self.a3z**2
            cos_2k = (self.d2**2+sqrt_b-sqrt_r)/2/self.d2/(sqrt_b**0.5)
            
            print(f"annn:{rad2angle((math.acos(cos_2k),))}")
            
            self.get_world_pos("005")
        return ans5
    
    def check_ik(self,P:tuple,ans:list):
        for list in ans5:
            print(f"theta:{rad2angle(tuple(list))}")
            ans = self.fk(rad2angle(tuple(list)))
            tol=1e-5
            if abs(P[0]-ans[0,0])<tol and abs(P[1]-ans[1,0])<tol and abs(P[2]-ans[2,0])<tol:
                return list

    
class Mind(Robot_Kinematics,socket.socket):
    def __init__(self,host,port):
        Robot_Kinematics.__init__(self)  
        self.status = True
        socket.socket.__init__(self,socket.AF_INET, socket.SOCK_STREAM)
        self.bind((host,port))
        self.listen(50)
        self.test()

    def socket_receive(self,conn,addr):
        count = 0
        dircy = {}
        # 接受消息
        while self.status:
            count+=1
            dircy["count"]=count
            conn.send(json.dumps(dircy).encode())
            print(f"Mind+:{count}")
            time.sleep(3)
    def test(self):
        def run():
            self.settimeout(0.5)
            while self.status:
                try:
                    conn, addr = self.accept()
                    print("连接地址:",addr)
                    threading.Thread(target=self.socket_receive, args=(conn,addr)).start()
                except:
                    pass
            self.close()
            print("关闭Mind+连接")
        threading.Thread(target=run).start()

    def close_connect(self):
        self.status = False

class Robot_ZDFX0808(Robot_Kinematics):
    def __init__(self,host,port):
        socket.socket.__init__(self,socket.AF_INET, socket.SOCK_STREAM)
        self.settimeout(1)
        try:
            self.connect((host, port))
        except:
            print("连接机械臂超时")

if __name__=="__main__":
    # rk = Mind("localhost",5012) # 服务端
    rk = Robot_Kinematics() 
    jn_init = rk.get_euler("001","002","003","004","005","006")
    jn_init =(0,)*6
    jn_goal = (45,)+(30,)+(10,)+(0,)+(45,0)
    rk.set_euler(("001","002","003","004","005","006"),jn_goal)
    rk.get_world_oritention("006")
    (tx0,ty0,tz0),rad= rk.get_oritention()
    (dx0,dy0,dz0) = rk.get_world_pos("006")
    # # 回到初始姿态
    # rk.set_euler(("001","002","003","004","005","006"),rad2angle(jn_init))
    
    ans5 = rk.ik(tx0,ty0,tz0,dx0,dy0,dz0)
    # rad = rk.check_ik((dx0,dy0,dz0),ans5)
    # rk.robot_move(rad,3000)
    # md.close_connect()
