import bpy
import math
old_pos1,old_pos2=None,None
# ser = serial.Serial("com4",115200)

a = 0.3  # 第一节骨骼长度
b = 0.240  # 第二节骨骼长度
check_precision = 0.001


def max_r():
    return (a + b) - check_precision

def min_r():
    return math.fabs(a - b) + check_precision

def jn2xy(j1, j2):
    x=a*math.cos(j1)+b*math.cos(j1+j2)
    y=a*math.sin(j1)+b*math.sin(j1+j2)
    return x,y

#x,y转j1,j2
def xy2jn(x, y):
        squa_r = x ** 2 + y ** 2
        r = squa_r ** (1 / 2)
        if (r > max_r()):
            print("无法到达的位置,max")
            return None
        if (r < min_r()):
            print("无法到达的位置,min")
            return None
        cos_R = (a ** 2 + b ** 2 - squa_r) / 2 / a / b
        j2 = math.pi - math.acos(cos_R)
        cos_B = (squa_r + a ** 2 - b ** 2) / 2 / a / r
        if (1 < cos_B < 1 + check_precision):
            cos_B = 1
        elif (-1 > cos_B > -1 - check_precision):
            cos_B = -1
        j1 = math.atan2(y, x) - math.acos(cos_B)
        return j1, j2

#pos转角度
def interp_q2jn(j1q, j2q): 
    Nmin,Nmax,Omin,Omax=0,4096,3.14159,-3.14159
    j1 = (Omax - Omin) / (Nmax - Nmin) * (j1q - Nmin) + Omin
    Nmin,Nmax,Omin,Omax=0,4096,-1.570796,4.712388
    j2 = (Omax - Omin) / (Nmax - Nmin) * (j2q - Nmin) + Omin
    return j1,j2
#角度转pos
def interp_jn2q(j1, j2): 
    Nmin,Nmax,Omin,Omax=0,4096,3.14159,-3.14159
    j1q = (Nmax - Nmin) /  (Omax - Omin) * (j1 - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=0,4096,-1.570796,4.712388
    j2q = (Nmax - Nmin) /  (Omax - Omin) * (j2 - Omin) + Nmin
    return math.floor(j1q),math.floor(j2q)

#x,y转m,n
def xy2mn(x,y): 
    Nmin,Nmax,Omin,Omax=0,18,0.225,-0.225
    m = (Nmax - Nmin) /  (Omax - Omin) * (y - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=0,18,0.06,0.51
    n = (Nmax - Nmin) /  (Omax - Omin) * (x - Omin) + Nmin
    return round(m),round(n)

#m,n转x,y
def mn2xy(m,n): 
    Nmin,Nmax,Omin,Omax=0,18,0.06,0.51
    x = (Omax - Omin) / (Nmax - Nmin) * (n - Nmin) + Omin
    Nmin,Nmax,Omin,Omax=0,18,0.225,-0.225
    y = (Omax - Omin) / (Nmax - Nmin) * (m - Nmin) + Omin
    return x,y
#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['B'].rotation_euler[1] 
    j2=bpy.data.objects['C'].rotation_euler[1]
    return j1,j2 

#双臂移动
def robot_move_two(x1,y1,x2,y2,v):
    game=Game1("B","b")
    old_j1,old_j2=robot_get_jn()
    new_j1,new_j2=xy2jn(x1,y1)
    j1,j2=new_j1-old_j1,new_j2-old_j2
    old_j3,old_j4=bpy.data.objects['b'].rotation_euler[1],bpy.data.objects['c'].rotation_euler[1]
    new_j3,new_j4=xy2jn(x2-0.61,y2)
    j3,j4=new_j3-old_j3,new_j4-old_j4
    for i in range(v):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['B'].rotation_euler[1] +=j1/v
        bpy.data.objects['C'].rotation_euler[1] +=j2/v
        bpy.data.objects['B'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['C'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['b'].rotation_euler[1] +=j3/v
        bpy.data.objects['c'].rotation_euler[1] +=j4/v
        bpy.data.objects['b'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['c'].keyframe_insert(data_path='rotation_euler')
        bool=game.DoCollisions()
        print("碰撞检测:",bool,"第",i,"帧")
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)


















class AABBox:
    def __init__(self,object):
        self.box = object
        a,b,c=self.get_object_size()
        self.width,self.deepth,self.heigth=a,b,c

    def get_location_world(self):
        x=bpy.data.objects[self.box].matrix_world[0][3]
        y=bpy.data.objects[self.box].matrix_world[1][3]
        z=bpy.data.objects[self.box].matrix_world[2][3]
        return x,y,z
    
    def get_object_rotation_y(self):
        return bpy.data.objects[self.box].rotation_euler[1]

    def get_object_size(self):
        return bpy.data.objects[self.box].dimensions
    
    #物体坐标基于圆心，且在底部
    def get_box_interval(self):
        x,y,z=self.get_location_world()
        r=self.heigth/2
        minX,maxX,minY,maxY,minZ,maxZ=x-r,x-r+self.width,y-r,y-r+self.heigth,z,z+self.deepth
        A,B,C,D=[minX,minY,minZ],[maxX,minY,minZ],[maxX,maxY,minZ],[minX,maxY,minZ]
        return A,B,C,D

    def rotation_matrix(self,vetor):
        rad=self.get_object_rotation_y()
        xb,yb,zb=self.get_location_world()
        vetor[0],vetor[1],vetor[2]=vetor[0]-xb,vetor[1]-yb,vetor[2]-zb
        point_x=vetor[0]*math.cos(rad)-vetor[1]*math.sin(rad)
        point_y=vetor[0]*math.sin(rad)+vetor[1]*math.cos(rad)
        point_z=vetor[2]
        vetor_new=[point_x+xb,point_y+yb,point_z+zb]
        return vetor_new
    #切换坐标系
    def Coordinate_change(self,vetor):
        rad=self.get_object_rotation_y()
        xb,yb,zb=self.get_location_world()
        point_x=vetor[0]*math.cos(-rad)-vetor[1]*math.sin(-rad)-xb
        point_y=vetor[0]*math.sin(-rad)+vetor[1]*math.cos(-rad)-yb
        point_z=vetor[2]-zb
        vetor_new=[point_x,point_y,point_z]
        return vetor_new

#BOxA为基准，判断BoxB是否在A
class Game1:
    def __init__(self,boxA,boxB):
        self.boxA=AABBox(boxA)
        self.boxB=AABBox(boxB)

    def update(self):
        A,B,C,D=self.boxA.get_box_interval()
        a,b,c,d=self.boxB.get_box_interval()
        a,b,c,d=self.boxB.rotation_matrix(a),self.boxB.rotation_matrix(b),self.boxB.rotation_matrix(c),self.boxB.rotation_matrix(d)
        return A,B,C,D,a,b,c,d

    def DoCollisions(self):
        #B_box相对于A
        A,B,C,D,a,b,c,d=self.update()
        a,b,c,d=self.boxA.Coordinate_change(a),self.boxA.Coordinate_change(b),self.boxA.Coordinate_change(c),self.boxA.Coordinate_change(d)
        min_x,max_x=min(a[0],b[0],c[0],d[0]),max(a[0],b[0],c[0],d[0])
        min_y,max_y=min(a[1],b[1],c[1],d[1]),max(a[1],b[1],c[1],d[1])
        # print("A:",A)
        # print("B:",B)
        # print("C:",C)
        # print("D:",D)
        # print("a:",a)
        # print("b:",b)
        # print("c:",c)
        # print("d:",d)
        # A,B,C,D=[minX,minY,minZ],[maxX,minY,minZ],[maxX,maxY,minZ],[minX,maxY,minZ]
        if(A[0]<=a[0]<=B[0] and B[1]<=a[1]<=C[1] and 0<=a[2]<=self.boxA.deepth):
            print("True_a")
            return True
        if(A[0]<=b[0]<=B[0] and B[1]<=b[1]<=C[1] and 0<=b[2]<=self.boxA.deepth):
            print("True_b")
            return True
        if(A[0]<=c[0]<=B[0] and B[1]<=c[1]<=C[1] and 0<=c[2]<=self.boxA.deepth):
            print("True_c")
            return True
        if(A[0]<=d[0]<=B[0] and B[1]<=d[1]<=C[1] and 0<=d[2]<=self.boxA.deepth):
            print("True_d")
            return True
        if (max_x>=B[0] and min_x<=A[0])or(max_y>=C[1] and min_y<=B[1]):
            print("True")
            return True
        print("False")
        return False


if __name__ == "__main__":
    print("开始:")
    robot_move_two(0.5,0,0.4,-0.05,80)
    robot_move_two(0,0.5,0.8,0.2,80)