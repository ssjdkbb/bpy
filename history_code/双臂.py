import bpy
import math
a = 0.31  # 第一节骨骼长度
b = 0.327  # 第二节骨骼长度
check_precision = 0

def max_r():
    return (a + b) - check_precision

def min_r():
    return math.fabs(a - b) + check_precision
def xy2jn1(x, y):
        squa_r = x ** 2 + y ** 2
        r = squa_r ** (1 / 2)
        if (r > max_r()):
            print("无法到达的位置,max")
            return None
        if (r < min_r()):
            print("无法到达的位置,min")
            return None
        cos_R = (a ** 2 + b ** 2 - squa_r) / 2 / a / b
        j2=math.pi-math.acos(cos_R)
        cos_B = (squa_r + a ** 2 - b ** 2) / 2 / a / r
        if (1 < cos_B < 1 + check_precision):
            cos_B = 1
        elif (-1 > cos_B > -1 - check_precision):
            cos_B = -1
        j1 = math.atan2(y,x)-math.acos(cos_B)
        return j1, j2

def xy2jn2(x, y):
        squa_r = x ** 2 + y ** 2
        r = squa_r ** (1 / 2)
        if (r > max_r()):
            print("无法到达的位置,max")
            return None
        if (r < min_r()):
            print("无法到达的位置,min")
            return None
        cos_R = (a ** 2 + b ** 2 - squa_r) / 2 / a / b
        j2 =math.acos(cos_R)-math.pi
        cos_B = (squa_r + a ** 2 - b ** 2) / 2 / a / r
        if (1 < cos_B < 1 + check_precision):
            cos_B = 1
        elif (-1 > cos_B > -1 - check_precision):
            cos_B = -1
        j1 = math.atan2(y, x) + math.acos(cos_B)
        return j1, j2
#set world coordinate
def set_location_world(object,x,y,z):
    bpy.data.objects[object].matrix_world[0][3]=x
    bpy.data.objects[object].matrix_world[1][3]=y
    bpy.data.objects[object].matrix_world[2][3]=z

# get world coordinate
def get_location_world(object):
    x=bpy.data.objects[object].matrix_world[0][3]
    y=bpy.data.objects[object].matrix_world[1][3]
    z=bpy.data.objects[object].matrix_world[2][3]
    return x,y,z
#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['MA'].rotation_euler[2] 
    j2=bpy.data.objects['MC'].rotation_euler[2]
    j3=bpy.data.objects['Ma'].rotation_euler[2] 
    j4=bpy.data.objects['Mc'].rotation_euler[2]
    return j1,j2,j3,j4



def jn2q(j):#弧度转成j1q,j2q(0,4096)
    Nmin,Nmax,Omin,Omax=-math.pi/2,math.pi*3/2,0,4096
    j1q = (Omax - Omin) / (Nmax - Nmin) * (j[0] - Nmin) + Omin
    Nmin,Nmax=-math.pi/2,math.pi*3/2
    j2q = (Omax - Omin) / (Nmax - Nmin) * (j[1] - Nmin) + Omin
    Nmin,Nmax,Omin,Omax=-math.pi/2,math.pi*3/2,0,4096
    j3q = (Omax - Omin) / (Nmax - Nmin) * (j[2] - Nmin) + Omin
    Nmin,Nmax=-math.pi*3/2,math.pi/2
    j4q = (Omax - Omin) / (Nmax - Nmin) * (j[3] - Nmin) + Omin
    return math.floor(j1q+0.5),math.floor(j2q+0.5),math.floor(j3q+0.5),math.floor(j4q+0.5)

#机械臂移动
def robot_move_xy(x,y,v):
    x1,y1,z1=get_location_world('MA')
    x2,y2,z2=get_location_world('Ma')
    x3,y3,z3=get_location_world('水杯')

    old_j1,old_j2,old_j3,old_j4=robot_get_jn()
    print("old_j1,old_j2:",old_j1,old_j2)
    new_j1,new_j2=xy2jn1(x-x1,y-y1)
    new_j3,new_j4=xy2jn2(x-x2,y-y2)
    print("new_j1,new_j2:",new_j1,new_j2)
    j1,j2,j3,j4=new_j1-old_j1,new_j2-old_j2,new_j3-old_j3,new_j4-old_j4
    for i in range(v):
        bpy.data.objects['MA'].rotation_euler[2] +=j1/v
        bpy.data.objects['MC'].rotation_euler[2] +=j2/v
        bpy.data.objects['Ma'].rotation_euler[2] +=j3/v
        bpy.data.objects['Mc'].rotation_euler[2] +=j4/v
        a,b,c=get_location_world('末端安装夹爪.001')
        bpy.data.objects['水杯'].location[0] =a
        bpy.data.objects['水杯'].location[1] =b
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        
        print(jn2q(robot_get_jn()))




if __name__ == "__main__":
    robot_move_xy(0.2,0.3,45)
    for i in range(5,1,-1):
        robot_move_xy(0,i*0.1,45)