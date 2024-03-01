bl_info = {
    "name": "Test Panel",
    "author": "zzk",
    "version": (0, 0, 1),
    "blender": (2, 80, 0),
    "location": "",
    "description": "",
    "category": "Generic",
}

import bpy  
import time 
import serial
import threading
import math
import cv2



temp=0
click=0
v=20   #速度
old_pos1,old_pos2=None,None

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
    Nmin,Nmax,Omin,Omax=0,4096,math.pi,-math.pi
    j1 = (Omax - Omin) / (Nmax - Nmin) * (j1q - Nmin) + Omin
    Nmin,Nmax,Omin,Omax=0,4096,-math.pi*3/2,math.pi/2
    j2 = (Omax - Omin) / (Nmax - Nmin) * (j2q - Nmin) + Omin
    return j1,j2
#角度转pos
def interp_jn2q(j1, j2): 
    Nmin,Nmax,Omin,Omax=0,4096,math.pi,-math.pi
    j1q = (Nmax - Nmin) /  (Omax - Omin) * (j1 - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=0,4096,-math.pi*3/2,math.pi/2
    j2q = (Nmax - Nmin) /  (Omax - Omin) * (j2 - Omin) + Nmin
    return math.floor(j1q+0.5),math.floor(j2q+0.5)

#x,y转m,n
def xy2mn(x,y): 
    Nmin,Nmax,Omin,Omax=0,18,0.225,-0.225
    m = (Nmax - Nmin) /  (Omax - Omin) * (y - Omin) + Nmin
    Nmin,Nmax,Omin,Omax=0,18,0.06,0.51
    n = (Nmax - Nmin) /  (Omax - Omin) * (x - Omin) + Nmin
    return math.floor(m+0.5),math.floor(n+0.5)

#m,n转x,y
def mn2xy(m,n): 
    Nmin,Nmax,Omin,Omax=0,18,0.06,0.51
    x = (Omax - Omin) / (Nmax - Nmin) * (n - Nmin) + Omin
    Nmin,Nmax,Omin,Omax=0,18,0.225,-0.225
    y = (Omax - Omin) / (Nmax - Nmin) * (m - Nmin) + Omin
    return x,y


#激活物体
def choose_object(object):
    bpy.context.view_layer.objects.active=bpy.data.objects[object]

#set world coordinate
def set_location_world(object,x,y,z):
    bpy.data.objects[object].matrix_world[0][3]=x
    bpy.data.objects[object].matrix_world[1][3]=y
    bpy.data.objects[object].matrix_world[2][3]=z

#get world coordinate
def get_location_world(object):
    x=bpy.data.objects[object].matrix_world[0][3]
    y=bpy.data.objects[object].matrix_world[1][3]
    z=bpy.data.objects[object].matrix_world[2][3]
    return x,y,z

#设置相对位置
def set_location_xyz(object,x,y,z):
    bpy.data.objects[object].location[0]=x
    bpy.data.objects[object].location[1]=y
    bpy.data.objects[object].location[2]=z

#获取相对位置
def get_location_xyz(object):
    x=bpy.data.objects[object].location[0]
    y=bpy.data.objects[object].location[1]
    z=bpy.data.objects[object].location[2]
    return x,y,z

#创造矩阵
def create_matrix(row,col):
    matrix = [[j for j in range(col)] for i in range(row)]
    return matrix




#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['Control1'].rotation_euler[1] 
    j2=bpy.data.objects['Control2'].rotation_euler[1]
    return j1,j2

#机械臂移动
def robot_move_xy(x,y,v):
    old_j1,old_j2=robot_get_jn()
    new_j1,new_j2=xy2jn(x,y)
    j1,j2=new_j1-old_j1,new_j2-old_j2
    for i in range(v):
        bpy.data.objects['Control1'].rotation_euler[1] +=j1/v
        bpy.data.objects['Control2'].rotation_euler[1] +=j2/v
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)



bpy_status=0
lua_status=0
panduan_status=0
iswrite=0   #是否发送状态值
head="FF FF 01" #头：FF FF id:01 状态:

def panduan():  #判断能否通讯,以何种方式通讯
    global bpy_status,lua_status,panduan_status,iswrite
    if ser.isOpen() :
        if temp == 0 and iswrite==0 : # lua_control
            bpy_status=0
            ser.write(bytes.fromhex(head+"00"))
            iswrite=1
        elif temp == 1 and iswrite==0 : # control_lua
            bpy_status=1
            ser.write(bytes.fromhex(head+"01"))
            iswrite=1
        count=ser.inWaiting()
        if count>0:
            read_data=ser.read(count).decode('UTF-8','strict')  #解码后为str类型
            if len(read_data)==8:
                for i in range(3):
                    ser.write(bytes.fromhex(head+"FE"))  #第二次发送
                    time.sleep(0.01)
                lua_status=int(read_data[6:8])
            if bpy_status==0 and lua_status==1:
                if len(read_data)>=14:
                    lua_control(read_data[:14])
        if bpy_status==1 and lua_status==0:
            control_lua()
        if bpy_status==1 and lua_status==1:
            ser.write(bytes.fromhex(head+"01"))
            print('hhhhhhhhhhhhhhhhhhhhhhhhhh')
        if bpy_status==0 and lua_status==0:
            ser.write(bytes.fromhex(head+"00"))
            print("jjjjjjjjjjjjjjjjjj")

def lua_control(read_data):
    print("lua位置:",int(read_data[6:10],16),int(read_data[10:],16))
    # bpy.data.objects["Control1"].rotation_euler[1]+=0.01
    # bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

def control_lua():
    j1=bpy.data.objects['Control1'].rotation_euler[1]
    j2=bpy.data.objects['Control2'].rotation_euler[1]
    pos1,pos2=interp_jn2q(j1,j2)
    ser.write(bytes.fromhex(head+hex(pos1)[2:].zfill(4)+hex(pos2)[2:].zfill(4)))  
    print("blender正在控制机械臂")
#___________创建计时器以检测用户是否移除功能___________
def iswork():
    global iswrite
    panduan()    #循环
    if click==0:
        return None # 返回 None 给计时器，计时器会停止
    return 0.001


def asize(self):
    return bpy.data.objects["Control1"].rotation_euler[1]
def bsize(self):
    return bpy.data.objects["Control2"].rotation_euler[1]
def csize(self):
    rad=bpy.data.objects["Control1"].rotation_euler[1]
    return rad/math.pi*180
def dsize(self):
    rad=bpy.data.objects["Control2"].rotation_euler[1]
    return rad/math.pi*180
def Joint1(self,context):
    old_j1=bpy.data.objects["Control1"].rotation_euler[1]
    new_j1=bpy.data.scenes["Scene"].RNARotaition1/180*math.pi
    j1=new_j1-old_j1
    for i in range(abs(math.floor(j1*10))):
        bpy.data.objects['Control1'].rotation_euler[1] +=j1/abs(math.floor(j1*10))
    bpy.data.objects["Control1"].rotation_euler[1]=bpy.data.scenes["Scene"].RNARotaition1/180*math.pi
def Joint2(self,context):
    old_j2=bpy.data.objects["Control2"].rotation_euler[1]
    new_j2=bpy.data.scenes["Scene"].RNARotaition2/180*math.pi
    j2=new_j2-old_j2
    for i in range(abs(math.floor(j2*10))):
        bpy.data.objects['Control2'].rotation_euler[1] +=j2/abs(math.floor(j2*10))
    bpy.data.objects["Control2"].rotation_euler[1]=bpy.data.scenes["Scene"].RNARotaition2/180*math.pi



#RNA属性
def myProperty():
    #浮点类型
    bpy.types.Scene.RNAFloat1=bpy.props.FloatProperty(name="RNAFloat",get=asize)
    bpy.types.Scene.RNAFloat2=bpy.props.FloatProperty(name="RNAFloat",get=bsize)
    bpy.types.Scene.RNAFloat3=bpy.props.FloatProperty(name="RNAFloat",get=csize)
    bpy.types.Scene.RNAFloat4=bpy.props.FloatProperty(name="RNAFloat",get=dsize)
    #三维向量
    # bpy.types.Scene.RNAPosition=bpy.props.FloatVectorProperty(name="rotation",subtype="XYZ",step=1,update=fsize)
    bpy.types.Scene.RNARotaition1=bpy.props.FloatProperty(name="Joint1",min=-180,max=180,step=10,update=Joint1)
    bpy.types.Scene.RNARotaition2=bpy.props.FloatProperty(name="Joint2",min=-180,max=180,step=10,update=Joint2)


class TEST_OT_control2lua(bpy.types.Operator):
    bl_idname="arm.control2lua"
    bl_label="control2lua"  #别名
    def execute(self,context): #执行函数
        global temp,click,panduan_status,bpy_status,lua_status,iswrite
        if temp==0: #按下时候控制lua
            temp=1
            iswrite=0
            click+=1
        elif temp==1:
            temp=0
            iswrite=0
            click+=1
        if click==1:
            bpy.app.timers.register(iswork)                  
        return {"FINISHED"}
    
class TEST_OT_font(bpy.types.Operator):
    bl_idname="arm.font"
    bl_label="font"  #别名

    def execute(self,context): #执行函数
        global v
        x,y,z=get_location_world("Control3")
        robot_move_xy(x,y+0.1,v)
        print("y=",get_location_world("Control3")[1]-y)
        return {"FINISHED"}
class TEST_OT_behind(bpy.types.Operator):
    bl_idname="arm.behind"
    bl_label="behind"  #别名

    def execute(self,context): #执行函数
        global v
        x,y,z=get_location_world("Control3")
        robot_move_xy(x,y-0.1,v)
        print("y=",get_location_world("Control3")[1]-y)
        return {"FINISHED"}

class TEST_OT_left(bpy.types.Operator):
    bl_idname="arm.left"
    bl_label="left"  #别名
    bl_options={"REGISTER","UNDO"} #提示信息

    def execute(self,context): #执行函数
        global v
        x,y,z=get_location_world("Control3")
        robot_move_xy(x-0.1,y,v)
        print("y=",get_location_world("Control3")[1]-y)
        return {"FINISHED"}

#插件配置
class TEST_OT_hello(bpy.types.Operator):
    bl_idname="arm.right"
    bl_label="hello"  #别名
    bl_options={"REGISTER","UNDO"}

    # location_y : bpy.props.StringProperty(name="mSring",default="blender")

    def execute(self,context): #执行函数
        global v
        x,y,z=get_location_world("Control3")
        robot_move_xy(x+0.1,y,v)
        print("y=",get_location_world("Control3")[1]-y)
        # self.location_y=str(get_location_world("Control3")[1])
        # self.report({"INFO"},self.location_y)
        return {"FINISHED"}

    # def invoke(self,context,event):
    #     return context.window_manager.invoke_props_dialog(self)

class TEST_PT_view3d(bpy.types.Panel):
    bl_idname="TESR_PT_view3d"
    bl_label="机械臂控制"  #别名

    #标签分类
    bl_category="机械臂"

    # ui_type
    bl_space_type="VIEW_3D" 
    bl_region_type="UI" 
    bl_context="objectmode" 
    def __init__(self):
        ports = serial.tools.list_ports.comports()
        ser = serial.Serial("com49", 115200)

    def draw(self,context):
        global temp
        layout=self.layout
        layout.label(text="arms infomation",icon="BLENDER")#text：标题；icon:图标
        row1=layout.row()
        row2=layout.row()
        layout.label(text="arms control",icon="BLENDER")#text：标题；icon:图标
        col1=layout.column()
        row3=layout.row()
        row4=layout.row()
        col=layout.column()
        scene=context.scene
        row1.prop(scene,"RNAFloat1",text="rad1")
        row1.prop(scene,"RNAFloat2",text="rad2")
        row2.prop(scene,"RNAFloat3",text="ang1")
        row2.prop(scene,"RNAFloat4",text="ang2")
        col.prop(scene,"RNARotaition1",text="Joint1")
        col.prop(scene,"RNARotaition2",text="Joint2")

        #生成按钮;prop:填充
        if temp==0 :
            col1.operator("arm.control2lua",text="下位机控制",icon="CUBE",depress=False)
        elif temp==1:
            col1.operator("arm.control2lua",text="下位机控制",icon="CUBE",depress=True)
        row3.operator("arm.font",text="前进",icon="CUBE",depress=False)
        row3.operator("arm.behind",text="后退",icon="CUBE",depress=False)
        row4.operator("arm.left",text="向左",icon="CUBE",depress=False)
        row4.operator("arm.right",text="向右",icon="CUBE",depress=False)

class TEST_PT_view3d1(bpy.types.Panel):
    bl_idname="TESR_PT_view3d1"
    bl_label="机械臂视觉"  #别名

    #标签分类
    bl_category="机械臂"

    # ui_type
    bl_space_type="VIEW_3D" 
    bl_region_type="UI" 
    bl_context="objectmode" 

    def draw(self,context):
        global temp
        layout=self.layout
        layout.label(text="arms infomation",icon="BLENDER")#text：标题；icon:图标
        row1=layout.row()
        row2=layout.row()
        layout.label(text="arms control",icon="BLENDER")#text：标题；icon:图标
        col1=layout.column()
        row3=layout.row()
        row4=layout.row()
        col=layout.column()
        scene=context.scene
        row1.prop(scene,"RNAFloat1",text="rad1")
        row1.prop(scene,"RNAFloat2",text="rad2")
        row2.prop(scene,"RNAFloat3",text="ang1")
        row2.prop(scene,"RNAFloat4",text="ang2")
        col.prop(scene,"RNARotaition1",text="Joint1")
        col.prop(scene,"RNARotaition2",text="Joint2")

        #生成按钮;prop:填充
        if temp==0 :
            col1.operator("arm.control2lua",text="下位机控制",icon="CUBE",depress=False)
        elif temp==1:
            col1.operator("arm.control2lua",text="下位机控制",icon="CUBE",depress=True)
        row3.operator("arm.font",text="前进",icon="CUBE",depress=False)
        row3.operator("arm.behind",text="后退",icon="CUBE",depress=False)
        row4.operator("arm.left",text="向左",icon="CUBE",depress=False)
        row4.operator("arm.right",text="向右",icon="CUBE",depress=False)



def register():
    #RNA属性创建
    myProperty()
    # bpy.app.timers.register(lua2bpy)   
    bpy.utils.register_class(TEST_OT_control2lua)
    bpy.utils.register_class(TEST_OT_font)
    bpy.utils.register_class(TEST_OT_behind)
    bpy.utils.register_class(TEST_OT_left)
    bpy.utils.register_class(TEST_OT_hello)
    bpy.utils.register_class(TEST_PT_view3d)
    bpy.utils.register_class(TEST_PT_view3d1)

def unregister():
    ser.close()
    bpy.utils.unregister_class(TEST_OT_control2lua)
    bpy.utils.unregister_class(TEST_OT_font)
    bpy.utils.unregister_class(TEST_OT_behind)
    bpy.utils.unregister_class(TEST_OT_left)
    bpy.utils.unregister_class(TEST_OT_hello)
    bpy.utils.unregister_class(TEST_PT_view3d)
    bpy.utils.unregister_class(TEST_PT_view3d1)

if __name__ == "__main__":
    register()