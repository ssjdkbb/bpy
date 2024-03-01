import bpy 
import time 
import serial
import threading
import math

bpy.context.scene.frame_end = 500
bpy.context.scene.frame_start = 0
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


#激活物体
def choose_object(object):
    bpy.context.view_layer.objects.active=bpy.data.objects[object]
    bpy.data.objects[object].select_set(True)

def boo(object,object2):
    choose_object(object)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    bpy.data.objects[object].modifiers["布尔"].operation = 'INTERSECT'
    bpy.data.objects[object].modifiers["布尔"].use_hole_tolerant = True
    bpy.data.objects[object].modifiers["布尔"].object=bpy.data.objects[object2]
    bpy.ops.object.select_all(action='DESELECT')
    x,y,z=bpy.data.objects[object].dimensions
    print("x:",x)
    print("y:",y)
    print("z:",z)
    if x!=0 or y!=0 or z!=0:
        bpy.data.materials["材质"].node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.8, 0.0110605, 0.0134819, 1)
        bpy.data.objects[object2].hide_viewport = True
    else:
        bpy.data.materials["材质"].node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.775512, 0.76973, 0.8, 1)                                                            
        bpy.ops.object.modifier_remove(modifier="布尔")
        bpy.data.objects[object2].hide_viewport = False
#        bpy.ops.object.modifier_remove(modifier="布尔")
#    bpy.ops.object.modifier_apply(modifier="布尔", report=True)
#    bpy.data.objects.remove(bpy.data.objects["ball"])
def booc(object,object2):
    choose_object(object)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    bpy.data.objects[object].modifiers["布尔"].operation = 'INTERSECT'
    bpy.data.objects[object].modifiers["布尔"].use_hole_tolerant = True
    bpy.data.objects[object].modifiers["布尔"].object=bpy.data.objects[object2]
    bpy.ops.object.select_all(action='DESELECT')
    x,y,z=bpy.data.objects[object].dimensions
    print("x:",x)
    print("y:",y)
    print("z:",z)
    if x!=0 or y!=0 or z!=0:
        bpy.data.materials["C"].node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.0115161, 0.00215862, 0.8, 1)
        bpy.data.objects[object2].hide_viewport = True
    else:
        bpy.data.materials["C"].node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.775512, 0.76973, 0.8, 1)
        bpy.data.objects[object2].hide_viewport = False
        bpy.ops.object.modifier_remove(modifier="布尔")
#    bpy.ops.object.modifier_apply(modifier="布尔", report=True)
#    bpy.data.objects.remove(bpy.data.objects["ball"])



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
#绕x旋转 
def rotation_X(object,theta):
    bpy.data.objects[object].rotation_euler[0]=theta

#绕y旋转 
def rotation_Y(object,theta):
    bpy.data.objects[object].rotation_euler[1]=theta

#绕z旋转 
def rotation_Z(object,theta):
    bpy.data.objects[object].rotation_euler[2]=theta

#设置父级
def chose_parent(child,parent):
    bpy.data.objects[child].parent = bpy.data.objects[parent]

##取消父级
def cancel_parent(child):
    bpy.data.objects[child].parent =None

#创造矩阵
def create_matrix(row,col):
    matrix = [[j for j in range(col)] for i in range(row)]
    return matrix

#复制物体
def clone(object,x,y,z):
    bpy.data.objects[object].select_set(True)
    bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False, "mode":'TRANSLATION'}, TRANSFORM_OT_translate={"value":(x, y, z), "orient_axis_ortho":'X', "orient_type":'GLOBAL', "orient_matrix":((1, 0, 0), (0, 1, 0), (0, 0, 1)), "orient_matrix_type":'GLOBAL', "constraint_axis":(False, False, False), "mirror":False, "use_proportional_edit":False, "proportional_edit_falloff":'SMOOTH', "proportional_size":1, "use_proportional_connected":False, "use_proportional_projected":False, "snap":False, "snap_elements":{'INCREMENT'}, "use_snap_project":False, "snap_target":'CLOSEST', "use_snap_self":False, "use_snap_edit":False, "use_snap_nonedit":False, "use_snap_selectable":False, "snap_point":(0, 0, 0), "snap_align":False, "snap_normal":(0, 0, 0), "gpencil_strokes":False, "cursor_transform":False, "texture_space":False, "remove_on_cancel":False, "view2d_edge_pan":False, "release_confirm":False, "use_accurate":False, "use_automerge_and_split":False})
    bpy.data.objects[object+".001"].name="clone"
    # bpy.ops.object.parent_set(keep_transform=True)# 保持初始的位置
    # set_location_world(object,-0.03,0.06,0)
    # rotation_X(object,0)
    bpy.ops.object.select_all(action='DESELECT')
# 吸盘下降
def Chuck_down(s=0.027,N=30):
    k=s/N
    for i in range(N):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['Chuck'].location[1] +=k
        bpy.data.objects['Chuck'].keyframe_insert(data_path='location')
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

# 吸盘上升
def Chuck_up(s=0.027,N=30):
    k=s/N
    for i in range(N):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['Chuck'].location[1] -=k 
        bpy.data.objects['Chuck'].keyframe_insert(data_path='location')
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

#吸盘_吸
def Chuck_sip(object):
    chose_parent(object,"Chuck")        # 3 选择父亲
    x,y,z=get_location_world(object)
    bpy.ops.object.parent_set(keep_transform=True)# 保持初始的位置
    set_location_world(object,-0.03,0.06,0)
    rotation_X(object,0)

#吸盘_呼
def Chuck_hoo(object):
    Chess2board(object)


#相对棋盘坐标
def Chess2board_xy(m,n): 
    x,y,z=0.02,0.01,0.02    #x:行；z:列
    x,z=x+n*0.025,z+m*0.025
    return x,y,z

def Chess2board_mn():
    pass

#绑定棋盘19x19
def Chess2board(object):
    j1,j2=robot_get_jn()
    x,y=jn2xy(j1,j2)
    m,n=xy2mn(x,y)
    print(m,n)
    chose_parent(object,"Chessboard")
    x,y,z=Chess2board_xy(m,n)
    set_location_xyz(object,x,y,z)


#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['B'].rotation_euler[1] 
    j2=bpy.data.objects['C'].rotation_euler[1]
    return j1,j2 

#机械臂移动
def robot_move_xy(x,y,v):
    old_j1,old_j2=robot_get_jn()
    new_j1,new_j2=xy2jn(x,y)
    j1,j2=new_j1-old_j1,new_j2-old_j2
    for i in range(v):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['Control1'].rotation_euler[1] +=j1/v
        bpy.data.objects['Control2'].rotation_euler[1] +=j2/v
        bpy.data.objects['Control1'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control2'].keyframe_insert(data_path='rotation_euler')
        boo("B","b")
        booc("C","c")
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        choose_object("B")
        bpy.ops.object.modifier_remove(modifier="布尔")
        choose_object("C")
        bpy.ops.object.modifier_remove(modifier="布尔")

def robot_move_mn(m,n,v):
    x,y=mn2xy(m,n)
    robot_move_xy(x,y,v)

def ser_close():
    print("close")
    ser.close()

#现实映射blender
def lua2bpy(): 
    i=0
    while(ser.isOpen()):
        time.sleep(0.0001)
        count=ser.inWaiting()
        if count>0:
            read_data=ser.read(count)       
            if 3<=len(read_data)<=9:
                res = str(read_data)
                res=res.split(',')
                print("读取值:",int(res[0][2:]),int(res[1][:-1])) 
                if int(res[0][2:])==0 and int(res[1][:-1])==0:
                    Chuck_up(0.02)
                elif int(res[0][2:])==0 and int(res[1][:-1])==1:
                    Chuck_down(0.02)
                    Chuck_sip("Chess pieces1")
                    clone("Chess pieces1",0,0,0)
                else:
                    j1,j2=interp_q2jn(int(res[0][2:]),int(res[1][:-1]))
                    control1 = bpy.data.objects['Control1']
                    control2 = bpy.data.objects['Control2']
                    bpy.context.scene.frame_set(i)
                    control1.rotation_euler[1] = j1
                    control2.rotation_euler[1] = j2
                    control2.keyframe_insert(data_path='rotation_euler')
                    control1.keyframe_insert(data_path='rotation_euler')
                    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
                i+=1
                return lua2bpy()

#blender映射现实
def bpy2lua():
    global old_pos1,old_pos2
    while(ser.isOpen()):
        time.sleep(0.0001)
        j1=bpy.data.objects['Control1'].rotation_euler[1]
        j2=bpy.data.objects['Control2'].rotation_euler[1]
        pos1,pos2=interp_jn2q(j1,j2)
        if old_pos1!=pos1 or old_pos2!=pos2:
            old_pos1,old_pos2=pos1,pos2
            print("当前值:",pos1,pos2)
            ser.write(bytes.fromhex(hex(pos1)[2:].zfill(4)+hex(pos2)[2:].zfill(4)))
            return bpy2lua()

def text():
    j1,j2=0
    total_frames = 200 
    bpy.context.scene.frame_end = total_frames
    bpy.context.scene.frame_start = 0
    for i in range(total_frames):
        bpy.context.scene.frame_set(i)
        bpy.data.objects['Control1'].rotation_euler[1] = j1
        bpy.data.objects['Control2'].rotation_euler[1] = j2
        bpy.data.objects['Control1'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control2'].keyframe_insert(data_path='rotation_euler')
        a-=0.02708
    bpy.ops.screen.frame_offset(delta=-99)

def test(m,n):

    Chuck_down()
    Chuck_sip("Chess pieces1")
    Chuck_up()
    robot_move_mn(m,n,18)
    Chuck_down()
    Chuck_hoo("Chess pieces1")
    Chuck_up()
    clone("Chess pieces1",0,0,0)
    robot_move_xy(-0.03,0.06,18)

#遍历所有点
def all(m,n):
    for i in range(m):
        for j in range(n):
            if (i<=2 and j==18) or (i==10 and j==10) or (i==18 and j==18):
                pass
            else:
                test(i,j)


def get_Vetor(object,Vetor=[0,0]):
    pass

#双臂移动
def robot_move_two(x1,y1,x2,y2,v):
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
        boo("B","b")
        booc("C","c")
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        choose_object("B")
        bpy.ops.object.modifier_remove(modifier="布尔")
        choose_object("C")
        bpy.ops.object.modifier_remove(modifier="布尔")

#双臂协同
def work_together(object1):
    pass


if __name__ == "__main__":
    # timer=threading.Timer(20,ser_close)
    # timer.start()
    # lua2bpy()
    # thread1 = threading.Thread(target=redraw)
    # thread1.start()
    # all(19,19)  
    # robot_move_mn(10,10,20)
    # robot_move_xy(-0.03,0.06,20)
    robot_move_two(0.5,0,0.4,-0.05,20)
    robot_move_two(0,0.5,0.8,0.2,20)







