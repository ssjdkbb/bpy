import bpy 
import time 
import serial
import threading
import math
bpy.context.scene.frame_end = 500
bpy.context.scene.frame_start = 0


#设置机械臂角度(世界坐标系)
def robot_set_jn(j1,j2,j3,j4,j5,j6):
    bpy.data.objects['Control1'].rotation_euler[1]=j1
    bpy.data.objects['Control2'].rotation_euler[1]=j2
    bpy.data.objects['Control3'].rotation_euler[1]=j3
    bpy.data.objects['Control4'].rotation_euler[1]=j4
    bpy.data.objects['Control5'].rotation_euler[1]=j5
    bpy.data.objects['Control6'].rotation_euler[1]=j6
#获取机械臂角度(世界坐标系)
def robot_get_jn():
    j1=bpy.data.objects['Control1'].rotation_euler[1] 
    j2=bpy.data.objects['Control2'].rotation_euler[1]
    j3=bpy.data.objects['Control3'].rotation_euler[1] 
    j4=bpy.data.objects['Control4'].rotation_euler[1]
    j5=bpy.data.objects['Control5'].rotation_euler[1] 
    j6=bpy.data.objects['Control6'].rotation_euler[1]
    return j1,j2,j3,j4,j5,j6

#机械臂移动
def robot_move(v):
    j1,j2,j3,j4,j5,j6=robot_get_jn()
    j1,j2,j3,j4,j5,j6=1.75,1.5,1.25,1,0.75,0.5
    for i in range(v):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['Control1'].rotation_euler[1] +=j1/v
        bpy.data.objects['Control1'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control2'].rotation_euler[1] +=j2/v
        bpy.data.objects['Control2'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control3'].rotation_euler[1] +=j3/v
        bpy.data.objects['Control3'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control4'].rotation_euler[1] +=j4/v
        bpy.data.objects['Control4'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control5'].rotation_euler[1] +=j5/v
        bpy.data.objects['Control5'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control6'].rotation_euler[1] +=j6/v
        bpy.data.objects['Control6'].keyframe_insert(data_path='rotation_euler')
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    print("ok")
#机械臂移动
def robot_move_o(v):
    j1,j2,j3,j4,j5,j6=robot_get_jn()
    j1,j2,j3,j4,j5,j6=1.75,1.5,1.25,1,0.75,0.5
    for i in range(v):
        bpy.context.scene.frame_set(0)
        bpy.data.objects['Control1'].rotation_euler[1] -=j1/v
        bpy.data.objects['Control1'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control2'].rotation_euler[1] -=j2/v
        bpy.data.objects['Control2'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control3'].rotation_euler[1] -=j3/v
        bpy.data.objects['Control3'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control4'].rotation_euler[1] -=j4/v
        bpy.data.objects['Control4'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control5'].rotation_euler[1] -=j5/v
        bpy.data.objects['Control5'].keyframe_insert(data_path='rotation_euler')
        bpy.data.objects['Control6'].rotation_euler[1] -=j6/v
        bpy.data.objects['Control6'].keyframe_insert(data_path='rotation_euler')
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
    print("ok")

if __name__ == "__main__":
    
    robot_move(30)
    robot_move_o(100)