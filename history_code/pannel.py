bl_info = {
    "name": "rpu_pu",
    "author": "zzk",
    "version": (0, 0, 1),
    "blender": (2, 80, 0),
    "location": "",
    "description": "",
    "category": "机械臂",
}

import bpy  
import blf  
import bmesh
import pynput
import time 
from Robot import Robot
import serial
from serial.tools import list_ports
import threading
import math
import cv2
import numpy as np
import varmscara_fk

def draw_callback_px(self, context):
    blf.position(0, 20, 20, 0)  # 字体位置
    blf.size(0, 20)         # 字体大小
    blf.color(0,1,1,0,1)        # 字体颜色
    blf.draw(0, "       按TAB退出")  # 在当前上下文中绘制文本

def choose_object(object):
    print("选中:",object)
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active=None
    bpy.data.objects[object].select_set(True)
    bpy.context.view_layer.objects.active=bpy.data.objects[object]

class uped_control(bpy.types.Operator):
    bl_idname="button.uped"
    bl_label="uped_control"  #别名

    # ---------------------------------------初始化-------------------------------------------------------
    
    _timer = None       # 用于承载计时器
    info = None         # 用于承载文字绘制
    w,a,s,d = [.0]*4    # 用于检测w,a,s,d按键状态
    
    # ---------------------------------------模态函数-------------------------------------------------------
    # 模态函数
    def modal(self, context, event):       
        
        # 按ESC键退出运算符
        if event.type =='ESC':
            self.cancel(context) # 运行"退出"函数
                        
            # 更新3d视图(如果有)
            for area in bpy.context.screen.areas:
                area.tag_redraw()
                                
            return {'CANCELLED'} # 退出模态运算符
        
        # 计时器事件(实时检测)
        if event.type == 'TIMER': # 如果是计时器事件，即操作物体
            choose_object("Dof1")
            bpy.ops.transform.translate(value = (0, self.w + self.s, 0), orient_axis_ortho='X', orient_type='LOCAL')
            bpy.ops.transform.rotate(value = self.a + self.d, orient_axis='Z', orient_type='GLOBAL')
            choose_object("Dof4")
            bpy.ops.transform.translate(value = (0, self.w + self.s, 0), orient_axis_ortho='X', orient_type='LOCAL')
            bpy.ops.transform.rotate(value = -self.a-self.d , orient_axis='Z', orient_type='GLOBAL')
            bpy.ops.object.select_all(action='DESELECT')
            bpy.context.view_layer.objects.active=None



        # w,a,s,d 按键状态检测
        if event.type == 'W':
            if event.value == 'PRESS':
                self.w = .1
            if event.value == 'RELEASE':
                self.w = .0
            return {'RUNNING_MODAL'}
            
        if event.type == 'A':
            if event.value == 'PRESS':
                self.a = -.1
            if event.value == 'RELEASE':
                self.a = .0
            return {'RUNNING_MODAL'}
        
        if event.type == 'S':
            if event.value == 'PRESS':
                self.s = -.1
            if event.value == 'RELEASE':
                self.s = 0
            return {'RUNNING_MODAL'}         
        
        if event.type == 'D':
            if event.value == 'PRESS':
                self.d = .1
            if event.value == 'RELEASE':
                self.d = .0
            return {'RUNNING_MODAL'}

        return {'PASS_THROUGH'} # 如果没有做事情，则绕过模态(这样在除了上面按键外，保留blender自身快捷键)
    
    # 当脚本运行时立即执行
    def execute(self, context):
        self.info = bpy.types.SpaceView3D.draw_handler_add(draw_callback_px, (None, None), 'WINDOW', 'POST_PIXEL') # 将文本绘制添加到窗口中
        self._timer = context.window_manager.event_timer_add(1/24, window=context.window) # 将计时器添加到窗口中
        context.window_manager.modal_handler_add(self) # 将模态处理函数添加到窗口中
        return {'RUNNING_MODAL'} # 返回到模态，保持运算符在blender上一直处于运行状态

    # ----------------------------------------------------------------------------------------------
    
    # "退出"函数(命令取消时)
    def cancel(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self.info,'WINDOW') # 将模态处理函数在窗口管理器中移除
        context.window_manager.event_timer_remove(self._timer) # 将给定的窗口中的计时器移除

# ui_type;固定各式
class ui_type:
    bl_space_type,bl_region_type,bl_context="VIEW_3D","UI","objectmode"     


# 面板2
class rpu_arms(ui_type,bpy.types.Panel):  
    bl_idname="rpu_arms"
    bl_label="机械臂控制"                                                    #  显示的标签名字
    bl_category="机械臂"                                                     #  标签分类
    
    def draw(self,context):
        layout=self.layout
        layout.label(text="status",icon="BLENDER")
        row3=layout.row()
        row3.operator("button.uped",text="上升",icon="CUBE")
        





classes=[
    rpu_arms,
    uped_control,
]


def register():
    for item in classes:
        bpy.utils.register_class(item)
    
def unregister():
    for item in classes:
        bpy.utils.unregister_class(item)


if __name__ == "__main__":
    register()