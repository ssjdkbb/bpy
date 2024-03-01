
import bpy
import blf

""" 利用模态运算符在3D视口中进行按键交互 """

# --------------------------------------绘制文字到3D视口--------------------------------------------------------

def draw_callback_px(self, context):
    
    """ 3D视口中绘制提示 """
    
    blf.position(0, 20, 20, 0)  # 字体位置
    blf.size(0, 20)         # 字体大小
    blf.color(0,1,1,0,1)        # 字体颜色
    blf.draw(0, "按 W、A、S、D 移动，按Esc退出")  # 在当前上下文中绘制文本


# ----------------------------------------创建模态运算符------------------------------------------------------


class ModalOperator(bpy.types.Operator):
    
    """ 用键盘移动对象，模态运算符 """
    
    bl_idname = "object.modal_operator"
    bl_label = "Simple Modal Operator" 

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
            bpy.ops.transform.translate(value = (0, self.w + self.s, 0), orient_axis_ortho='X', orient_type='LOCAL')
            bpy.ops.transform.rotate(value = self.a + self.d, orient_axis='Z', orient_type='GLOBAL')


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

    # ----------------------------------------------------------------------------------------------

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
        

# ----------------------------------------------------------------------------------------------

# 直接注册运算符
bpy.utils.register_class(ModalOperator)

# 直接运行运算符
bpy.ops.object.modal_operator('INVOKE_DEFAULT')

