import bpy, blf
from bpy_extras import view3d_utils
from bpy.types import Menu

def draw_callback_px(self, context):
    
    """ 3D视口中绘制提示 """

    blf.size(0, 25)         # 字体大小
    blf.position(0, 20, 40, 0)  # 字体位置
    blf.color(0,1,1,0,1)        # 字体颜色
    blf.draw(0, f"功德：{self.virtues}")
    blf.size(0, 15)
    blf.position(0, 20, 15, 0)
    blf.draw(0, "点击木槌捡起，点击木鱼敲击，右键放下按木槌，Esc键退出")

def main(context, event, obj):
    """用鼠标左键运行此功能，执行光线投射"""
    # 获取上下文参数
    scene = context.scene # 当前场景
    region = context.region # 3D视口的区域
    rv3d = context.region_data # 区域数据
    coord = event.mouse_region_x, event.mouse_region_y # 鼠标x、y位置

    # 从视口和鼠标中获取光线
    view_vector = view3d_utils.region_2d_to_vector_3d(region, rv3d, coord) # 从特定二维区域坐标下返回视口的方向向量(规格化向量)。
    ray_origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, coord) # 从2D坐标返回3D视图原点位置。

    # 射线目标
    ray_target = ray_origin + view_vector

    # 光线投射函数
    def obj_ray_cast(obj, matrix):
        """用于将光线移动到对象空间的光线投射的函数"""

        # 获取相对于对象的光线
        matrix_inv = matrix.inverted() # 逆矩阵
        ray_origin_obj = matrix_inv @ ray_origin # 相对原点
        ray_target_obj = matrix_inv @ ray_target # 相对目标
        ray_direction_obj = ray_target_obj - ray_origin_obj # 相对朝向

        # 将光线投射到对象空间中已评估的几何体上，以返回：是否击中、击中的位置、法向和网格面编号
        success, location, normal, face_index = obj.ray_cast(ray_origin_obj, ray_direction_obj)
        
        # 如果击中,则传回:位置、法向和面编号；否则所有参数都传回 None(空的)。
        if success:
            return location, normal, face_index
        else:
            return None, None, None

    # 投射光线并找到最近的对象
    
    # 初始化最佳拾取深度,和对象
    best_length_squared = -1.0
    best_obj = None
    hit = (0,)*3
    
    # 对象的全局变换
    matrix = obj.matrix_world
    # 如果该对象是网格
    if obj.type == 'MESH':
        hit, normal, face_index = obj_ray_cast(obj, matrix) # 执行光线投射函数并取得位置、法向和面编号
        
        # 如果位置不为空
        if hit is not None:
            hit_world = matrix @ hit # 全局位置
            length_squared = (hit_world - ray_origin).length_squared # 原点位置到投射位置的距离,位置矢量自己点乘自己,得到位置矢量的长度的平方
            if best_obj is None or length_squared < best_length_squared:
                best_length_squared = length_squared
                best_obj = obj.original
                    
    return best_obj, ray_origin, view_vector, hit, normal

# 创建模态运算符
class ElectronicMokugyo(bpy.types.Operator):
    """Electronic mokugyo"""
    bl_idname = "view3d.modal_electronic_mokugyo"
    bl_label = "开始"

    _timer = None       # 用于承载计时器
    info = None         # 用于承载文字绘制
    pick_up = False
    beat_mo = False
    virtues = 0
    
    mokugyo = bpy.data.objects['木鱼']
    collide = bpy.data.objects['木鱼_凸壳碰撞体']
    hammer = bpy.data.objects['木槌']
    fixed_point = bpy.data.objects['敲击点']
    gesture = bpy.data.objects['手势定位']
    Speaker = bpy.data.objects['Speaker']
       
    def modal(self, context, event):
        
        obj = self.collide if self.pick_up else self.hammer
        best_obj, ray_origin, view_vector, hit, normal = main(context, event, obj)
        
        gesture_const = self.hammer.constraints[0]
        hammer_const = self.hammer.constraints[1]
        
        # 计时器事件
        if event.type == 'TIMER':
            
            # 当木槌处于“拿起”状态时
            if self.pick_up:
                # 更新手势位置
                self.gesture.location = ray_origin + view_vector * 8
                self.gesture.rotation_euler = ray_origin.to_track_quat('Z','Y').to_euler()
                
                # 将木槌逐渐约束到手势位置
                if gesture_const.influence < 1:
                    gesture_const.influence = min(gesture_const.influence + .1, 1)
                
                # 不选择木槌，为了避免亮框显示
                self.hammer.select_set(False)
                
                # 如果光标在木鱼上，光标射线击中木鱼的凸壳碰撞体
                if self.collide is best_obj:
                    
                    # 将敲击定位点和声音发生器附着到木鱼表面上（凸壳碰撞体上）
                    self.fixed_point.location =  self.mokugyo.matrix_world @ hit
                    self.fixed_point.rotation_euler = normal.to_track_quat('Z','Y').to_euler()
                    self.Speaker.location =  self.mokugyo.matrix_world @ hit
                    self.Speaker.rotation_euler = normal.to_track_quat('-Z','Y').to_euler()
                    
                    # 将木槌逐渐约束到敲击点位置
                    if hammer_const.influence < 1:
                        hammer_const.influence = min(hammer_const.influence + .1, 1)
                        
                # 将木槌在敲击点位置的约束中逐渐解除
                else:
                    if hammer_const.influence > 0:
                        hammer_const.influence = max(hammer_const.influence - .1, 0)
                        
            # 当木槌处于“放下”状态时
            else:
                # 如果光标在木槌上,则将木槌设置为选择，为了突显亮框
                if self.hammer is best_obj:
                    self.hammer.select_set(True)
                else:
                    self.hammer.select_set(False)
                
                # 如果具有约束，则逐渐解除约束
                if hammer_const.influence > 0:
                    hammer_const.influence = max(hammer_const.influence - .1, 0)
                if gesture_const.influence > 0:
                    gesture_const.influence = max(gesture_const.influence - .1, 0)
                    
            # 当处于“敲击”状态时
            if self.beat_mo:
                beat_Action = bpy.data.objects["槌位点"].constraints["Action"]
                # 如果动画未播放或未完成播放，则继续播放动画
                if beat_Action.eval_time < 1:
                    beat_Action.eval_time = min(beat_Action.eval_time + .1, 1)
                    if beat_Action.eval_time == 0.5:
                        bpy.ops.screen.animation_cancel()
                        bpy.ops.screen.animation_play()
                        self.virtues += 1
                        
                # 如果动画已播完,回到未播放状态,并关闭“敲击”的状态
                else:
                    beat_Action.eval_time = 0
                    self.beat_mo = False
                    
            # 当声音已播放完成,停止动画并回到第0帧
            if context.scene.frame_current > 10:
                bpy.ops.screen.animation_cancel()
                bpy.context.scene.frame_set(0)            
        
        
        # 鼠标键事件
        if event.type in {'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}:
            # 如果是鼠标键事件，则绕过模态以允许操作视图
            return {'PASS_THROUGH'}
        
        # 左键事件
        elif event.type == 'LEFTMOUSE':
            # 如果是木槌，开启“拿起”状态
            if self.hammer is best_obj:
                self.pick_up = True
                
            # 如果是木鱼，开启“敲击”状态
            if self.pick_up:
                if self.collide is best_obj:
                    self.beat_mo = True
 
            return {'RUNNING_MODAL'}
        
        # 右键事件
        elif event.type == 'RIGHTMOUSE':
            # 设置为“放下”状态
            self.pick_up = False 

        # ESC键事件
        elif event.type == 'ESC':
            # 移除约束
            hammer_const.influence = 0
            gesture_const.influence = 0
            
            self.cancel(context) # 运行"退出"函数
            
            return {'CANCELLED'} # 返回'CANCELLED',以退出模态运算符

        return {'RUNNING_MODAL'} # 返回'RUNNING_MODAL',以继续模态运算符

    def invoke(self, context, event):
        
        if context.space_data.type == 'VIEW_3D':
            self.info = bpy.types.SpaceView3D.draw_handler_add(draw_callback_px, (self, context), 'WINDOW', 'POST_PIXEL') # 将文本绘制添加到窗口中
            self._timer = context.window_manager.event_timer_add(1/24, window=context.window) # 将计时器添加到窗口中
            context.window_manager.modal_handler_add(self) # 将模态处理函数添加到窗口中
            return {'RUNNING_MODAL'} # 返回到模态，保持运算符在blender上一直处于运行状态
        else:
            self.report({'WARNING'}, "活动空间必须是View3d")
            return {'CANCELLED'}
        
    # "退出"函数(命令取消时)
    def cancel(self, context):
        bpy.types.SpaceView3D.draw_handler_remove(self.info,'WINDOW') # 将模态处理函数在窗口管理器中移除
        context.window_manager.event_timer_remove(self._timer) # 将给定的窗口中的计时器移除


# 该类的名称必须与此完全相同，才能在“视图”菜单中插入条目
class VIEW3D_MT_object(Menu):
    bl_label = "电子木鱼"

    def draw(self, context):
        pass

# 添加一个菜单项
def menu_func(self, context):
    layout = self.layout
    layout.separator()
    layout.operator(ElectronicMokugyo.bl_idname)


# 注册并添加到“视图”菜单（也可以使用F3搜索“Raycast view Modal Operator”进行快速访问）。

classes = (
    ElectronicMokugyo,
    VIEW3D_MT_object,
)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.types.VIEW3D_MT_object.append(menu_func)

        
def unregister():    
    bpy.types.VIEW3D_MT_object.remove(menu_func)
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)

if __name__ == "__main__":

    register()


