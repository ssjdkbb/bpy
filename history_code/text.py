import bpy, blf
from bpy_extras.view3d_utils import location_3d_to_region_2d
from mathutils import Vector


#_____________________初始化时间和按钮状态____________________________

time_initial = False
testing_click = False

#_____________________创建一个移除功能按钮______________________

# 创建一个命令以切换移除状态
class remove_OP(bpy.types.Operator):
    bl_idname="obj.remove_operator"
    bl_label="切换移除状态"
    
    def execute(self,context):
        global testing_click
        testing_click = True  # 当执行按钮命令时，将"移除"的状态切换为：是     
        return{'FINISHED'}

# 绘制一个按钮,点击按钮将执行"切换移除状态"命令
def TEST_HT_view3d(self, context):
    self.layout.operator(
        operator=remove_OP.bl_idname,
        icon='CANCEL',
        text='关闭顶点编号'
    )

#__________________创建计时器以检测用户是否移除功能________________

def run_times():
    global time_initial
        
    # 第一次计时时启用功能
    if time_initial == False:
        bpy.utils.register_class(remove_OP)
        bpy.types.VIEW3D_HT_header.prepend(TEST_HT_view3d)
        bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)
        time_initial = True
        
    # 当检测到"移除"状态被切换为“是”的时候，移除功能，并停止计时器
    elif testing_click == True:
        bpy.types.VIEW3D_HT_header.remove(TEST_HT_view3d)
        bpy.utils.unregister_class(remove_OP)
        bpy.types.SpaceView3D.draw_handler_remove(hide_index,'WINDOW')
        return None # 返回 None 给计时器，计时器会停止
        
    return 0.1

#_____________________________绘制索引函数_____________________________
def pos_to_text(context, pos, color,index):
  pos = bpy.context.object.matrix_world@pos  #--转换为全局位置。
  pos_text = location_3d_to_region_2d(context.region, context.region_data, pos)  #--返回相对3D位置的2D区域位置。

  blf.position(0, pos_text[0], pos_text[1], 0) #--位置
  blf.size(0, 32, 32) #--大小 
  blf.color(0, *color) #--颜色
  blf.draw(0, str(index)) #--数值转字符串

#_______________________________绘制文字（批量）_________________________
def draw_callback_px(context, dummy):
    if bpy.context.object is None:
        return
    ob = bpy.context.object
    depsgraph = bpy.context.evaluated_depsgraph_get()
    cube_eval = ob.evaluated_get(depsgraph)
    
    for v in cube_eval.data.vertices:
        pos_to_text(context, v.co, [1, 1, 0, 0.3], v.index)

#________________________添加绘制以及移除的即时判定______________________

bpy.app.timers.register(run_times)   
hide_index = bpy.types.SpaceView3D.draw_handler_add(draw_callback_px, (bpy.context, None), 'WINDOW', 'POST_PIXEL')

