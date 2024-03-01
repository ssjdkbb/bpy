import bpy
from bpy_extras.view3d_utils import location_3d_to_region_2d

def view3d_find():
    # returns first 3d view, normally we get from context
    for area in bpy.context.window.screen.areas:
        if area.type == 'VIEW_3D':
            v3d = area.spaces[0]
            rv3d = v3d.region_3d
            for region in area.regions:
                if region.type == 'WINDOW':
                    return region, rv3d
    return None, None

def view3d_camera_border(scene):
    obj = scene.camera
    cam = obj.data

    frame = cam.view_frame(scene)

    # move into object space
    frame = [obj.matrix_world * v for v in frame]

    # move into pixelspace

    region, rv3d = view3d_find()
    frame_px = [location_3d_to_region_2d(region, rv3d, v) for v in frame]
    return frame_px

class ModalTimerOperator(bpy.types.Operator):
    """Operator which runs its self from a timer"""
    bl_idname = "wm.modal_timer_operator"
    bl_label = "ROBOTCONTROL"

    _timer = None

    def modal(self, context, event):
        scene = context.scene
        obj = context.object
        region = context.area.regions[-1]
        print("modal")

        if event.type in {'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type in {'RIGHTMOUSE'}:
            print("modal")
            if obj.mode == 'EDIT' and obj.type == 'MESH':
                camframe = view3d_camera_border(scene)

                #print([xy for xy in camframe])

                bpy.ops.view3d.select_border(gesture_mode=3, xmin=camframe[2].x, xmax=camframe[0].x,
                                             ymin=camframe[1].y, ymax=camframe[0].y)

        return {'PASS_THROUGH'}

    def execute(self, context):
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.1, context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)

def register():
    bpy.utils.register_class(ModalTimerOperator)

def unregister():
    bpy.utils.unregister_class(ModalTimerOperator)


if __name__ == "__main__":
    register()