import bpy
import bmesh



#get world coordinate
def get_location_world(object):
    x=bpy.data.objects[object].matrix_world[0][3]
    y=bpy.data.objects[object].matrix_world[1][3]
    z=bpy.data.objects[object].matrix_world[2][3]
    return x,y,z


#复制物体
def clone(object,Vector):
    bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False, "mode":'TRANSLATION'}, TRANSFORM_OT_translate={"value":(Vector[0]+tx, ty-Vector[2],0), "orient_axis_ortho":'X', "orient_type":'GLOBAL', "orient_matrix":((1, 0, 0), (0, 1, 0), (0, 0, 1)), "orient_matrix_type":'GLOBAL', "constraint_axis":(False, False, False), "mirror":False, "use_proportional_edit":False, "proportional_edit_falloff":'SMOOTH', "proportional_size":1, "use_proportional_connected":False, "use_proportional_projected":False, "snap":False, "snap_elements":{'INCREMENT'}, "use_snap_project":False, "snap_target":'CLOSEST', "use_snap_self":False, "use_snap_edit":False, "use_snap_nonedit":False, "use_snap_selectable":False, "snap_point":(0, 0, 0), "snap_align":False, "snap_normal":(0, 0, 0), "gpencil_strokes":False, "cursor_transform":False, "texture_space":False, "remove_on_cancel":False, "view2d_edge_pan":False, "release_confirm":False, "use_accurate":False, "use_automerge_and_split":False})
#    bpy.data.objects[object].name="clone"
    



def choose_object(object):  
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active=None
    bpy.data.objects[object].select_set(True)
    bpy.context.view_layer.objects.active=bpy.data.objects[object]

Vector=[]
obj=bpy.context.object
if obj.mode == 'EDIT':
    bm=bmesh.from_edit_mesh(obj.data)
    for v in bm.verts:
        if v.select:
            for i in range (3):
                Vector.append(v.co[i])

xp,yp,zp=get_location_world("Chess_piece")
xb,yb,zb=get_location_world("Chessboard")
tx,ty=xb-xp,yb-yp

print(Vector)
bpy.ops.object.editmode_toggle()
choose_object("Chess_piece")
clone("Chess_piece",Vector)
choose_object("Chessboard")
bpy.ops.object.mode_set(mode='EDIT')
