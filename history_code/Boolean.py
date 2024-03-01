import bpy


#激活物体
def choose_object(object):
    bpy.context.view_layer.objects.active=bpy.data.objects[object]

def boo(object,object2):
    choose_object(object)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    bpy.context.object.modifiers["布尔"].operation = 'INTERSECT'
    bpy.data.objects[object].modifiers["布尔"].object=bpy.data.objects[object2]
    bpy.ops.object.select_all(action='DESELECT')
    x,y,z=bpy.data.objects[object].dimensions
    print("x:",x)
    print("y:",y)
    print("z:",z)
    bpy.data.materials["Material"].node_tree.nodes["Principled BSDF"].inputs[0].default_value = (0.8, 0.0211218, 0.0184601, 1)
    bpy.data.objects[object2].hide_viewport = True
#    bpy.ops.object.modifier_remove(modifier="布尔")
#    bpy.ops.object.modifier_apply(modifier="布尔", report=True)
#    bpy.data.objects.remove(bpy.data.objects["ball"])




if __name__ == "__main__":
    boo("B","b")
