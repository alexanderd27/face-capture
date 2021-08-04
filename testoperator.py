import bpy


class TestProperties(bpy.types.PropertyGroup):
    bpy.types.Scene.starting_pose = bpy.props.EnumProperty(
        name = "Starting Pose",
        default = 'none',
        items = [('stand', 'Stand', ''),
                 ('sit', 'Sit', ''),
                 ('none', 'None', '')]
    )
    bpy.types.Scene.recorded_arm = bpy.props.EnumProperty(
        name = "Choose Arm",
        default = 'both',
        items = [('right', 'Record Right Arm only', ''),
                 ('left', 'Record Left Arm Only', ''),
                 ('both', 'Record Both Arms', '')]
    )
    bpy.types.Scene.FPS = bpy.props.IntProperty(
        name = "FPS",
        default = 1,
        soft_min = 1,
        soft_max = 50
    )


class TestButtonOperator(bpy.types.Operator):
    bl_idname = "scene.testbutton"
    bl_label = "Test Button"

    def execute(self, context):
        if context.scene.recorded_arm == 'both':
            print('yes')
        if context.scene.starting_pose == 'none':
            print('yes')
        if context.scene.FPS == 1:
            print('yes')
        return {'FINISHED'}


class TestPanel(bpy.types.Panel):
    bl_label = "Test"
    bl_idname = "scene.test_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'

    @classmethod
    def poll(self, context):    
        return True

    def draw(self, context):
        layout = self.layout
        layout.prop(bpy.context.scene, "FPS")
        layout.prop(bpy.context.scene, "starting_pose")
        layout.prop(bpy.context.scene, "recorded_arm")
        layout.operator(TestButtonOperator.bl_idname)


bl_info = {"name": "MyTest", "category": "User"}

def register():
    bpy.utils.register_class(TestButtonOperator)
    bpy.utils.register_class(TestPanel)
    bpy.utils.register_class(TestProperties)

def unregister():
    bpy.utils.unregister_class(TestButtonOperator)
    bpy.utils.unregister_class(TestPanel)
    bpy.utils.unregister_class(TestProperties)

if __name__ == "__main__":
    register()

