import subprocess

script_path = "D:/7.intelligent/6.C/1.blender/ZDFX0808/bpy_ethercat.py"
# 调用blender
process = subprocess.Popen(['D:/Program Files/Blender 3.6/blender.exe',"D:/7.intelligent/6.C/1.blender/FX.blend","-P", script_path])
# stdout, stderr = process.communicate()

