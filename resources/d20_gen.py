import bpy
import bmesh

phi = (1 + 5**0.5)/2
vert_base = (0, 1.0, phi)
vertices = []

for s1 in [-1, 1]:
    vertex = (vert_base[0], s1*vert_base[1], vert_base[2])
    for s2 in [-1, 1]:
        vertex = (vertex[0], vertex[1], s2*vert_base[2])
        for i in range(3):
            vertices.append((vertex[-i], vertex[1-i], vertex[2-i]))
            print((vertex[-i], vertex[1-i], vertex[2-i]))
mesh = bpy.data.meshes.new("mesh")
obj = bpy.data.objects.new("D20", mesh)

scene = bpy.context.scene
scene.objects.link(obj)
scene.objects.active = obj
obj.select = True

mesh = bpy.context.object.data
bm = bmesh.new()

for v in vertices:
    bm.verts.new(v)
    
bm.to_mesh(mesh)
bm.free()