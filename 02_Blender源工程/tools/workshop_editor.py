from pathlib import Path
import sys, json, math, copy, uuid, subprocess, shutil, hashlib
import bpy, bmesh
from mathutils import Matrix, Vector
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from workshop import OUT, ROOT, ROOM, COLORS, LABELS, initial_layout, fixed_room, prototype, surface, validate, transforms, copy_subtree, export_mjcf, package_botlab, write_json, check_room_bounds
MATERIALS = {}

def geometry_signature(o):
    return hashlib.sha256(json.dumps([[round(v, 7) for v in p.co] for p in o.data.vertices]).encode()).hexdigest()

def material(name):
    if name in MATERIALS and MATERIALS[name].name in bpy.data.materials:
        return MATERIALS[name]
    rgba = COLORS[name]
    m = bpy.data.materials.get('Workshop_' + name) or bpy.data.materials.new('Workshop_' + name)
    m.diffuse_color = rgba
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = rgba
    p.inputs['Roughness'].default_value = 0.45
    if name in ('bin', 'glass'):
        p.inputs['Transmission Weight'].default_value = 0.65
        p.inputs['Alpha'].default_value = rgba[3]
        p.inputs['Roughness'].default_value = 0.16
        m.surface_render_method = 'DITHERED'
    if name in ('metal', 'foil'):
        p.inputs['Metallic'].default_value = 0.55
    MATERIALS[name] = m
    return m

def make_geom(n):
    s = n['shape']
    sz = n['size']
    if s == 'box':
        bpy.ops.mesh.primitive_cube_add(size=1)
    elif s == 'cylinder':
        bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=0.5, depth=1)
    elif s == 'ellipsoid':
        bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=0.5)
    else:
        v, f = surface(n)
        me = bpy.data.meshes.new(n['id'])
        me.from_pydata(v, [], f)
        me.update()
        o = bpy.data.objects.new(n['id'], me)
        bpy.context.collection.objects.link(o)
        for p in me.polygons:
            p.use_smooth = True
        return o
    o = bpy.context.object
    o.dimensions = sz
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if s == 'box':
        be = o.modifiers.new('Manufactured edges', 'BEVEL')
        be.width = min(0.003, min(sz) * 0.16)
        be.segments = 3
        o.modifiers.new('Surface normals', 'WEIGHTED_NORMAL')
    else:
        for p in o.data.polygons:
            p.use_smooth = True
        be = o.modifiers.new('Rim rounding', 'BEVEL')
        be.width = min(0.001, min(sz) * 0.1)
        be.segments = 2
        o.modifiers.new('Surface normals', 'WEIGHTED_NORMAL')
    return o

def make_camera(name, pos, target, ortho=None, lens=45):
    bpy.ops.object.camera_add(location=pos)
    o = bpy.context.object
    o.name = name
    o.rotation_euler = (Vector(target) - o.location).to_track_quat('-Z', 'Y').to_euler()
    o.data.lens = lens
    if ortho:
        o.data.type = 'ORTHO'
        o.data.ortho_scale = ortho
    return o

def store_layout(layout):
    t = bpy.data.texts.get('Workshop_layout.json') or bpy.data.texts.new('Workshop_layout.json')
    t.clear()
    t.write(json.dumps(layout, ensure_ascii=False))

def scene_layout():
    t = bpy.data.texts.get('Workshop_layout.json')
    if not t:
        raise ValueError('当前文件不是药房工作坊工程')
    layout = json.loads(t.as_string())
    by = {n['id']: n for n in layout['nodes']}
    for id, n in by.items():
        o = bpy.data.objects.get(id)
        if o is None:
            raise ValueError('组件缺失，请使用“清空内部”或恢复参考布局：' + id)
        if any((abs(s - 1) > 1e-05 for s in o.scale)):
            raise ValueError('不支持缩放组件：' + id)
        if n['kind'] == 'geom' and geometry_signature(o) != o.get('ws_geometry_signature'):
            raise ValueError('网格已被直接修改，无法保持布局与仿真一致；请恢复该组件：' + id)
        e = o.rotation_euler
        if n.get('role') in ('door', 'flap'):
            axis = n.get('axis', 'z')
            n['angle'] = float(e.x if axis == 'x' else e.z)
            n['yaw'] = 0.0
            if abs(e.y) > 1e-05 or abs(e.z if axis == 'x' else e.x) > 1e-05:
                raise ValueError('请沿组件铰链轴旋转：' + id)
        else:
            if abs(e.x) > 1e-05 or abs(e.y) > 1e-05:
                raise ValueError('布局组件只支持水平旋转：' + id)
            n['yaw'] = float(e.z)
        n['pos'] = list(map(float, o.location))
    for n in layout['nodes']:
        if n['id'] == 'room' or n['id'].startswith('room_'):
            r = next((r for r in fixed_room() if r['id'] == n['id']))
            if max((abs(a - b) for a, b in zip(n['pos'], r['pos']))) < 1e-06 and abs(n['yaw'] - r['yaw']) < 1e-06:
                n.update(r)
    layout['zones'] = []
    layout['routes'] = []
    for o in bpy.context.scene.objects:
        if o.get('workshop_zone'):
            if abs(o.rotation_euler.x) > 1e-05 or abs(o.rotation_euler.y) > 1e-05:
                raise ValueError('分区边界应保持水平')
            layout['zones'].append({'id': o.name, 'pos': list(o.location), 'size': list(o.scale[:2]), 'yaw': float(o.rotation_euler.z)})
        if o.get('workshop_route'):
            layout['routes'].append({'id': o.name, 'points': [[float(v) for v in (o.matrix_world @ Vector(p.co[:3]))[:2]] for p in o.data.splines[0].points]})
    layout['aisle_width_m'] = bpy.context.scene.ws_aisle_width
    validate(layout)
    return layout

def populate(layout):
    validate(layout)
    for o in list(bpy.data.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    MATERIALS.clear()
    objects = {}
    interior = bpy.data.collections.get('室内模块') or bpy.data.collections.new('室内模块')
    shell = bpy.data.collections.get('固定房间') or bpy.data.collections.new('固定房间')
    if interior.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(interior)
    if shell.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(shell)
    for n in layout['nodes']:
        if n['kind'] == 'geom':
            o = make_geom(n)
            o.data.materials.append(material(n['material']))
        else:
            o = bpy.data.objects.new(n['id'], None)
            bpy.context.collection.objects.link(o)
            o.empty_display_type = 'PLAIN_AXES'
            o.empty_display_size = 0.09
        o.name = n['id']
        objects[n['id']] = o
        o['ws_id'] = n['id']
        o['label'] = n.get('label', n['id'])
        o.lock_scale = (True, True, True)
        if n['kind'] == 'geom':
            o['ws_geometry_signature'] = geometry_signature(o)
        if n['parent']:
            o.parent = objects[n['parent']]
        o.location = n['pos']
        o.rotation_euler = (n.get('angle', 0) if n.get('axis') == 'x' else 0, 0, n['yaw'] + (n.get('angle', 0) if n.get('axis', 'z') == 'z' else 0))
        for c in list(o.users_collection):
            c.objects.unlink(o)
        (shell if n['id'] == 'room' or n['id'].startswith('room_') else interior).objects.link(o)
        if n['id'] == 'room' or n['id'].startswith('room_'):
            o.lock_location = (True, True, True)
            o.lock_rotation = (True, True, True)
            o.hide_select = True
            if n.get('cutaway'):
                o.hide_render = True
                o.hide_set(True)
    if 'robot_station' in objects:
        reference = Matrix.Translation((2.85, 3.25, 0))
        snapshot = OUT / 'robot_geometry.json' if layout.get('robot_hand_profile') == 'Handle-L1-right-v5' else ROOT / 'scene/robot_geometry.json'
        if not snapshot.exists():
            snapshot = OUT / 'robot_geometry.json'
        for r in json.loads(snapshot.read_text()):
            me = bpy.data.meshes.new(r['name'])
            me.from_pydata(r['vertices'], [], r['faces'])
            me.update()
            bm = bmesh.new()
            bm.from_mesh(me)
            bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=1e-08)
            bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=1e-08)
            bmesh.ops.triangulate(bm, faces=list(bm.faces))
            bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.calc_area() < 1e-10], context='FACES')
            bmesh.ops.delete(bm, geom=[v for v in bm.verts if not v.link_faces], context='VERTS')
            bm.to_mesh(me)
            bm.free()
            me.update()
            o = bpy.data.objects.new(r['name'], me)
            interior.objects.link(o)
            o.parent = objects['robot_station']
            mw = Matrix(r['rotation']).to_4x4()
            mw.translation = Vector(r['pos'])
            o.matrix_local = reference.inverted() @ mw
            o['ws_robot_mesh'] = True
            o.hide_select = True
            o.data.materials.append(material('black' if r['name'].startswith('hand/') else 'metal'))
            for f in me.polygons:
                f.use_smooth = True
    cameras = [make_camera('Overview', (-5.5, -6.2, 7.5), (1.8, 3.2, 0.7), 9.3), make_camera('Entry', (0.3, -1.8, 2.3), (1.9, 4.3, 1.05), lens=30), make_camera('Medicine detail', (0.65, 1.65, 1.9), (1.62, 2.85, 0.81), ortho=1.05), make_camera('Cabinet detail', (0.4, 4.5, 2.1), (0.8, 6.0, 0.85), lens=35)]
    for name, pos, power, size in [('Ceiling', (1.8, 3.2, 5.2), 950, 5), ('Front', (-2, -1, 4), 700, 5), ('Back', (1.5, 6, 4), 350, 3)]:
        bpy.ops.object.light_add(type='AREA', location=pos)
        o = bpy.context.object
        o.name = name
        o.data.energy = power
        o.data.shape = 'DISK'
        o.data.size = size
        o.rotation_euler = (Vector((1.8, 3.2, 0)) - o.location).to_track_quat('-Z', 'Y').to_euler()
    s = bpy.context.scene
    s.unit_settings.system = 'METRIC'
    s.unit_settings.length_unit = 'METERS'
    s.world.color = (0.25, 0.25, 0.25)
    s.render.engine = 'CYCLES'
    s.cycles.samples = 24
    s.cycles.use_denoising = True
    s.render.resolution_x = 1280
    s.render.resolution_y = 1000
    s.render.resolution_percentage = 100
    s.view_settings.view_transform = 'AgX'
    s.view_settings.exposure = -0.65
    s.camera = cameras[0]
    s.ws_aisle_width = layout.get('aisle_width_m', 0.6)
    for z in layout.get('zones', []):
        add_zone(z['id'], z['pos'], z['size'], z.get('yaw', 0))
    for r in layout.get('routes', []):
        add_route(r['id'], r['points'])
    store_layout(layout)
    for area in bpy.context.screen.areas if bpy.context.screen else []:
        if area.type == 'VIEW_3D':
            area.spaces.active.region_3d.view_distance = 9
            area.spaces.active.region_3d.view_location = (1.8, 3.2, 0.8)
            area.spaces.active.shading.type = 'MATERIAL'
            area.spaces.active.show_region_ui = True
    return cameras

def add_zone(name='分区', pos=(1.8, 2, 0.012), size=(0.6, 0.7), yaw=0):
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions = '3D'
    cu.bevel_depth = 0.004
    sp = cu.splines.new('POLY')
    sp.points.add(3)
    for p, co in zip(sp.points, [(-1, -1, 0, 1), (1, -1, 0, 1), (1, 1, 0, 1), (-1, 1, 0, 1)]):
        p.co = co
    sp.use_cyclic_u = True
    o = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(o)
    o.location = pos
    o.scale = (size[0], size[1], 1)
    o.rotation_euler.z = yaw
    o['workshop_zone'] = True
    return o

def add_route(name='动线路径', points=None):
    points = points or [[1.7, 0.05], [1.7, 1.3], [0.4, 2.0], [0.4, 4.6]]
    cu = bpy.data.curves.new(name, 'CURVE')
    cu.dimensions = '3D'
    cu.bevel_depth = 0.012
    sp = cu.splines.new('POLY')
    sp.points.add(len(points) - 1)
    for p, xy in zip(sp.points, points):
        p.co = (*xy, 0.015, 1)
    o = bpy.data.objects.new(name, cu)
    bpy.context.collection.objects.link(o)
    o['workshop_route'] = True
    return o

def choose(o):
    bpy.ops.object.select_all(action='DESELECT')
    o.hide_set(False)
    o.select_set(True)
    bpy.context.view_layer.objects.active = o

def module_root(o):
    by = {n['id']: n for n in json.loads(bpy.data.texts['Workshop_layout.json'].as_string())['nodes']}
    while o:
        if by.get(o.get('ws_id', ''), {}).get('prototype') or o.name == 'room':
            return o
        o = o.parent
    return None

def route_report(layout):
    import numpy as np
    mats = transforms(layout)
    by = {n['id']: n for n in layout['nodes']}
    bounds = {}
    for n in layout['nodes']:
        if n['kind'] != 'geom' or n.get('collision') is False:
            continue
        p = n
        while p['parent']:
            p = by[p['parent']]
        if p.get('prototype') not in ('cabinet', 'table', 'return_table', 'chair', 'rack', 'dispatch_rack', 'robot_station'):
            continue
        q = np.array([[x, y, z, 1] for x in (-n['size'][0] / 2, n['size'][0] / 2) for y in (-n['size'][1] / 2, n['size'][1] / 2) for z in (-n['size'][2] / 2, n['size'][2] / 2)]) @ mats[n['id']].T
        xy = q[:, :2]
        lo = xy.min(0)
        hi = xy.max(0)
        if p['id'] in bounds:
            lo = np.minimum(lo, bounds[p['id']][0])
            hi = np.maximum(hi, bounds[p['id']][1])
        bounds[p['id']] = (lo, hi)

    def point_seg(p, a, b):
        v = b - a
        t = np.clip(np.dot(p - a, v) / max(np.dot(v, v), 1e-12), 0, 1)
        return float(np.linalg.norm(p - (a + t * v)))

    def seg_rect(a, b, lo, hi):
        tlo, thi = (0.0, 1.0)
        delta = b - a
        for i in range(2):
            if abs(delta[i]) < 1e-12:
                if a[i] < lo[i] or a[i] > hi[i]:
                    break
            else:
                t1, t2 = sorted(((lo[i] - a[i]) / delta[i], (hi[i] - a[i]) / delta[i]))
                tlo = max(tlo, t1)
                thi = min(thi, t2)
        else:
            if tlo <= thi:
                return 0.0
        corners = [np.array([x, y]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
        ds = [point_seg(p, a, b) for p in corners]
        for p in (a, b):
            ds.append(float(np.linalg.norm(p - np.clip(p, lo, hi))))
        return min(ds)
    rows = []
    width = layout.get('aisle_width_m', 0.6)
    for route in layout.get('routes', []):
        pts = [np.array(p) for p in route['points']]
        for k, (a, b) in enumerate(zip(pts, pts[1:])):
            distances = {id: seg_rect(a, b, *bb) for id, bb in bounds.items()}
            wall = min(a[0], b[0], ROOM[0] - a[0], ROOM[0] - b[0], a[1], b[1], ROOM[1] - a[1], ROOM[1] - b[1])
            if k == 0 and min(a[1], b[1]) < width / 2 and (1.2 + width / 2 <= a[0] <= 2.2 - width / 2):
                wall = min(a[0], b[0], ROOM[0] - a[0], ROOM[0] - b[0], ROOM[1] - a[1], ROOM[1] - b[1])
            near = min(distances, key=distances.get) if distances else 'room'
            clear = min(wall, distances.get(near, float('inf')))
            rows.append(dict(route=route['id'], segment=k + 1, available_width_m=float(max(0, 2 * clear)), required_width_m=float(width), blocked=bool(clear < width / 2), nearest=near))
    return dict(method='Conservative 2D assembly AABB vs path capsule; current door pose, not swept opening envelope. User-defined width, no reachability or regulatory claim.', segments=rows)

class WS_OT_action(bpy.types.Operator):
    bl_idname = 'workshop.action'
    bl_label = '药房操作'
    bl_options = {'REGISTER', 'UNDO'}
    action: bpy.props.StringProperty()

    def execute(self, context):
        try:
            a = self.action
            o = context.active_object
            if a == 'select':
                root = module_root(o)
                if not root or root.name == 'room':
                    raise ValueError('请选择室内组件')
                choose(root)
            elif a == 'component':
                while o and o.type == 'MESH':
                    o = o.parent
                if not o or o.name == 'room':
                    raise ValueError('请选择柜门、层板、瓶盖等组件')
                choose(o)
            elif a in ('reference', 'empty'):
                l = json.loads((OUT / 'layout_reference.json').read_text()) if (OUT / 'layout_reference.json').exists() else initial_layout()
                if a == 'empty':
                    l['nodes'] = fixed_room()
                    l['zones'] = []
                    l['routes'] = []
                populate(l)
            elif a == 'duplicate':
                l = scene_layout()
                root = module_root(o)
                if not root or root.name in ('room', 'robot_station'):
                    raise ValueError('只能复制家具与物品；机器人保持一套')
                id = root['ws_id']
                new = id + '_copy_' + uuid.uuid4().hex[:6]
                copy_subtree(l, id, new)
                populate(l)
                choose(bpy.data.objects[new])
            elif a == 'add':
                l = scene_layout()
                kind = context.scene.ws_module
                if kind == 'robot_station' and any((n.get('prototype') == 'robot_station' for n in l['nodes'])):
                    raise ValueError('当前控制工程只支持一套机器人')
                id = 'robot_station' if kind == 'robot_station' else kind + '_' + uuid.uuid4().hex[:6]
                l['nodes'].extend(prototype(kind, id, context.scene.cursor.location))
                populate(l)
                choose(bpy.data.objects[id])
            elif a == 'delete':
                l = scene_layout()
                if not o or not o.get('ws_id') or o.name == 'room' or o.name.startswith('room_'):
                    raise ValueError('请选择可移除的室内组件')
                ids = {o['ws_id']}
                changed = True
                while changed:
                    more = {n['id'] for n in l['nodes'] if n['parent'] in ids}
                    changed = not more.issubset(ids)
                    ids |= more
                l['nodes'] = [n for n in l['nodes'] if n['id'] not in ids]
                populate(l)
            elif a == 'walls':
                for n in fixed_room():
                    if n.get('cutaway'):
                        obj = bpy.data.objects[n['id']]
                        obj.hide_render = not obj.hide_render
                        obj.hide_set(obj.hide_render)
            elif a == 'zone':
                choose(add_zone('分区_' + uuid.uuid4().hex[:4], context.scene.cursor.location))
            elif a == 'route':
                choose(add_route('动线_' + uuid.uuid4().hex[:4]))
            elif a == 'check':
                l = scene_layout()
                r = route_report(l)
                write_json(OUT / 'edits/route_report.json', r)
                if not r['segments']:
                    raise ValueError('请先添加动线路径，在编辑模式拖动路径顶点')
                blocked = sum((x['blocked'] for x in r['segments']))
                self.report({'WARNING'} if blocked else {'INFO'}, f"{len(r['segments'])}段路径，{blocked}段不足设定宽度；详见route_report.json")
            elif a == 'save':
                l = scene_layout()
                check_room_bounds(l)
                ed = OUT / 'edits'
                write_json(ed / 'layout.json', l)
                interpreter = next((p / '.venv/bin/python' for p in (ROOT, *ROOT.parents) if (p / '.venv/bin/python').exists()), None)
                interpreter = str(interpreter) if interpreter else shutil.which('python3')
                if not interpreter:
                    raise RuntimeError('未找到Python；请先安装requirements.txt中的依赖')
                result = subprocess.run([interpreter, str(HERE / 'workshop.py'), '--layout', str(ed / 'layout.json'), '--dest', str(ed)], capture_output=True, text=True)
                if result.returncode:
                    raise RuntimeError('仿真导出失败：' + result.stderr[-800:])
                store_layout(l)
                bpy.ops.wm.save_as_mainfile(filepath=str(ed / 'pharmacy_edited.blend'))
                export_glb(ed / 'pharmacy_edited.glb')
                self.report({'INFO'}, '已保存布局、Blender、GLB和对应MJCF；含机器人时同时生成BotLab包')
            elif a == 'load':
                p = OUT / 'edits/layout.json'
                if not p.exists():
                    raise ValueError('尚无已保存的布局')
                populate(json.loads(p.read_text()))
            return {'FINISHED'}
        except Exception as exc:
            self.report({'ERROR'}, str(exc))
            print('Workshop action failed:', exc)
            return {'CANCELLED'}

class WS_PT_panel(bpy.types.Panel):
    bl_label = '药房布局工作坊'
    bl_idname = 'WS_PT_panel'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = '药房工作坊'

    def draw(self, context):
        l = self.layout
        l.label(text='固定房间 3.6 × 6.4 × 2.9 m', icon='LOCKED')

        def button(text, a, icon='NONE'):
            l.operator('workshop.action', text=text, icon=icon).action = a
        button('选中整个组合', 'select')
        button('选中层板／门／盖', 'component')
        button('复制选中组合', 'duplicate')
        button('移除选中组件', 'delete')
        o = context.active_object
        if o and o.get('ws_id') and (o.name != 'room') and (not o.name.startswith('room_')):
            l.label(text=o.get('label', o.name))
            l.prop(o, 'location', text='位置（米）')
            l.prop(o, 'rotation_euler', text='旋转')
        l.separator()
        l.prop(context.scene, 'ws_module', text='模块')
        button('添加到3D游标位置', 'add')
        button('添加分区边界', 'zone')
        button('添加动线路径', 'route')
        l.prop(context.scene, 'ws_aisle_width', text='路径宽度（米）')
        button('检查当前动线', 'check')
        l.label(text='路径：Tab编辑，拖动顶点；分区可缩放')
        button('显示／隐藏剖切墙面', 'walls')
        button('清空全部室内组件', 'empty')
        button('恢复照片参考布局', 'reference')
        l.separator()
        button('保存布局并导出仿真', 'save', 'FILE_TICK')
        button('读取已保存布局', 'load', 'FILE_FOLDER')

def register():
    for c in (WS_OT_action, WS_PT_panel):
        try:
            bpy.utils.register_class(c)
        except ValueError:
            pass
    bpy.types.Scene.ws_module = bpy.props.EnumProperty(name='模块', items=[(k, v, v) for k, v in LABELS.items()])
    bpy.types.Scene.ws_aisle_width = bpy.props.FloatProperty(name='路径宽度', default=0.6, min=0.2, max=2.0, precision=2)

def export_glb(path, selection=False):
    bpy.ops.export_scene.gltf(filepath=str(path), export_format='GLB', export_cameras=False, export_lights=False, use_selection=selection, export_extras=True, export_apply=True)

def build_blender(render=False):
    register()
    layout = json.loads((OUT / 'layout_reference.json').read_text())
    cams = populate(layout)
    OUT.mkdir(exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'pharmacy_workshop.blend'))
    export_glb(OUT / 'pharmacy_workshop.glb')
    modules = OUT / 'modules'
    modules.mkdir(exist_ok=True)
    for kind in LABELS:
        candidates = [n for n in layout['nodes'] if n.get('prototype') == kind]
        if not candidates:
            continue
        root = bpy.data.objects[candidates[0]['id']]
        bpy.ops.object.select_all(action='DESELECT')
        for o in [root] + list(root.children_recursive):
            o.hide_set(False)
            o.select_set(True)
        old = root.matrix_world.copy()
        root.matrix_world = Matrix.Identity(4)
        bpy.context.view_layer.update()
        export_glb(modules / (kind + '.glb'), True)
        root.matrix_world = old
    bpy.context.view_layer.update()
    if render:
        previews = OUT / 'previews'
        previews.mkdir(exist_ok=True)
        for c in (cams[0], cams[2], cams[3]):
            bpy.context.scene.camera = c
            bpy.context.scene.render.filepath = str(previews / (c.name + '.png'))
            bpy.ops.render.render(write_still=True)
    empty = copy.deepcopy(layout)
    empty['nodes'] = fixed_room()
    empty['zones'] = []
    empty['routes'] = []
    populate(empty)
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / 'pharmacy_empty.blend'))
    export_glb(OUT / 'pharmacy_empty.glb')
    if render:
        bpy.context.scene.render.filepath = str(OUT / 'previews/Empty room.png')
        bpy.ops.render.render(write_still=True)
    print('Workshop Blender assets generated:', OUT)
if __name__ == '__main__':
    register()
