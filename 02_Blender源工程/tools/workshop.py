from pathlib import Path
import copy, json, math, shutil, xml.etree.ElementTree as ET, zipfile
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT if (ROOT / 'templates/pharmacy_v1.xml').exists() else ROOT / 'workshop_v2_2026-11-17'
ROOM = (3.6, 6.4, 2.9)
COLORS = {'metal': [0.66, 0.7, 0.67, 1], 'wood': [0.63, 0.42, 0.2, 1], 'black': [0.065, 0.075, 0.085, 1], 'glass': [0.39, 0.59, 0.61, 0.24], 'bin': [0.62, 0.8, 0.82, 0.42], 'white': [0.93, 0.93, 0.88, 1], 'blue': [0.055, 0.32, 0.6, 1], 'yellow': [0.93, 0.68, 0.1, 1], 'red': [0.38, 0.065, 0.08, 1], 'cardboard': [0.56, 0.37, 0.18, 1], 'foil': [0.64, 0.68, 0.73, 1], 'amber': [0.23, 0.1, 0.025, 1], 'wall': [0.86, 0.85, 0.79, 1], 'floor': [0.61, 0.53, 0.41, 1]}

def node(id, parent=None, pos=(0, 0, 0), yaw=0, kind='assembly', **kw):
    return dict(id=id, parent=parent, pos=list(pos), yaw=yaw, kind=kind, **kw)

def geom(id, parent, pos, size, material='metal', shape='box', **kw):
    return node(id, parent, pos, kind='geom', shape=shape, size=list(size), material=material, **kw)

def fixed_room():
    a = [node('room', kind='room', label='固定房间', locked=True)]
    for name, pos, size, mat in [('floor', (1.8, 3.2, -0.04), (3.6, 6.4, 0.08), 'floor'), ('back', (1.8, 6.44, 1.45), (3.6, 0.08, 2.9), 'wall'), ('right', (3.64, 3.2, 1.45), (0.08, 6.4, 2.9), 'wall'), ('left', (-0.04, 3.2, 1.45), (0.08, 6.4, 2.9), 'wall'), ('front_left', (0.6, -0.04, 1.45), (1.2, 0.08, 2.9), 'wall'), ('front_right', (2.9, -0.04, 1.45), (1.4, 0.08, 2.9), 'wall'), ('entry_header', (1.7, -0.04, 2.65), (1, 0.08, 0.5), 'wall')]:
        a.append(geom('room_' + name, 'room', pos, size, mat, cutaway=name in ('left', 'front_left', 'front_right', 'entry_header')))
    return a

def prototype(kind, id, pos=(0, 0, 0), yaw=0, parent=None):
    a = [node(id, parent, pos, yaw, prototype=kind, label=LABELS.get(kind, kind))]

    def g(s, p, sz, mat='metal', shape='box', owner=None, **kw):
        a.append(geom(id + '__' + s, owner or id, p, sz, mat, shape, **kw))
        return a[-1]

    def n(s, p=(0, 0, 0), owner=None, **kw):
        x = id + '__' + s
        a.append(node(x, owner or id, p, **kw))
        return x
    if kind == 'cabinet':
        w, d, h = (0.86, 0.4, 1.9)
        g('back', (0, 0.19, 0.95), (0.86, 0.02, 1.9))
        for side in (-1, 1):
            g('side' + str(side), (side * 0.415, 0, 0.95), (0.03, 0.4, 1.9))
        for z in (0.025, 1.885):
            g('cap' + str(z), (0, 0, z), (0.86, 0.4, 0.03))
        for k in range(6):
            s = n('shelf' + str(k), (0, 0, 0.08 + k * 0.29), role='shelf', label='层板 ' + str(k + 1))
            g('shelfplate' + str(k), (0, 0, 0), (0.8, 0.37, 0.025), owner=s)
            for j in (-1, 1):
                a.extend(prototype('bin', id + f'__bin{k}_{j}', (j * 0.205, 0, 0.014), parent=s))
        for side in (-1, 1):
            door = n('door' + str(side), (side * 0.425, -0.218, 0.02), role='door', label='左柜门' if side < 0 else '右柜门', hinge=True)
            dx = -side * 0.2125
            g('glass' + str(side), (dx, 0, 0.93), (0.375, 0.009, 1.75), 'glass', owner=door)
            for x in (0, -side * 0.425):
                g('doorrail' + str(side) + str(x), (x, 0, 0.93), (0.025, 0.025, 1.86), owner=door)
            for z in (0.015, 1.845):
                g('doorcross' + str(side) + str(z), (dx, 0, z), (0.425, 0.025, 0.03), owner=door)
            g('handle' + str(side), (-side * 0.37, -0.025, 0.9), (0.018, 0.026, 0.13), 'black', owner=door)
            for z in (0.18, 1.66):
                g('hinge' + str(side) + str(z), (0, 0.008, z), (0.025, 0.025, 0.07), 'metal', shape='cylinder', owner=door)
    elif kind in ('table', 'return_table'):
        w, d = (1.46, 1.6) if kind == 'table' else (1.42, 0.55)
        g('top', (0, 0, 0.742), (w, d, 0.036), 'wood')
        for x in (-1, 1):
            for y in (-1, 1):
                g(f'leg{x}_{y}', (x * (w / 2 - 0.055), y * (d / 2 - 0.06), 0.362), (0.035, 0.035, 0.724), 'black')
        for x in (-1, 1):
            g('frame' + str(x), (x * (w / 2 - 0.04), 0, 0.63), (0.025, d - 0.1, 0.04), 'black')
    elif kind in ('rack', 'dispatch_rack'):
        w, d = (0.9, 0.38) if kind == 'rack' else (1.1, 0.4)
        for x in (-1, 1):
            for y in (-1, 1):
                g(f'post{x}_{y}', (x * (w / 2 - 0.02), y * (d / 2 - 0.02), 1.02), (0.035, 0.035, 2.04), 'black')
        for k in range(5):
            s = n('shelf' + str(k), (0, 0, 0.16 + k * 0.38), role='shelf', label='货架层板 ' + str(k + 1))
            g('rackplate' + str(k), (0, 0, 0), (w, d, 0.035), 'black', owner=s)
    elif kind == 'chair':
        g('seat', (0, 0, 0.44), (0.38, 0.38, 0.06), 'red')
        g('back', (0, 0.18, 0.69), (0.38, 0.045, 0.44), 'red')
        for x in (-1, 1):
            for y in (-1, 1):
                g(f'leg{x}{y}', (x * 0.14, y * 0.14, 0.21), (0.025, 0.025, 0.42), 'black')
    elif kind == 'bin':
        w, d, h = (0.35, 0.3, 0.17)
        t = 0.005
        g('base', (0, 0, t / 2), (w, d, t), 'bin')
        for s in (-1, 1):
            g('side' + str(s), (s * w / 2, 0, h / 2), (t, d, h), 'bin')
            g('end' + str(s), (0, s * d / 2, h / 2), (w, t, h), 'bin')
            g('rimx' + str(s), (0, s * d / 2, h), (w + 0.012, 0.012, 0.012), 'bin')
            g('rimy' + str(s), (s * w / 2, 0, h), (0.012, d, 0.012), 'bin')
        g('label', (0, -d / 2 - 0.003, h * 0.52), (0.09, 0.002, 0.035), 'white', collision=False)
    elif kind == 'bottle':
        g('base', (0, 0, 0.003), (0.055, 0.055, 0.006), 'amber', 'cylinder')
        g('body', (0, 0, 0), (0.055, 0.049, 0.084), 'amber', 'tube', radii=[0.0275, 0.0245], height=0.084)
        g('shoulder', (0, 0, 0), (0.055, 0.035, 0.016), 'amber', 'profile', profile=[[0.084, 0.0275], [0.1, 0.0175], [0.1, 0.015], [0.083, 0.0245]])
        g('neck', (0, 0, 0.1), (0.035, 0.03, 0.017), 'amber', 'tube', radii=[0.0175, 0.015], height=0.017)
        g('label', (0, 0, 0.024), (0.056, 0.055, 0.043), 'white', 'tube', radii=[0.028, 0.0276], height=0.043, collision=False)
        cap = n('cap', (0, 0, 0.116), role='cap', label='瓶盖')
        g('cap_top', (0, 0, 0.018), (0.041, 0.041, 0.004), 'blue', 'cylinder', owner=cap)
        g('cap_skirt', (0, 0, 0), (0.041, 0.036, 0.018), 'blue', 'tube', radii=[0.0205, 0.018], height=0.018, owner=cap)
        for k in range(24):
            theta = k * math.tau / 24
            z = g('cap_rib' + str(k), (0.0205 * math.cos(theta), 0.0205 * math.sin(theta), 0.01), (0.0015, 0.0015, 0.015), 'blue', owner=cap)
    elif kind == 'carton':
        w, d, h = (0.07, 0.1, 0.09)
        t = 0.0015
        g('base', (0, 0, 0.001), (w, d, t), 'white')
        for s in (-1, 1):
            g('side' + str(s), (s * (w / 2 - t / 2), 0, h / 2), (t, d, h), 'white')
            g('end' + str(s), (0, s * (d / 2 - t / 2), h / 2), (w, t, h), 'white')
        g('stripe', (0, -d / 2 - 0.001, 0.045), (w, 0.001, 0.026), 'blue', collision=False)
        flap = n('flap', (0, d / 2, h), role='flap', label='盒盖', axis='x')
        g('lid', (0, -d / 2, 0), (w, d, t), 'white', owner=flap)
        g('tab', (0, -d - 0.009, 0), (0.04, 0.018, t), 'white', owner=flap)
    elif kind == 'blister':
        g('foil', (0, 0, 0.001), (0.065, 0.105, 0.002), 'foil')
        for i in range(2):
            for j in range(3):
                x = (i - 0.5) * 0.029
                y = (j - 1) * 0.031
                p = n(f'pill{i}{j}', (x, y, 0.005), role='pill', label='药片')
                g(f'tablet{i}{j}', (0, 0, 0), (0.013, 0.013, 0.005), 'white', 'ellipsoid', owner=p)
                g(f'dome{i}{j}', (x, y, 0.005), (0.021, 0.021, 0.014), 'bin', 'ellipsoid', collision=False)
    elif kind == 'cup':
        g('base', (0, 0, 0.0025), (0.045, 0.045, 0.005), 'bin', 'cylinder')
        g('wall', (0, 0, 0), (0.06, 0.045, 0.048), 'bin', 'profile', profile=[[0.004, 0.0225], [0.048, 0.03], [0.048, 0.028], [0.004, 0.0205]])
    elif kind == 'robot_station':
        g('pedestal', (0, 0, 0.44), (0.18, 0.18, 0.88), 'black')
        g('mount', (0, 0, 0.89), (0.22, 0.22, 0.02))
    elif kind == 'archive_box':
        g('body', (0, 0, 0.16), (0.6, 0.34, 0.32), 'cardboard')
        g('lid', (0, 0, 0.327), (0.62, 0.36, 0.025), 'cardboard')
        g('label', (0, -0.172, 0.14), (0.17, 0.002, 0.11), 'white', collision=False)
    else:
        raise ValueError('Unknown module: ' + kind)
    return a
LABELS = {'cabinet': '玻璃药柜', 'table': '配药工作台', 'return_table': '周转箱工作台', 'rack': '开放货架', 'dispatch_rack': '待取药货架', 'chair': '座椅', 'bin': '周转箱', 'bottle': '药瓶', 'carton': '药盒', 'blister': '泡罩药板', 'cup': '分药杯', 'robot_station': '机器人工作站', 'archive_box': '档案箱'}

def initial_layout():
    a = fixed_room()
    for i in range(3):
        a.extend(prototype('cabinet', f'back_cabinet_{i}', (0.48 + i * 0.88, 6.12, 0)))
        a.extend(prototype('cabinet', f'right_cabinet_{i}', (3.37, 5.18 - i * 0.88, 0), -math.pi / 2))
    for i in range(2):
        a.extend(prototype('table', f'central_table_{i}', (1.65, 2.65 + i * 1.62, 0)))
    for i in range(2):
        for j in (-1, 1):
            a.extend(prototype('chair', f'chair_{i}_{j}', (1.65 + j * 0.95, 2.25 + i * 1.3, 0), -j * math.pi / 2))
    a.extend(prototype('rack', 'side_storage_rack', (3.27, 1.95, 0), math.pi / 2))
    a.extend(prototype('return_table', 'return_table', (0.85, 0.55, 0)))
    a.extend(prototype('dispatch_rack', 'dispatch_rack', (2.8, 0.48, 0)))
    for i in range(3):
        for k in range(3):
            a.extend(prototype('bin', f'return_bin_{i}_{k}', (0.4 + i * 0.44, 0.55, 0.762 + k * 0.18)))
    for i in range(4):
        a.extend(prototype('bin', f'table_bin_{i}', (-0.35 + i % 2 * 0.64, -0.5 + i // 2 * 0.45, 0.762), parent='central_table_1'))
    for i in range(3):
        a.extend(prototype('archive_box', f'archive_{i}', (0, 0, 1.91), parent=f'back_cabinet_{i}'))
    a.extend(prototype('robot_station', 'robot_station', (2.85, 3.25, 0)))
    a.extend(prototype('carton', 'test_medicine_box', (0.5, 0.45, 0.762), parent='central_table_0'))
    for i in range(3):
        a.extend(prototype('bottle', f'medicine_bottle_{i}', (-0.33 + i * 0.15, 0.34, 0.762), parent='central_table_0'))
    a.extend(prototype('blister', 'blister_0', (0.06, -0.02, 0.762), parent='central_table_0'))
    for i in range(3):
        a.extend(prototype('cup', f'dose_cup_{i}', (-0.3 + i * 0.15, -0.12, 0.762), parent='central_table_0'))
    return dict(version=2, room_size_m=list(ROOM), nodes=a, zones=[], routes=[], aisle_width_m=0.6, description='Fixed design boundary; dimensions are project assumptions, not site survey.')

def validate(layout):
    if tuple(layout.get('room_size_m', ())) != ROOM:
        raise ValueError('房间尺寸固定为3.6×6.4×2.9m，不能修改。')
    nodes = layout['nodes']
    ids = [n['id'] for n in nodes]
    if len(ids) != len(set(ids)):
        raise ValueError('Duplicate semantic node IDs')
    by = {n['id']: n for n in nodes}
    for r in fixed_room():
        if by.get(r['id']) != r:
            raise ValueError('固定房间外壳被修改：' + r['id'])
    for n in nodes:
        if n['parent'] is not None and n['parent'] not in by:
            raise ValueError('Missing parent: ' + n['id'])
        if not np.isfinite(n['pos']).all() or not math.isfinite(n['yaw']):
            raise ValueError('Invalid transform')
        if n.get('scale', [1, 1, 1]) != [1, 1, 1]:
            raise ValueError('模块只支持移动旋转，不能缩放：' + n['id'])
        seen = set()
        p = n
        while p['parent']:
            if p['id'] in seen:
                raise ValueError('Hierarchy cycle')
            seen.add(p['id'])
            p = by[p['parent']]
    return by

def transforms(layout):
    by = validate(layout)
    out = {}

    def world(id):
        if id in out:
            return out[id]
        n = by[id]
        a = n.get('angle', 0)
        axis = n.get('axis', 'z')
        c, s = (math.cos(a), math.sin(a))
        r = np.eye(4)
        r[:3, 3] = n['pos']
        c0, s0 = (math.cos(n['yaw']), math.sin(n['yaw']))
        r[:3, :3] = [[c0, -s0, 0], [s0, c0, 0], [0, 0, 1]]
        if axis == 'x':
            rr = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        else:
            rr = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        r[:3, :3] = r[:3, :3] @ rr
        out[id] = world(n['parent']) @ r if n['parent'] else r
        return out[id]
    for id in by:
        world(id)
    return out

def check_room_bounds(layout):
    mats = transforms(layout)
    violations = []
    for n in layout['nodes']:
        if n['kind'] != 'geom' or n['id'].startswith('room_'):
            continue
        if n['shape'] in ('tube', 'profile'):
            vs, _ = surface(n)
            q = np.column_stack((np.array(vs), np.ones(len(vs))))
        else:
            sz = np.array(n['size']) / 2
            q = np.array([[x, y, z, 1] for x in (-sz[0], sz[0]) for y in (-sz[1], sz[1]) for z in (-sz[2], sz[2])])
        xyz = (q @ mats[n['id']].T)[:, :3]
        if np.any(xyz.min(0) < -1e-05) or np.any(xyz.max(0) > np.array(ROOM) + 1e-05):
            violations.append(n['id'])
    if violations:
        raise ValueError('组件超出固定房间边界：' + ', '.join(violations[:6]))
    return True

def surface(n, segments=48):
    shape = n['shape']
    verts = []
    faces = []
    if shape == 'tube':
        ro, ri = n['radii']
        h = n['height']
        prof = [(0, ro), (h, ro), (h, ri), (0, ri)]
    elif shape == 'profile':
        prof = n['profile']
    else:
        raise ValueError(shape)
    for z, r in prof:
        for i in range(segments):
            t = i * math.tau / segments
            verts.append([r * math.cos(t), r * math.sin(t), z])
    for j in range(len(prof)):
        for i in range(segments):
            faces.append([j * segments + i, j * segments + (i + 1) % segments, (j + 1) % len(prof) * segments + (i + 1) % segments, (j + 1) % len(prof) * segments + i])
    return (verts, faces)

def polygon_parts(n, segments=48):
    prof = n['profile'] if n['shape'] == 'profile' else [(0, n['radii'][0]), (n['height'], n['radii'][0]), (n['height'], n['radii'][1]), (0, n['radii'][1])]
    parts = []
    for i in range(segments):
        vs = []
        for t in (i * math.tau / segments, (i + 1) * math.tau / segments):
            for z, r in prof:
                vs.append([r * math.cos(t), r * math.sin(t), z])
        parts.append(vs)
    return parts

def copy_subtree(layout, id, new_id, pos_delta=(0.2, 0, 0)):
    by = validate(layout)
    if id == 'room' or id.startswith('room_'):
        raise ValueError('房间外壳不能复制')
    selected = []

    def descend(i):
        selected.append(i)
        for n in layout['nodes']:
            if n['parent'] == i:
                descend(n['id'])
    descend(id)
    mapping = {s: new_id + s[len(id):] for s in selected}
    if set(mapping.values()) & set(by):
        raise ValueError('副本名称已存在')
    for s in selected:
        n = copy.deepcopy(by[s])
        n['id'] = mapping[s]
        n['parent'] = mapping.get(n['parent'], n['parent'])
        if s == id:
            n['pos'] = (np.array(n['pos']) + pos_delta).tolist()
        layout['nodes'].append(n)
    return new_id

def write_json(path, value):
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        pending = path.with_suffix(path.suffix + '.pending')
        pending.write_text(json.dumps(value, ensure_ascii=False, indent=2))
        pending.replace(path)
    except OSError as e:
        raise RuntimeError(f'Cannot write {path}: {e}') from e

def export_mjcf(layout, dest):
    import mujoco as m
    validate(layout)
    check_room_bounds(layout)
    dest = Path(dest)
    dest.mkdir(parents=True, exist_ok=True)
    src = ROOT / 'scene/pharmacy.xml'
    if layout.get('robot_hand_profile') == 'Handle-L1-right-v5':
        src = OUT / 'templates/pharmacy_v1.xml'
    if not src.exists():
        src = ROOT / 'templates/pharmacy_v1.xml'
    xml = ET.parse(src).getroot()
    xml.set('model', 'Pharmacy_workshop_fixed_room')
    wb = xml.find('worldbody')
    arm = wb.find(".//body[@name='arm/Base_R']")
    if arm is None:
        raise RuntimeError('Original arm root not found')
    arm = copy.deepcopy(arm)
    for c in list(wb):
        if c.tag in ('geom', 'camera', 'light', 'frame', 'body'):
            wb.remove(c)
    keys = xml.find('keyframe')
    if keys is not None:
        xml.remove(keys)
    asset = xml.find('asset')
    meshdir = dest / 'meshes'
    meshdir.mkdir(exist_ok=True)
    for mesh in list(asset.findall('mesh')):
        if not mesh.get('file'):
            asset.remove(mesh)
    for mesh in asset.findall('mesh'):
        if not mesh.get('file'):
            continue
        srcmesh = src.parent / mesh.get('file')
        shutil.copy2(srcmesh, meshdir / srcmesh.name)
        mesh.set('file', 'meshes/' + srcmesh.name)
    by = validate(layout)
    bodies = {}
    mats = transforms(layout)
    for n in layout['nodes']:
        if n['kind'] == 'geom':
            continue
        par = bodies.get(n['parent'], wb)
        p = n['pos']
        yaw = n['yaw']
        attrib = {'name': n['id'], 'pos': ' '.join(map(str, p)), 'quat': f'{math.cos(yaw / 2)} 0 0 {math.sin(yaw / 2)}'}
        if n.get('angle'):
            a = n['angle'] / 2
            rot = [math.cos(a), math.sin(a) if n.get('axis') == 'x' else 0, 0, 0 if n.get('axis') == 'x' else math.sin(a)]
            q = np.zeros(4)
            m.mju_mulQuat(q, np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]), np.array(rot))
            attrib['quat'] = ' '.join(map(str, q))
        bodies[n['id']] = ET.SubElement(par, 'body', attrib)
    station = bodies.get('robot_station')
    if station is not None:
        arm.set('pos', '0 0 0.9')
        arm.set('quat', '0 0 0 1')
        station.append(arm)
    else:
        for tag in ('actuator', 'contact', 'equality', 'tendon', 'sensor'):
            e = xml.find(tag)
            if e is not None:
                xml.remove(e)
    for n in layout['nodes']:
        if n['kind'] != 'geom':
            continue
        par = bodies[n['parent']]
        shape = n['shape']
        rgba = COLORS[n['material']]
        common = dict(name=n['id'], pos=' '.join(map(str, n['pos'])), rgba=' '.join(map(str, rgba)), friction='.6 .01 .001')
        if n.get('collision') is False:
            common.update(contype='0', conaffinity='0')
        if shape in ('box', 'cylinder', 'ellipsoid'):
            common['type'] = shape
            sz = np.array(n['size']) / 2
            if shape == 'cylinder':
                sz = sz[[0, 2]]
            common['size'] = ' '.join(map(str, sz))
            ET.SubElement(par, 'geom', common)
        else:
            verts, faces = surface(n)
            meshname = 'mesh_' + n['id']
            tri = []
            for f in faces:
                tri.extend([f[:3], [f[0], f[2], f[3]]])
            ET.SubElement(asset, 'mesh', name=meshname, vertex=' '.join((str(v) for row in verts for v in row)), face=' '.join((str(v) for row in tri for v in row)))
            visual = dict(common, type='mesh', mesh=meshname, contype='0', conaffinity='0')
            ET.SubElement(par, 'geom', visual)
            if n.get('collision') is not False:
                for i, vs in enumerate(polygon_parts(n, 24)):
                    mn = f'{meshname}_collision{i}'
                    ET.SubElement(asset, 'mesh', name=mn, vertex=' '.join((str(v) for row in vs for v in row)))
                    ET.SubElement(par, 'geom', name=f"{n['id']}_collision{i}", pos=common['pos'], type='mesh', mesh=mn, rgba='0 0 0 0', group='3', friction='.6 .01 .001')
    if 'test_medicine_box' in bodies:
        b = bodies['test_medicine_box']
        oldpar = bodies[by['test_medicine_box']['parent']]
        oldpar.remove(b)
        wb.append(b)
        t = mats['test_medicine_box']
        b.set('pos', ' '.join(map(str, t[:3, 3])))
        q = np.zeros(4)
        m.mju_mat2Quat(q, t[:3, :3].ravel())
        b.set('quat', ' '.join(map(str, q)))
        ET.SubElement(b, 'freejoint', name='test_carton_free')
        for g in b.findall('.//geom'):
            g.set('mass', '.006')
    ET.SubElement(wb, 'light', pos='1.8 3.2 4', dir='0 0 -1', diffuse='.8 .8 .8')
    temp = dest / 'pharmacy.xml'
    ET.indent(xml)
    ET.ElementTree(xml).write(temp, encoding='utf-8', xml_declaration=True)
    model = m.MjModel.from_xml_path(str(temp))
    data = m.MjData(model)
    for i, v in enumerate([0, 0.6, 0, -1.6, 0, 0.8, 0], 1):
        j = m.mj_name2id(model, m.mjtObj.mjOBJ_JOINT, f'arm/Joint{i}_R')
        if j >= 0:
            data.qpos[model.jnt_qposadr[j]] = v
    for a in range(model.nu):
        j = model.actuator_trnid[a, 0]
        data.ctrl[a] = data.qpos[model.jnt_qposadr[j]]
    key = ET.SubElement(xml, 'keyframe')
    ET.SubElement(key, 'key', name='home', qpos=' '.join(map(str, data.qpos)), ctrl=' '.join(map(str, data.ctrl)))
    ET.indent(xml)
    ET.ElementTree(xml).write(temp, encoding='utf-8', xml_declaration=True)
    return temp

def package_botlab(xmlpath, out):
    import mujoco as m
    if 'right_hand' in Path(xmlpath).read_text() or 'right_thumb_mcp_roll_joint2' in Path(xmlpath).read_text():
        from handle_l1 import package
        return package(xmlpath, out, task=False)
    out = Path(out)
    out.mkdir(exist_ok=True, parents=True)
    dest = out / 'assets/pharmacy'
    shutil.copytree(xmlpath.parent, dest, dirs_exist_ok=True)
    for extra in ('layout.json', 'layout_empty.json', 'modules.json'):
        (dest / extra).unlink(missing_ok=True)
    for p in list(dest.glob('*.blend')) + list(dest.glob('*.glb')):
        p.unlink()
    model = m.MjModel.from_xml_path(str(xmlpath))
    data = m.MjData(model)
    m.mj_resetDataKeyframe(model, data, 0)
    template = ROOT / 'botlab_project/controller.py'
    if not template.exists():
        template = ROOT / 'templates/controller_v1.py'
    ctrl = template.read_text()
    start = ctrl.index('config=')
    end = ctrl.index('\n\ndef create_controller', start)
    cfg = {'actionDimension': int(model.nu), 'jointQposOffset': 0, 'jointQvelOffset': 0, 'initialQpos': data.qpos.tolist(), 'initialQvel': data.qvel.tolist(), 'defaultJointPositions': data.qpos[:23].tolist(), 'policyJointOrder': list(range(23)), 'motorJointOrder': list(range(23)), 'actuatorJointOrder': [int(model.actuator_trnid[a, 0]) for a in range(model.nu)], 'actionScale': 1}
    ctrl = ctrl[:start] + 'config=' + repr(cfg) + ')' + ctrl[end:]
    ctrl = ctrl.replace('Pharmacy · Marvin + LEAP', 'Pharmacy workshop v2')
    (out / 'controller.py').write_text(ctrl)
    main = ROOT / 'botlab_project/main.py'
    if not main.exists():
        main = ROOT / 'templates/main.py'
    if main.resolve() != (out / 'main.py').resolve():
        shutil.copy2(main, out / 'main.py')
    (out / 'README.md').write_text('药房工作坊 v2：固定房间，Marvin 7关节 + LEAP四指替代手16执行器。控制验证不代表抓取或训练完成。\n')
    files = [str(p.relative_to(out)) for p in dest.rglob('*') if p.is_file()]
    manifest = {'format': 'botlab-project', 'version': 1, 'files': ['README.md', 'controller.py', 'main.py'], 'assets': [{'resourceId': 'Pharmacy' if p.endswith('pharmacy.xml') else 'workspace-asset-' + p.replace('/', '-'), 'source': p, 'path': p, 'name': Path(p).name, 'mimeType': 'application/octet-stream', 'role': 'robot' if p.endswith('pharmacy.xml') else 'mesh'} for p in files], 'assetBase': 'project', 'assetPaths': files}
    write_json(out / 'botlab.project.json', manifest)
    zpath = out.parent / 'pharmacy_workshop_v2.botlab.zip'
    with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
        for p in out.rglob('*'):
            if p.is_file():
                z.write(p, p.relative_to(out))
    return zpath

def build_workshop():
    OUT.mkdir(exist_ok=True)
    layout = initial_layout()
    validate(layout)
    write_json(OUT / 'layout_reference.json', layout)
    empty = copy.deepcopy(layout)
    empty['nodes'] = fixed_room()
    empty['zones'] = []
    empty['routes'] = []
    write_json(OUT / 'layout_empty.json', empty)
    library = {k: prototype(k, k) for k in LABELS}
    write_json(OUT / 'module_library.json', library)
    export_mjcf(layout, OUT / 'simulation')
    package_botlab(OUT / 'simulation/pharmacy.xml', OUT / 'botlab_project')
    print('Built fixed-room modular workshop:', OUT)
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--layout', type=Path)
    parser.add_argument('--dest', type=Path)
    args = parser.parse_args()
    try:
        if args.layout:
            layout = json.loads(args.layout.read_text())
            dest = args.dest or args.layout.parent
            xml = export_mjcf(layout, dest / 'simulation')
            if any((n.get('prototype') == 'robot_station' for n in layout['nodes'])):
                package_botlab(xml, dest / 'botlab_project')
            print('Saved matching simulation:', xml)
        else:
            build_workshop()
    except Exception as exc:
        raise RuntimeError(f'Workshop generation failed: {exc}') from exc
