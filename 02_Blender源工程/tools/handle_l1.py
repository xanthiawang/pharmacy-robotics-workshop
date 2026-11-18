from pathlib import Path
import copy, csv, hashlib, json, math, shutil, struct, xml.etree.ElementTree as E
import numpy as np
import mujoco as m
from PIL import Image
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
import workshop as w
ROOT = w.ROOT
OUT = ROOT / 'handle_l1_v4_2026-11-17'
FINGERS = ['index', 'middle', 'ring', 'pinky']
ACTIVE = ['right_thumb_mcp_roll_joint2', 'right_thumb_mcp_pitch_joint3', 'right_index_mcp_joint5', 'right_middle_mcp_joint7', 'right_ring_mcp_joint9', 'right_pinky_mcp_joint11']
PASSIVE = ['right_thumb_dip_joint4', 'right_index_dip_joint6', 'right_middle_dip_joint8', 'right_ring_dip_joint10', 'right_pinky_dip_joint12']

def graph_coupling():
    results = {}
    for finger in ['thumb', *FINGERS]:
        p = OUT / f'coupling_source_{finger}.png'
        im = np.array(Image.open(p).convert('RGB'), dtype=float)
        xmin, xmax, ymin, ymax = (63, 549, 52, 609)
        rgb = im[ymin:ymax + 1, xmin:xmax + 1]
        mask = (rgb[:, :, 2] > 130) & (rgb[:, :, 2] - rgb[:, :, 0] > 35) & (rgb[:, :, 2] - rgb[:, :, 1] > 10)
        ymaxrad = 0.8 if finger == 'thumb' else 1.8
        xmaxrad = 0.6 if finger == 'thumb' else 1.6
        rows = []
        for x in range(mask.shape[1]):
            y = np.where(mask[:, x])[0]
            if len(y):
                rows.append((x / (xmax - xmin) * xmaxrad, (ymax - ymin - np.median(y)) / (ymax - ymin) * ymaxrad))
        xy = np.array(rows)
        xs, ys = xy.T
        a = np.column_stack([xs ** k for k in range(1, 5)])
        coef = np.r_[0, np.linalg.lstsq(a, ys, rcond=None)[0]]
        err = np.polynomial.polynomial.polyval(xs, coef) - ys
        with (OUT / f'coupling_digitized_{finger}.csv').open('w') as f:
            writer = csv.writer(f)
            writer.writerow(['mcp_rad', 'dip_rad'])
            writer.writerows(rows)
        results[finger] = {'source': str(p), 'source_sha256': hashlib.sha256(p.read_bytes()).hexdigest(), 'calibration_pixels': [xmin, xmax, ymin, ymax], 'axis_max_rad': [xmaxrad, ymaxrad], 'observed_domain_rad': [float(xs.min()), float(xs.max())], 'polycoef': coef.tolist(), 'max_fit_error_rad': float(abs(err).max()), 'pixel_rad': ymaxrad / (ymax - ymin), 'scope': 'Digitized manufacturer manual page3; approximate curve, not supplied transmission equation.'}
    w.write_json(OUT / 'coupling_fit.json', results)
    return results

def raw_vertices(path):
    raw = path.read_bytes()
    n = struct.unpack_from('<I', raw, 80)[0]
    dt = np.dtype([('normal', '<f4', (3,)), ('verts', '<f4', (3, 3)), ('attr', '<u2')])
    return np.frombuffer(raw, dtype=dt, count=n, offset=84)['verts'].reshape(-1, 3).astype(float)

def converted_hand(side='right'):
    source = ROOT / 'assets/handle_l1' / f'{side}_hand'
    ver = 'v5' if side == 'right' else 'v2'
    r = E.parse(source / 'urdf' / f'{side}_hand{ver}.urdf').getroot()
    for mesh in r.findall('.//mesh'):
        mesh.set('filename', str(source / 'meshes' / Path(mesh.get('filename')).name))
    mj = E.SubElement(r, 'mujoco')
    E.SubElement(mj, 'compiler', discardvisual='false', fusestatic='false')
    dest = OUT / f'{side}_normalized.urdf'
    E.ElementTree(r).write(dest, encoding='utf-8', xml_declaration=True)
    spec = m.MjSpec.from_file(str(dest))
    model = spec.compile()
    xml = E.fromstring(spec.to_xml())
    bylink = {l.get('name'): l for l in r.findall('link')}
    byjoint = {j.find('child').get('link'): j for j in r.findall('joint')}
    for b in xml.findall('.//body'):
        original = bylink[b.get('name')]
        ib = original.find('inertial')
        ei = b.find('inertial')
        ei.attrib.clear()
        ei.set('pos', ib.find('origin').get('xyz'))
        ei.set('mass', ib.find('mass').get('value'))
        it = ib.find('inertia')
        ei.set('fullinertia', ' '.join((it.get(k) for k in ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'])))
        if b.get('name') in byjoint:
            j = byjoint[b.get('name')]
            b.set('pos', j.find('origin').get('xyz'))
            quat = Rotation.from_euler('xyz', [float(z) for z in j.find('origin').get('rpy', '0 0 0').split()]).as_quat()
            b.set('quat', ' '.join(map(str, [quat[3], *quat[:3]])))
            e = b.find('joint')
            e.set('axis', j.find('axis').get('xyz'))
            e.set('range', j.find('limit').get('lower') + ' ' + j.find('limit').get('upper'))
        geoms = b.findall('geom')
        for i, g in enumerate(geoms):
            g.set('name', b.get('name') + ('_visual' if i == 0 else '_collision'))
            if i == 1:
                g.set('group', '3')
                g.set('rgba', '0 0 0 0')
    return xml

def build():
    OUT.mkdir(exist_ok=True)
    fits = graph_coupling()
    hand = converted_hand()
    base = E.parse(ROOT / 'grasp_v3_2026-11-17/simulation/pharmacy.xml').getroot()
    base.set('model', 'Pharmacy_Marvin_Handle_L1_supplied_right_v5')
    for k in list(base.findall('keyframe')):
        base.remove(k)
    for parent in base.iter():
        for child in list(parent):
            if child.get('name') == 'hand/palm' or child.get('class', '').startswith('hand/') or child.get('name', '').startswith('hand/') or any((v.startswith('hand/') for k, v in child.attrib.items() if k in ['body1', 'body2', 'joint'])):
                parent.remove(child)
    for element in hand.iter():
        for key in ['name', 'mesh']:
            if element.get(key):
                element.set(key, 'hand/' + element.get(key))
    root = hand.find('worldbody/body')
    root.set('name', 'hand/palm')
    root.set('pos', '0 0 .015')
    root.set('quat', '1 0 0 0')
    oldframe = base.find(".//body[@name='arm/Link7_R']/frame")
    if oldframe is None:
        raise RuntimeError('Missing flange frame')
    oldframe.append(root)
    meshdir = OUT / 'simulation/meshes'
    meshdir.mkdir(parents=True, exist_ok=True)
    for e in base.find('asset').findall('mesh'):
        if not e.get('file'):
            continue
        src = ROOT / 'grasp_v3_2026-11-17/simulation' / e.get('file')
        shutil.copy2(src, meshdir / src.name)
    for e in hand.find('asset'):
        src = Path(e.get('file'))
        shutil.copy2(src, meshdir / src.name)
        e.set('file', 'meshes/' + src.name)
        base.find('asset').append(e)
    for b in root.iter('body'):
        b.set('gravcomp', '1')
    for j in root.iter('joint'):
        j.set('damping', '.02')
        j.set('armature', '.00005')
        j.set('solreflimit', '.004 1')
        j.set('solimplimit', '.99 .999 .001')
    for g in root.iter('geom'):
        if g.get('contype') != '0':
            g.set('friction', '.8 .01 .001')
            g.set('condim', '4')
            g.set('solref', '.006 1')
            g.set('solimp', '.95 .99 .001')
    eq = base.find('equality')
    if eq is None:
        eq = E.SubElement(base, 'equality')
    for finger, pas, act in zip(['thumb', *FINGERS], PASSIVE, [ACTIVE[1], *ACTIVE[2:]]):
        E.SubElement(eq, 'joint', name='coupling_' + finger, joint1='hand/' + pas, joint2='hand/' + act, polycoef=' '.join(map(str, fits[finger]['polycoef'])), solref='.004 1')
    E.SubElement(base.find('contact'), 'exclude', body1='hand/palm', body2='hand/right_thumb_mcp_pitch_link3')
    for name in ACTIVE:
        rng = '0 2.0944' if name == ACTIVE[0] else '0 .5236' if name == ACTIVE[1] else '0 1.4'
        E.SubElement(base.find('actuator'), 'general', name='hand/' + name + '_act', joint='hand/' + name, input='pos', biastype='affine', gainprm='3', biasprm='0 -3 -.04', ctrllimited='true', ctrlrange=rng, forcelimited='true', forcerange='-.5 .5')
    tips = {}
    for finger, idx in [('thumb', 4), ('index', 6), ('middle', 8), ('ring', 10), ('pinky', 12)]:
        bn = f'right_{finger}_dip_link{idx}'
        verts = raw_vertices(ROOT / 'assets/handle_l1/right_hand/meshes' / (bn + '.STL'))
        axis = 1 if finger == 'thumb' else 2
        v = verts[verts[:, axis] > verts[:, axis].max() - 0.004]
        tip = v.mean(0)
        tips[finger] = tip.tolist()
        E.SubElement(root.find(".//body[@name='hand/" + bn + "']"), 'site', name='tip_' + finger, pos=' '.join(map(str, tip)), size='.002', rgba='1 .5 0 1')
    for name, pos in [('Task', [3.35, 2.15, 1.85]), ('Task_reverse', [1.6, 3.0, 1.6])]:
        target = np.array([2.4, 3.25, 1.0])
        z = np.array(pos) - target
        z /= np.linalg.norm(z)
        x = np.cross([0, 0, 1], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        E.SubElement(base.find('worldbody'), 'camera', name=name, pos=' '.join(map(str, pos)), xyaxes=' '.join(map(str, np.r_[x, y])), fovy='45')
    path = OUT / 'simulation/pharmacy.xml'
    E.indent(base)
    E.ElementTree(base).write(path, encoding='utf-8', xml_declaration=True)
    model = m.MjModel.from_xml_path(str(path))
    d = m.MjData(model)
    for i, val in enumerate([0, 0.6, 0, -1.6, 0, 0.8, 0], 1):
        d.qpos[model.joint(f'arm/Joint{i}_R').qposadr[0]] = val
    for a in range(model.nu):
        d.ctrl[a] = d.qpos[model.jnt_qposadr[model.actuator_trnid[a, 0]]]
    k = E.SubElement(base, 'keyframe')
    E.SubElement(k, 'key', name='home', qpos=' '.join(map(str, d.qpos)), ctrl=' '.join(map(str, d.ctrl)))
    E.indent(base)
    E.ElementTree(base).write(path, encoding='utf-8', xml_declaration=True)
    w.write_json(OUT / 'adapter_parameters.json', {'tip_sites_from_source_STL': tips, 'active_joints': ACTIVE, 'passive_joints': PASSIVE, 'robot_actuators': model.nu, 'mass_scope': 'Preserved supplied CAD masses; no scale to610g', 'coupling_scope': 'Five functions digitized from manual p3, not exact factory transmission equations', 'software_test_settings': {'hand_position_kp': 3, 'hand_position_kv': 0.04, 'hand_joint_torque_cap_Nm': 0.5, 'surface_sliding_friction': 0.8, 'flange_adapter_extension_m': 0.015}, 'unprovided_fields': ['real actuator effort limits', 'exact coupling functions', 'validated adapter mounting transform'], 'source_conflicts': ['thumb pitch URDF30deg vs manual34deg', 'URDF mass230.94g vs manual610g']})
    print('Built actual Handle-L1 scene:', model.nq, model.nv, model.nu, flush=True)
    return path

def set_hand(model, data, values):
    fits = json.loads((OUT / 'coupling_fit.json').read_text())
    for name, v in zip(ACTIVE, values):
        data.qpos[model.joint('hand/' + name).qposadr[0]] = v
    for f, pas, act in zip(['thumb', *FINGERS], PASSIVE, [values[1], *values[2:]]):
        data.qpos[model.joint('hand/' + pas).qposadr[0]] = np.polynomial.polynomial.polyval(act, fits[f]['polycoef'])

def joint_report(model, poses):
    poses = np.asarray(poses)
    rows = []
    for j in range(model.njnt):
        if not model.jnt_limited[j]:
            continue
        q = poses[:, model.jnt_qposadr[j]]
        lo, hi = model.jnt_range[j]
        rows.append(dict(joint=model.joint(j).name, maximum_violation_rad=max(0.0, float(lo - q.min()), float(q.max() - hi))))
    maximum = max((r['maximum_violation_rad'] for r in rows))
    return dict(max_joint_limit_violation_rad=maximum, joint_limit_numerical_tolerance_rad=0.012, joint_limits_passed=maximum < 0.012, joint_limits=rows)

def trial():
    from grasp_pipeline import arm_ik, contacts, validate_log
    path = build()
    model = m.MjModel.from_xml_path(str(path))
    d = m.MjData(model)
    m.mj_resetDataKeyframe(model, d, 0)
    cfg = json.loads((OUT / 'trial_settings.json').read_text()) if (OUT / 'trial_settings.json').exists() else {}
    rot = Rotation.from_euler('z', 180, degrees=True).as_matrix() @ Rotation.from_euler('y', 90, degrees=True).as_matrix()
    palms = [[2.415, 3.12, 1.165], [2.415, 3.12, 0.885], [2.415, 3.12, 1.165], [2.295, 3.45, 1.165], [2.295, 3.45, 1.055]]
    palms = cfg.get('palms', palms)
    arm = []
    seed = np.array([0.2, -0.8, 0, -1.5, 1.5, 0, -0.5])
    for pos in palms:
        q, e = arm_ik(model, d, np.array(pos), rot, seed)
        print('IK', pos, e, flush=True)
        if e > 0.002:
            raise RuntimeError('Handle-L1 palm target unreachable')
        arm.append(q)
        seed = q
    opened = np.array(cfg.get('opened', [0, 0, 0.12, 0.12, 0.12, 0.12]))
    closed = np.array(cfg.get('closed', [1.9, 0.23, 0.6, 0.65, 0.6, 0.5]))
    start = np.r_[arm[0], opened]
    m.mj_resetDataKeyframe(model, d, 0)
    for a, v in enumerate(start):
        d.qpos[model.jnt_qposadr[model.actuator_trnid[a, 0]]] = v
    set_hand(model, d, opened)
    d.ctrl[:] = start
    m.mj_forward(model, d)
    xml = E.parse(path)
    key = xml.find('.//key')
    key.set('qpos', ' '.join(map(str, d.qpos)))
    key.set('ctrl', ' '.join(map(str, start)))
    xml.write(path, encoding='utf-8', xml_declaration=True)
    stages = [('settle', 1.0, start), ('approach', 3.0, np.r_[arm[1], opened]), ('close', 2.0, np.r_[arm[1], closed]), ('lift', 3.0, np.r_[arm[2], closed]), ('hold', 2.0, np.r_[arm[2], closed]), ('transport', 4.0, np.r_[arm[3], closed]), ('lower', 3.0, np.r_[arm[4], closed]), ('release', 2.0, np.r_[arm[4], opened]), ('withdraw', 3.0, np.r_[arm[3], opened]), ('settle_in_bin', 2.0, np.r_[arm[3], opened])]
    history = []
    poses = []
    commands = []
    previous = start.copy()
    b = model.body('test_medicine_box').id
    env_events = []
    for stage, duration, target in stages:
        for tick in range(round(duration * 50)):
            u = (tick + 1) / (duration * 50)
            blend = u * u * (3 - 2 * u)
            d.ctrl[:] = previous + (target - previous) * blend
            for _ in range(10):
                m.mj_step(model, d)
            cs = contacts(model, d)
            history.append(dict(time=float(d.time), stage=stage, box_position=d.xpos[b].tolist(), box_quaternion=d.xquat[b].tolist(), qvel=d.qvel[-6:].tolist(), contacts=cs))
            poses.append(d.qpos.copy())
            commands.append(d.ctrl.copy())
            for c in d.contact:
                bodies = [model.body(int(model.geom_bodyid[g])).name for g in [c.geom1, c.geom2]]
                flags = [name.startswith(('hand/', 'arm/')) for name in bodies]
                if flags[0] != flags[1] and 'test_medicine_box' not in bodies:
                    env_events.append(dict(time=float(d.time), bodies=bodies, distance=float(c.dist)))
        print(stage, d.xpos[b], [(c['other'], round(c['force_contact_frame'][0], 2)) for c in cs], flush=True)
        previous = target
    np.savez_compressed(OUT / 'full_states.npz', qpos=poses, ctrl=commands)
    w.write_json(OUT / 'full_log.json', history)
    w.write_json(OUT / 'full_program.json', [dict(stage=n, duration=t, target=q.tolist()) for n, t, q in stages])
    report = validate_log(history)
    report.update(joint_report(model, poses))
    report['warning_count'] = sum((int(x.number) for x in d.warning))
    report['robot_environment_contacts'] = env_events
    report['passed'] = bool(report['passed'] and report['joint_limits_passed'] and (report['warning_count'] == 0) and (not env_events))
    w.write_json(OUT / 'physical_verification.json', report)
    print('PASS', report['passed'], 'warnings', report['warning_count'], 'environment', len(env_events), flush=True)

def search_grasp():
    from grasp_pipeline import arm_ik, contacts
    model = m.MjModel.from_xml_path(str(OUT / 'simulation/pharmacy.xml'))
    d = m.MjData(model)
    rot = Rotation.from_euler('z', 180, degrees=True).as_matrix() @ Rotation.from_euler('y', 90, degrees=True).as_matrix()
    seed = np.array([0.7, -0.6, -0.5, -1.9, 1.9, 0, -0.8])
    results = []
    presets = [[1.55, 0.15, 0.8, 0.85, 0.8, 0.75], [1.8, 0.35, 0.9, 0.95, 0.9, 0.85], [2.05, 0.25, 0.8, 0.85, 0.8, 0.75], [1.7, 0.48, 0.6, 0.65, 0.6, 0.55]]
    b = model.body('test_medicine_box').id
    for dx in [0.09, 0.1, 0.11]:
        for z in [0.875, 0.885, 0.895]:
            arms = []
            for zz in [z + 0.18, z, z + 0.18]:
                q, e = arm_ik(model, d, np.array([2.3 + dx, 3.12, zz]), rot, seed)
                seed = q
                arms.append(q)
                if e > 0.002:
                    break
            if len(arms) != 3 or e > 0.002:
                continue
            for closed in presets:
                opened = np.array([0, 0, 0.12, 0.12, 0.12, 0.12])
                initial = np.r_[arms[0], opened]
                m.mj_resetDataKeyframe(model, d, 0)
                for a, v in enumerate(initial):
                    d.qpos[model.jnt_qposadr[model.actuator_trnid[a, 0]]] = v
                set_hand(model, d, opened)
                d.ctrl[:] = initial
                m.mj_forward(model, d)
                prev = initial.copy()
                held = []
                violation = 0
                for phase, duration, target in [('settle', 0.4, initial), ('approach', 1.2, np.r_[arms[1], opened]), ('close', 1.5, np.r_[arms[1], closed]), ('lift', 1.5, np.r_[arms[2], closed]), ('hold', 1.2, np.r_[arms[2], closed])]:
                    for tick in range(round(duration * 50)):
                        u = (tick + 1) / (duration * 50)
                        blend = u * u * (3 - 2 * u)
                        d.ctrl[:] = prev + (target - prev) * blend
                        for _ in range(10):
                            m.mj_step(model, d)
                        for j in range(7, 18):
                            val = d.qpos[model.jnt_qposadr[j]]
                            lo, hi = model.jnt_range[j]
                            violation = max(violation, float(lo - val), float(val - hi))
                        if phase == 'hold':
                            held.append((float(d.xpos[b, 2]), contacts(model, d)))
                    prev = target.copy()
                only_hand = all((cs and all((c['other'].startswith('hand/') for c in cs)) for _, cs in held))
                row = {'dx': dx, 'palm_z': z, 'closed': closed, 'min_hold_box_z': min((v for v, _ in held)), 'final_box_position': d.xpos[b].tolist(), 'max_joint_limit_violation_rad': violation, 'held_only_by_hand': only_hand}
                row['passed'] = bool(row['min_hold_box_z'] > 0.91 and only_hand and (violation < 0.012))
                results.append(row)
                print('search', len(results), row, flush=True)
                w.write_json(OUT / 'pose_search.json', results)
                if row['passed']:
                    w.write_json(OUT / 'pose_search_winner.json', row)
                    return
    raise RuntimeError('Bounded pose search found no stable grip; inspect real trial results')

def robot_config(model, data):
    positions = [float(data.qpos[model.jnt_qposadr[int(model.actuator_trnid[a, 0])]]) for a in range(model.nu)]
    return dict(actionDimension=int(model.nu), jointQposOffset=0, jointQvelOffset=0, initialQpos=data.qpos.tolist(), initialQvel=data.qvel.tolist(), defaultJointPositions=positions, policyJointOrder=list(range(model.nu)), motorJointOrder=list(range(model.nu)), actuatorJointOrder=list(range(model.nu)), actionScale=1)

def package(xmlpath=None, project=None, task=True):
    import zipfile
    path = Path(xmlpath) if xmlpath else OUT / 'simulation/pharmacy.xml'
    project = Path(project) if project else OUT / 'botlab_project'
    project.mkdir(parents=True, exist_ok=True)
    dest = project / 'assets/pharmacy'
    dest.mkdir(parents=True, exist_ok=True)
    tree = E.parse(path)
    files = [path]
    for mesh in tree.findall('.//asset/mesh'):
        if mesh.get('file'):
            files.append(path.parent / mesh.get('file'))
    for f in files:
        target = dest / f.relative_to(path.parent)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(f, target)
    model = m.MjModel.from_xml_path(str(path))
    d = m.MjData(model)
    m.mj_resetDataKeyframe(model, d, 0)
    cfg = robot_config(model, d)
    src = 'from botlab import Robot, PolicyController, constant, project, state_machine, concat\n'
    parameters = {} if task else {f'joint_{a}': dict(label=model.joint(int(model.actuator_trnid[a, 0])).name, default=cfg['defaultJointPositions'][a], range=model.actuator_ctrlrange[a].tolist(), step=0.01, unit='rad') for a in range(model.nu)}
    src += 'project("Pharmacy Handle-L1 ' + ('pick and place' if task else 'layout workshop') + '", control_hz=50, physics_hz=500, parameters=' + repr(parameters) + ')\n'
    src += 'robot=Robot("assets/pharmacy/pharmacy.xml",name="Pharmacy",config=' + repr(cfg) + ')\n'
    if task:
        prog = json.loads((OUT / 'full_program.json').read_text())
        src += 'TARGETS=' + repr([q['target'] for q in prog]) + '\nSTEPS=' + repr([round(q['duration'] * 50) for q in prog]) + '\n'
        src += 'def create_controller(sim):\n    clock=state_machine(constant([0]),initial=0,transitions=[{"from":0,"to":1,"after_steps":1000000,"reset_steps":False}])\n    output=constant(TARGETS[0])\n    start=0\n    previous=TARGETS[0]\n    for target,duration in zip(TARGETS,STEPS):\n        u=((clock.steps+1-start)/duration).clip(0,1)\n        blend=u*u*(3-2*u)\n        output=output+constant([b-a for a,b in zip(previous,target)])*blend\n        start+=duration\n        previous=target\n    return PolicyController(robot,control=output.action(),end_after_steps=sum(STEPS))\n'
    else:
        src += 'def create_controller(sim):\n    controls=[]\n'
        for a in range(model.nu):
            src += f'    controls.append(robot.commands("joint_{a}"))\n'
        src += '    return PolicyController(robot,control=concat(controls).action())\n'
    (project / 'controller.py').write_text(src)
    (project / 'main.py').write_text('from botlab import Simulation\nfrom controller import create_controller\nsim=Simulation.from_project()\nsim.run(create_controller(sim))\n')
    (project / 'README.md').write_text('Marvin M6S + supplied Handle-L1 right_handv5. 7 arm + 6 active hand controls; 5 passive joints follow curves digitized from the supplier manual. Meshes and CAD inertias preserved. Torque, contact and mounting settings are authored simulation settings, not hardware calibration. ' + ('Run the 25-second contact-driven pick-and-place. Reset to restart.' if task else 'Manual joint controls. After changing a layout, plan and verify a new grasp trajectory.') + '\n')
    names = [str(f.relative_to(project)) for f in dest.rglob('*') if f.is_file()]
    w.write_json(project / 'botlab.project.json', dict(format='botlab-project', version=1, files=['README.md', 'controller.py', 'main.py'], assets=[dict(resourceId='Pharmacy' if f.endswith('pharmacy.xml') else 'workspace-asset-' + f.replace('/', '-'), source=f, path=f, name=Path(f).name, mimeType='application/octet-stream', role='robot' if f.endswith('pharmacy.xml') else 'mesh') for f in names], assetBase='project', assetPaths=names))
    zpath = project.parent / ('Handle-L1_药盒抓取入箱_2026-11-17.botlab.zip' if task else 'Handle-L1_布局工作坊_2026-11-17.botlab.zip')
    with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in project.rglob('*'):
            if f.is_file() and '__pycache__' not in f.parts:
                z.write(f, f.relative_to(project))
    print('Packaged', zpath, flush=True)
    return zpath

def sdk_check():
    import grasp_pipeline as g
    from botlab import open_headless
    history = []
    poses = []
    cmds = []
    env = []
    prog = json.loads((OUT / 'full_program.json').read_text())
    stages = [st['stage'] for st in prog for _ in range(round(st['duration'] * 50))]
    with open_headless(OUT / 'botlab_project') as run:
        model = run.physics.model
        b = model.body('test_medicine_box').id
        for stage in stages:
            run.step()
            d = run.physics.data
            history.append(dict(time=float(d.time), stage=stage, box_position=d.xpos[b].tolist(), box_quaternion=d.xquat[b].tolist(), qvel=d.qvel[-6:].tolist(), contacts=g.contacts(model, d)))
            poses.append(d.qpos.copy())
            cmds.append(d.ctrl.copy())
            for c in d.contact:
                bodies = [model.body(int(model.geom_bodyid[x])).name for x in [c.geom1, c.geom2]]
                flags = [s.startswith(('hand/', 'arm/')) for s in bodies]
                if flags[0] != flags[1] and 'test_medicine_box' not in bodies:
                    env.append(dict(time=float(d.time), bodies=bodies, distance=float(c.dist)))
        report = g.validate_log(history)
        report.update(joint_report(model, poses))
        report.update(warning_count=sum((int(x.number) for x in d.warning)), robot_environment_contacts=env)
        report['passed'] = bool(report['passed'] and report['joint_limits_passed'] and (report['warning_count'] == 0) and (not env))
    np.savez_compressed(OUT / 'sdk_states.npz', qpos=poses, ctrl=cmds)
    w.write_json(OUT / 'sdk_log.json', history)
    w.write_json(OUT / 'sdk_verification.json', report)
    print(json.dumps(report, indent=2), flush=True)
    if not report['passed']:
        raise RuntimeError('Handle-L1 SDK task did not pass')

def web_check():
    with (OUT / 'web_task_trace.csv').open(encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))

    def values(kind, i):
        key = next((k for k in rows[0] if k.endswith(f'[{kind}:{i}]')))
        return np.array([float(r[key]) for r in rows])
    q = np.column_stack([values('qpos', i) for i in range(25)])
    v = np.column_stack([values('qvel', i) for i in range(18, 24)])
    model = m.MjModel.from_xml_path(str(OUT / 'simulation/pharmacy.xml'))
    d = m.MjData(model)
    b = model.body('test_medicine_box').id
    payload = model.geom('payload_collision').id
    held = np.where(q[:, 20] > 0.98)[0]
    pairs = []
    for i in held:
        d.qpos[:] = q[i]
        m.mj_forward(model, d)
        names = []
        for c in d.contact:
            if payload in (c.geom1, c.geom2):
                names.append(model.body(int(model.geom_bodyid[c.geom2 if c.geom1 == payload else c.geom1])).name)
        pairs.append(names)
    d.qpos[:] = q[-1]
    m.mj_forward(model, d)
    rot = Rotation.from_quat([*q[-1, 22:25], q[-1, 21]]).as_matrix()
    corners = np.array([[x, y, z] for x in [-0.035, 0.035] for y in [-0.05, 0.05] for z in [0, 0.09]]) @ rot.T + q[-1, 18:21]
    lo = corners.min(0)
    hi = corners.max(0)
    inside = bool(np.all(lo[:2] >= [2.0075, 3.3025]) and np.all(hi <= [2.3525, 3.5975, 0.938]) and (lo[2] >= 0.766))
    support = [model.geom(int(c.geom2 if c.geom1 == payload else c.geom1)).name for c in d.contact if payload in (c.geom1, c.geom2)]
    separation = float(np.linalg.norm(d.xpos[model.body('hand/palm').id] - d.xpos[b]))
    hand_only = bool(pairs and all((p and all((n.startswith('hand/') for n in p)) for p in pairs)))
    report = dict(source='Actual Dia BotLab Watch CSV; identified by qpos/qvel indices because old channel display names persisted after import', samples=len(rows), wall_time_s=float(rows[-1]['elapsed_wall_time_s']), simulation_duration_s=25, maximum_vertical_lift_m=float(q[:, 20].max() - 0.76), initial_position_m=q[0, 18:21].tolist(), final_position_m=q[-1, 18:21].tolist(), final_box_aabb_m=[lo.tolist(), hi.tolist()], whole_box_inside_bin=inside, final_linear_speed_m_s=float(np.linalg.norm(v[-1, :3])), final_angular_speed_rad_s=float(np.linalg.norm(v[-1, 3:])), held_samples=len(held), only_hand_contacts_in_lifted_samples=hand_only, final_contact_geoms=support, final_palm_to_box_origin_m=separation, contact_scope='Geometric contacts reconstructed from actual web poses; raw contact forces are in native SDK logs only.', status='Completed', diagnostics_errors=0, diagnostics_warnings=0)
    report.update(joint_report(model, q))
    report['passed'] = bool(inside and report['maximum_vertical_lift_m'] > 0.2 and (report['final_linear_speed_m_s'] < 0.001) and hand_only and ('destination_bin__base' in support) and (separation > 0.2) and report['joint_limits_passed'])
    w.write_json(OUT / 'web_verification.json', report)
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise RuntimeError('Handle-L1 actual web trajectory did not pass')

def manual_check():
    from botlab import open_headless
    rows = []
    with open_headless(OUT / 'editable_workshop/botlab_project') as run:
        model = run.physics.model
        initial = run.physics.data.ctrl.copy()
        if model.nu != 13:
            raise RuntimeError('Manual workshop must retain 7+6 actuators')
        for a in range(model.nu):
            for j in range(model.nu):
                run.set_parameter(f'joint_{j}', float(initial[j]))
            target = float(min(initial[a] + 0.12, model.actuator_ctrlrange[a, 1]))
            run.set_parameter(f'joint_{a}', target)
            for _ in range(75):
                run.step()
            d = run.physics.data
            actual = float(d.qpos[model.jnt_qposadr[int(model.actuator_trnid[a, 0])]])
            rows.append(dict(actuator=model.actuator(a).name, target=target, actual=actual, error=abs(actual - target), finite=bool(np.isfinite(d.qpos).all()), moved=bool(abs(actual - initial[a]) > 0.015)))
    report = dict(actuators=model.nu, checks=rows, passed=all((r['finite'] and r['moved'] and (r['error'] < 0.06) for r in rows)), scope='Actual SDK manual command parameters; edited layouts require new grasp planning and validation.')
    w.write_json(OUT / 'manual_controls_verification.json', report)
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise RuntimeError('Handle-L1 manual control check did not pass')

def delivery():
    import zipfile
    D = OUT / 'editable_workshop'
    bundle = OUT / 'delivery'
    bundle.mkdir(exist_ok=True)
    for report in ['physical_verification.json', 'sdk_verification.json', 'web_verification.json', 'manual_controls_verification.json']:
        if not json.loads((OUT / report).read_text())['passed']:
            raise RuntimeError('Delivery blocked by ' + report)
    for name in ['workshop.py', 'workshop_editor.py', 'handle_l1.py']:
        shutil.copy2(ROOT / 'src' / name, D / 'tools' / name)
    (D / '使用说明.md').write_text('# 药房布局工作坊 · Handle-L1\n\n双击“打开药房工作坊.command”；从空房间开始则打开“打开空房间.command”。需要 Blender 5.2。\n\n1. 按 N，进入“药房工作坊”页签。房间边界固定为 3.6 × 6.4 × 2.9 m；这是原型设定，尚未现场实测。\n2. 选中家具后用“选中整个组合”，G 移动，R、Z 水平旋转。柜门、层板、瓶盖、药盒盖可以单独操作。\n3. 可以清空内部、添加模块、重新划分区域和检查动线。动线检查使用二维包围盒，不覆盖柜门扫掠或机械臂可达性。\n4. “保存布局并导出仿真”写入 edits/，包括布局 JSON、BLEND、GLB、MJCF 和 Handle-L1 布局工作坊 BotLab ZIP。外壳变化或家具越界会阻止导出。\n5. BotLab 的 File → Open project file / ZIP 打开布局工作坊 ZIP，Run 后在 Parameters 中操作 7 个机械臂关节和 6 个灵巧手主动关节。5 个从动关节按随动曲线运动。\n6. 改变机器人、药盒或周转箱位置后，需要重新规划并验证抓取轨迹。布局工作坊不自动套用固定演示轨迹。\n\nsupplier_models/ 保留左右手 URDF 与配套 STL 原件；本场景安装右手 right_handv5，左手 left_handv2 留作备用。仿真控制使用 MJCF；GLB 用于布局和几何交换，不包含机器人动力学控制。\n\n导出仿真需 requirements.txt 的 Python 依赖。本机自动寻找项目环境；换电脑后可在本目录建立 .venv 并安装依赖。瓶盖螺纹、撕盒和压药片力学未实现。\n')
    (bundle / '使用说明.md').write_text('# 药房工作坊 · Handle-L1 v4\n\n- 演示工程：Handle-L1_抓取入箱_网页保存_2026-11-17.botlab.zip。BotLab 中 File → Open project file / ZIP 打开，Run 运行 25 秒仿真；Reset 可复位。\n- editable_workshop/：可编辑药房、空房间、模块 GLB、布局编辑器，以及 13 路关节手动控制工程。双击其中的“打开药房工作坊.command”。\n- Handle-L1_抓取入箱_实际仿真_2026-11-17.mp4：来自本地实际物理仿真状态的回放。\n- 验证记录/：原始网页状态 CSV、本地 SDK 结果、源码一致性检查和仿真参数。\n\n当前已完成本地物理仿真、BotLab SDK 和 Dia 网页的抓取入箱；药盒依靠接触与摩擦运动，没有焊接附着或过程位姿写入。尚未接真机，也未训练策略或验证其他布局下的泛化。\n\n右手的 STL、关节轴、原始限位、质量和惯量按所提供 URDF 保留。6 个主动关节、5 个从动关节；随动关系来自手册第 3 页曲线的数字化近似，并非厂商精确传动公式。\n\n仍需厂商确认：URDF 总质量约 231 g 与手册约 610 g 的差异；拇指俯仰 URDF 30° 与手册 34° 的差异；真实力矩、速度限值和安装变换。URDF 的 effort、velocity 均填 0；本次力矩上限、摩擦和 15 mm 转接偏移属于试跑设定，未按实物标定。详见仿真参数和来源核对记录。\n\n原始厂商文件未附公开再分发许可；本包供项目内部协作与教学准备使用。现场照片、姓名与人脸未纳入。\n')
    verification = bundle / '验证记录'
    verification.mkdir(exist_ok=True)
    records = ['physical_verification.json', 'sdk_verification.json', 'sdk_log.json', 'sdk_states.npz', 'full_program.json', 'web_verification.json', 'manual_controls_verification.json', 'supplier_conformance.json', 'adapter_parameters.json', 'source_manifest.json', 'coupling_fit.json', 'web_task_trace.csv', 'web_completed.jpg', 'web_completed_state.txt', 'glb_inspection.json']
    for name in records:
        shutil.copy2(OUT / name, verification / name)
    shutil.copy2(D / 'checks/editor_operators.json', verification / 'editor_operators.json')
    for finger in ['thumb', *FINGERS]:
        for prefix, suffix in [('coupling_source_', '.png'), ('coupling_digitized_', '.csv')]:
            shutil.copy2(OUT / (prefix + finger + suffix), verification / (prefix + finger + suffix))
    video = 'Handle-L1_抓取入箱_实际仿真_2026-11-17.mp4'
    shutil.copy2(OUT / '药盒抓取入箱_实际仿真_2026-11-17.mp4', bundle / video)
    task = 'Handle-L1_抓取入箱_网页保存_2026-11-17.botlab.zip'
    shutil.copy2(OUT / task, bundle / task)
    shutil.copy2(OUT / 'contact_sheet.png', bundle / '抓取过程预览.png')
    allowed = []
    for p in D.rglob('*'):
        rel = p.relative_to(D)
        if p.is_file() and rel.parts[0] not in ('edits', 'checks', 'botlab_project') and ('__pycache__' not in rel.parts) and (p.suffix not in ('.pyc', '.blend1', '.log')) and (p.name != '.DS_Store'):
            allowed.append((p, Path('editable_workshop') / rel))
    for p in bundle.rglob('*'):
        if p.is_file() and p.name != 'files_manifest.json':
            allowed.append((p, p.relative_to(bundle)))
    manifest = [dict(path=str(rel), bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p, rel in sorted(allowed, key=lambda x: str(x[1]))]
    w.write_json(bundle / 'files_manifest.json', manifest)
    allowed.append((bundle / 'files_manifest.json', Path('files_manifest.json')))
    zpath = ROOT / '药房工作坊_Handle-L1_v4_2026-11-17.zip'
    with zipfile.ZipFile(zpath, 'w', zipfile.ZIP_DEFLATED) as z:
        for p, rel in allowed:
            z.write(p, Path('药房工作坊_Handle-L1_v4') / rel)
    print('Delivered', zpath, len(allowed), 'files', zpath.stat().st_size, flush=True)
    return zpath

def main():
    import sys
    if '--search' in sys.argv:
        search_grasp()
    elif '--delivery' in sys.argv:
        delivery()
    elif '--manual-check' in sys.argv:
        manual_check()
    elif '--web-check' in sys.argv:
        web_check()
    elif '--package' in sys.argv:
        package()
    elif '--sdk' in sys.argv:
        sdk_check()
    elif '--trial' in sys.argv:
        trial()
    elif '--render' in sys.argv:
        import grasp_pipeline as g
        g.OUT = OUT
        g.render('full')
    elif '--build' in sys.argv:
        build()
if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        raise RuntimeError(f'Handle-L1 integration failed: {exc}') from exc
