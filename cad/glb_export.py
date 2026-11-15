"""site/assets/wisp.gltf (embedded buffer, link hierarchy, mm, z-up) + site/assets/robot_info.json
(every part with its layer group, label text, explode offset, colour; joints)."""
import os, json, numpy as np, trimesh
import wisp
from textmesh import glb_to_embedded_gltf
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'site', 'assets')

PART_TXT = {
    'dome': ('Head dome', 'One shell: hooded brow, round face window and the camera "third eye". Clear-smoke in production.'),
    'face_frame': ('Face frame', 'Inner frame that tilts with the head. Holds the eye screen, brain board and speaker.'),
    'mantle': ('Mantle', 'Upper body shell with the neck collar and head-servo saddle built in.'),
    'glow_ring': ('Light band', 'Translucent ring: mood light and night-light. The tail grows out of it.'),
    'chassis': ('Chassis', 'Drive tub: floor, motor saddles, battery cradle, sensor windows. Hides the wheels.'),
    'tail': ('Tail', 'Soft TPU tail on a micro servo: wags, perks up, droops.'),
    'dock': ('Dock', 'Crescent charger with two copper pads and an IR homing beacon.'),
}
GROUP = {'dome': 'shell', 'mantle': 'shell', 'glow_ring': 'shell', 'chassis': 'shell', 'tail': 'shell', 'face_frame': 'structure', 'dock': 'dock',
         'bracket': 'structure', 'motor': 'drivetrain', 'wheel': 'drivetrain', 'caster': 'drivetrain',
         'battery': 'power', 'charger': 'power', 'boost': 'power', 'pogo': 'power',
         'pico': 'electronics', 'xiao': 'electronics', 'drv8833': 'electronics', 'imu': 'electronics', 'amp': 'electronics',
         'tof': 'sensors', 'irrx': 'sensors', 'cliff': 'sensors', 'camera': 'sensors', 'mic': 'sensors',
         'lcd': 'expression', 'speaker': 'expression', 'ledstrip': 'expression', 'servo': 'expression'}
EXPLODE = {'dome': (0, 0, 95), 'face_frame': (0, 0, 50), 'mantle': (0, 0, 55), 'glow_ring': (0, 0, 30), 'chassis': (0, 0, 0), 'tail': (-45, 0, 40),
           'dock': (0, -40, 0), 'servo_head': (0, 0, 55), 'servo_tail': (0, 0, 30), 'ledstrip': (0, 0, 30),
           'wheel_l': (0, 45, 0), 'wheel_r': (0, -45, 0), 'motor_l': (0, 22, 8), 'motor_r': (0, -22, 8), 'bracket_l': (0, 22, 20), 'bracket_r': (0, -22, 20),
           'battery': (-10, 0, 25), 'caster': (0, 0, -25), 'cliff': (0, 0, -15), 'tof': (25, 0, 0), 'irrx': (22, 0, 0), 'pogo': (25, 0, 0)}
HEAD_E = (0, 0, 50)
BASE_E = (0, 0, 18)


def group_of(key):
    k = key.split(':')[-1]
    for g in (k, k.split('_')[0]):
        if g in GROUP:
            return GROUP[g]
    return 'dock' if k.startswith('dock') else 'electronics'


def explode_of(key, link):
    k = key.split(':')[-1]
    for g in (k, k.split('_')[0]):
        if g in EXPLODE:
            return EXPLODE[g]
    if link == 'dock':
        return EXPLODE['dock']
    return HEAD_E if link == 'head' else BASE_E


def write(parts, E, origin, J, summary):
    os.makedirs(OUT, exist_ok=True)
    sc = trimesh.Scene()
    parent = {c: p for jn, (p, c, o, a, lim) in J.items()}
    order = ['base', 'dock'] + [J[j][1] for j in J]
    for l in order:
        T = np.eye(4)
        if l in parent:
            T[:3, 3] = origin[l] - origin[parent[l]]
        if l == 'dock':
            T[:3, 3] = origin['dock']
        sc.graph.update(frame_from=('link:' + parent[l]) if l in parent else sc.graph.base_frame, frame_to='link:' + l, matrix=T)

    def put(name, mesh, link, faces):
        m = mesh.copy()
        if link != 'dock':
            m.apply_translation(-origin[link])
        if len(m.faces) > faces:
            m = m.simplify_quadric_decimation(face_count=faces)
        m.vertices = np.round(m.vertices, 2)
        sc.add_geometry(m, node_name=name, geom_name=name, parent_node_name='link:' + link)

    items, labels = {}, []
    LBL = {'dome': (28, 34, wisp.DOME_CZ + 34), 'mantle': (38, 52, 98), 'chassis': (40, -60, 30), 'glow_ring': (52, 54, 63),
           'tail': tuple(wisp.TAIL_PTS[-1]), 'face_frame': (25, -24, wisp.PIVOT[1] + 9), 'dock': (86, -45, 28)}
    for P in parts:
        key = 'part:' + P['name']
        put(key, P['mesh'], P['link'], 12000 if P['name'] in ('chassis', 'mantle', 'dome') else 4000)
        c = np.array(LBL[P['name']], float)
        loc = (c - (origin[P['link']] if P['link'] != 'dock' else 0)).tolist()
        t, s = PART_TXT[P['name']]
        items[key] = dict(title=t, desc=s, group=group_of(key), color=P['color'], explode=explode_of(key, P['link']), kind='printed',
                          material=P['material'], translucent=P['name'] in ('glow_ring',))
        labels.append(dict(key=key, node='link:' + P['link'], local=loc))
    seen = set()
    for e in E:
        key = 'elec:' + e['key']
        put(key, e['mesh'], e['link'], 1500)
        items[key] = dict(title=e['name'], desc=e['spec'], group=group_of(key), color=e['color'], explode=explode_of(key, e['link']),
                          kind=e['kind'], emissive=e['key'] in ('ledstrip', 'lcd'))
        if e.get('label') and e['name'] not in seen:
            seen.add(e['name'])
            c = e['mesh'].bounds.mean(0)
            labels.append(dict(key=key, node='link:' + e['link'], local=(c - (origin[e['link']] if e['link'] != 'dock' else 0)).tolist()))
    glb = os.path.join(OUT, 'wisp.glb')
    sc.export(glb)
    glb_to_embedded_gltf(glb, os.path.join(OUT, 'wisp.gltf')); os.remove(glb)
    info = dict(summary=summary,
                joints={jn: dict(node='link:' + c, axis=list(a), limits_deg=list(lim) if lim else None) for jn, (p, c, o, a, lim) in J.items()},
                items=items, labels=labels)
    json.dump(info, open(os.path.join(OUT, 'robot_info.json'), 'w'), indent=1)
    print('wrote wisp.gltf', os.path.getsize(os.path.join(OUT, 'wisp.gltf')) // 1024, 'kB')
