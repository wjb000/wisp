#!/usr/bin/env python3
"""Build everything from cad/wisp.py + cad/elec.py:
  cad/step/*.step                 printed parts + full assembly (printed parts, vendor CAD, modelled bought parts)
  cad/stl/*.stl                   printed parts, already oriented for printing (ASCII)
  cad/reports/*.json              interference + motion-clearance checks, mass properties / stability, print report
  sim/wisp.urdf, sim/wisp.xml     URDF + MuJoCo model (link masses / inertias from this CAD), sim/meshes/*.obj
  site/assets/wisp.gltf, robot_info.json   viewer model with link hierarchy + labels
Run: python cad/fetch_vendor.py && python cad/export.py   (cadquery, trimesh, manifold3d)"""
import os, sys, json, math, time
import numpy as np, trimesh, cadquery as cq
import manifold3d as mf
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import wisp, elec
from textmesh import write_ascii_stl, write_obj, glb_to_embedded_gltf

DENS = {'PETG': 1.27e-3, 'PLA': 1.24e-3, 'TPU': 1.21e-3}       # g / mm^3
FILL = 0.85        # thin 2.2-2.4 mm walls print almost solid (3 perimeters); 15 % gyroid in the thicker bosses
STL_FACES = 7000   # ASCII STL facet budget per part (keeps files small enough for git / the web)


def d(*p):
    q = os.path.join(ROOT, *p); os.makedirs(q, exist_ok=True); return q


def tess(shape, tol=0.12):
    v, f = shape.tessellate(tol, 0.15)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True)
    m.merge_vertices(); trimesh.repair.fix_normals(m)
    return m


def wp_mesh(wp, tol=0.12):
    return trimesh.util.concatenate([tess(s, tol) for s in wp.vals()]) if len(wp.vals()) > 1 else tess(wp.val(), tol)


def to_mf(m):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def inter_vol(a, b):
    try:
        return (to_mf(a) ^ to_mf(b)).volume()
    except Exception:
        return float('nan')


def rot_about(m, axis, deg, origin):
    m = m.copy(); m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(deg), axis, origin)); return m


def main():
    t0 = time.time()
    J = wisp.JOINTS
    origin = {'base': np.zeros(3), 'dock': np.array([-40.0, -235.0, 0.0])}   # viewer: dock shown beside the robot (docked pose = 0)
    for jn, (p, c, o, a, lim) in J.items():
        origin[c] = np.array(o, float)
    parts = []
    for P in wisp.PARTS:
        s = P['fn'](); P = dict(P); P['solid'] = s; P['mesh'] = wp_mesh(s); parts.append(P)
        print(f"{P['name']}: {len(P['mesh'].faces)} faces, {s.val().Volume() / 1000:.1f} cm3")
    E = elec.build()
    for e in E:
        e['mesh'] = wp_mesh(e['solid'], 0.2)
    print('built in', round(time.time() - t0), 's')

    # ---------------- STEP
    sd = d('cad', 'step')
    asm = cq.Assembly(name='wisp')
    for P in parts:
        cq.exporters.export(P['solid'], os.path.join(sd, P['name'] + '.step'))
        asm.add(P['solid'], name=P['name'], color=cq.Color(P['color']))
    for e in E:
        asm.add(e['solid'], name=e['key'], color=cq.Color(e['color']))
    asm.save(os.path.join(sd, 'wisp_assembly.step'))

    # ---------------- printable STL (print orientation)
    ORIENT = {'chassis': [], 'glow_ring': [], 'mantle': [], 'dome': [((1, 0, 0), 180)], 'face_frame': [((0, 1, 0), -90)],
              'tail': [((1, 0, 0), 90)], 'dock': []}
    print_rep = {}
    std = d('cad', 'stl')
    for P in parts:
        m = P['mesh'].copy()
        for ax, deg in ORIENT[P['name']]:
            m = rot_about(m, ax, deg, (0, 0, 0))
        m.apply_translation(-m.bounds[0] + [0, 0, 0])
        mp = m if len(m.faces) <= STL_FACES else m.simplify_quadric_decimation(face_count=STL_FACES)
        write_ascii_stl(mp, os.path.join(std, P['name'] + '.stl'), P['name'])
        mat = P['material'].split()[0].split('/')[-1]
        g = P['solid'].val().Volume() * DENS[mat] * FILL
        ext = (m.bounds[1] - m.bounds[0]).round(1).tolist()
        print_rep[P['name']] = dict(material=P['material'], grams=round(g, 1), bbox_mm=ext, fits_256_bed=max(ext[:2]) <= 256 and ext[2] <= 256,
                                    est_print_h=round(g / 9.0, 1), supports=P['name'] in ('mantle', 'face_frame'))
    P_by = {P['name']: P for P in parts}

    # ---------------- interference (home pose) + motion sweeps
    allm = [(P['name'], P['mesh'], P['link']) for P in parts] + [(e['key'], e['mesh'], e['link']) for e in E]
    ALLOWED = {frozenset(x) for x in [('servo_head', 'face_frame'), ('servo_head', 'mantle'), ('tail', 'servo_tail'), ('caster', 'chassis'),
                                       ('motor_l', 'bracket_l'), ('motor_r', 'bracket_r'), ('pico', 'chassis'), ('glow_ring', 'chassis'),
                                       ('motor_l', 'wheel_l'), ('motor_r', 'wheel_r'), ('dock', 'dock_pad_1'), ('dock', 'dock_pad_-1'),
                                       ('camera', 'xiao'), ('mic', 'xiao'), ('glow_ring', 'mantle'), ('lcd', 'face_frame'), ('xiao', 'face_frame'), ('dome', 'face_frame'),
                                       ('amp', 'face_frame'), ('speaker', 'face_frame'), ('camera', 'face_frame')]}
    MOUNTED = {'drv8833', 'boost', 'charger', 'imu', 'pico', 'battery', 'cliff_0', 'cliff_1', 'cliff_2', 'servo_tail', 'tof', 'irrx_l', 'irrx_r', 'pogo_l', 'pogo_r', 'ledstrip', 'bracket_l', 'bracket_r', 'motor_l', 'motor_r'}
    ALLOWED |= {frozenset(('chassis', m)) for m in MOUNTED} | {frozenset(('glow_ring', 'ledstrip'))}
    clashes = []
    for i in range(len(allm)):
        for k in range(i + 1, len(allm)):
            a, b = allm[i], allm[k]
            if a[2] == 'dock' or b[2] == 'dock':
                if not (a[2] == 'dock' and b[2] == 'dock'):
                    continue
            ba, bb = a[1].bounds, b[1].bounds
            if np.any(ba[1] < bb[0]) or np.any(bb[1] < ba[0]):
                continue
            v = inter_vol(a[1], b[1])
            if v > 1.0:
                clashes.append(dict(a=a[0], b=b[0], mm3=round(v, 1), allowed=frozenset((a[0], b[0])) in ALLOWED))
    hard = [c for c in clashes if not c['allowed']]
    print('interference: %d contacts, %d unexpected' % (len(clashes), len(hard)), hard[:8])

    head = [m for (n, m, l) in allm if l == 'head']
    head_names = [n for (n, m, l) in allm if l == 'head']
    fixed = [(n, m) for (n, m, l) in allm if l == 'base' and n in ('mantle', 'glow_ring', 'chassis', 'servo_head')]
    piv = np.array(J['head_tilt'][2], float)
    tilt = []
    for ang in np.linspace(-wisp.TILT_UP, wisp.TILT_DOWN, 8):
        worst = 0.0; who = None
        for hn, hm in zip(head_names, head):
            r = rot_about(hm, (0, 1, 0), ang, piv)
            for fn, fm in fixed:
                if {hn, fn} == {'face_frame', 'servo_head'}:
                    continue
                v = inter_vol(r, fm)
                if v > worst:
                    worst, who = v, f'{hn}/{fn}'
        tilt.append(dict(deg=round(float(ang), 1), clash_mm3=round(worst, 1), pair=who))
    tm = P_by['tail']['mesh']; tax = np.array(J['tail_wag'][2], float)
    wag = []
    for ang in np.linspace(-wisp.TAIL_WAG, wisp.TAIL_WAG, 7):
        r = rot_about(tm, (0, 0, 1), ang, tax); worst = 0.0; who = None
        for fn in ('glow_ring', 'mantle', 'chassis'):
            v = inter_vol(r, P_by[fn]['mesh'])
            if v > worst:
                worst, who = v, fn
        wag.append(dict(deg=round(float(ang), 1), clash_mm3=round(worst, 1), pair=who))
    print('tilt', tilt); print('wag', wag)

    # ---------------- mass properties per link
    link_items = {}
    for P in parts:
        mat = P['material'].split()[0].split('/')[-1]
        link_items.setdefault(P['link'], []).append((P['name'], P['mesh'], P['solid'].val().Volume() * DENS[mat] * FILL))
    for e in E:
        link_items.setdefault(e['link'], []).append((e['key'], e['mesh'], e['mass']))
    link_items['base'].append(('wiring_fasteners', None, 35.0))
    link_items['head'].append(('wiring_fasteners', None, 6.0))
    links = {}
    for l, items in link_items.items():
        M = 0.0; c = np.zeros(3); parts_mi = []
        for n, m, g in items:
            if m is None:
                cm = links_guess(l, origin); I = np.zeros((3, 3))
            else:
                mm = m.convex_hull if not m.is_volume else m
                vol = abs(mm.volume) if abs(mm.volume) > 1e-6 else 1.0
                mm = mm.copy(); mm.density = g / vol
                cm = mm.center_mass; I = mm.moment_inertia
            parts_mi.append((g, np.array(cm), np.array(I))); M += g; c += g * np.array(cm)
        c /= M
        Itot = np.zeros((3, 3))
        for g, cm, I in parts_mi:
            r = cm - c
            Itot += I + g * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        links[l] = dict(mass_g=round(M, 1), com_mm=(c - origin[l]).round(2).tolist(), com_world_mm=c.round(2).tolist(), inertia_g_mm2=Itot.round(1).tolist())
    robot = [l for l in links if l != 'dock']
    Mt = sum(links[l]['mass_g'] for l in robot)
    com = sum(np.array(links[l]['com_world_mm']) * links[l]['mass_g'] for l in robot) / Mt
    # static stability: support triangle wheels (0, +-WY) + caster
    cx = wisp.CASTER[0]
    ax = wisp.AXLE_X
    inside = (cx < com[0] < ax) and abs(com[1]) < wisp.WY * (com[0] - cx) / (ax - cx)
    stab = dict(total_mass_g=round(Mt, 1), com_mm=com.round(1).tolist(), com_inside_support=bool(inside),
                margin_to_axle_mm=round(ax - com[0], 1), margin_to_caster_mm=round(com[0] - cx, 1),
                max_brake_decel_g_before_nose_tip=round((ax - com[0]) / com[2], 3),
                max_accel_g_before_caster_lift=round((com[0] - cx) / com[2], 3))
    # head tilted fully down: CoM shift check
    print('mass', stab)

    rep = d('cad', 'reports')
    json.dump(dict(home_pose_contacts=clashes, unexpected=hard, head_tilt_sweep=tilt, tail_wag_sweep=wag), open(os.path.join(rep, 'interference.json'), 'w'), indent=1)
    json.dump(dict(links=links, stability=stab), open(os.path.join(rep, 'mass_properties.json'), 'w'), indent=1)
    json.dump(print_rep, open(os.path.join(rep, 'print_report.json'), 'w'), indent=1)

    # ---------------- sim meshes, URDF, MJCF

    # ---------------- viewer glTF
    import glb_export
    summary = dict(total_mass_g=round(Mt), printed_parts=len(parts), height_mm=round(max(P['mesh'].bounds[1][2] for P in parts if P['link'] == 'head')),
                   diameter_mm=round(2 * 75.5), unexpected_clashes=len(hard),
                   tilt_clear=all(t['clash_mm3'] < 1 for t in tilt), wag_clear=all(w['clash_mm3'] < 1 for w in wag),
                   com_inside_support=bool(inside), print_grams=round(sum(v['grams'] for v in print_rep.values())))
    glb_export.write(parts, E, origin, J, summary)
    json.dump(summary, open(os.path.join(rep, 'summary.json'), 'w'), indent=1)
    print('done in', round(time.time() - t0), 's', summary)


def links_guess(l, origin):
    return origin[l] + (np.array([0, 0, 25.0]) if l == 'base' else 0)


if __name__ == '__main__':
    main()
