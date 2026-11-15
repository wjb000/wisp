"""Text mesh writers. The repo stores meshes as text (ASCII STL for printing, OBJ for the sim, glTF with an
embedded buffer for the website) so every file diffs and uploads cleanly. MuJoCo reads OBJ (it only reads binary STL)."""
import base64, json, struct
import numpy as np

def write_ascii_stl(mesh, path, name='wisp'):
    v = mesh.vertices[mesh.faces]; n = mesh.face_normals
    out = [f'solid {name}']
    for (a, b, c), k in zip(v, n):
        out.append(f'facet normal {k[0]:.4f} {k[1]:.4f} {k[2]:.4f}\nouter loop\n'
                   f'vertex {a[0]:.4f} {a[1]:.4f} {a[2]:.4f}\nvertex {b[0]:.4f} {b[1]:.4f} {b[2]:.4f}\nvertex {c[0]:.4f} {c[1]:.4f} {c[2]:.4f}\n'
                   'endloop\nendfacet')
    out.append(f'endsolid {name}\n')
    open(path, 'w').write('\n'.join(out))

def write_obj(mesh, path, digits=6):
    L = [f'v {x:.{digits}g} {y:.{digits}g} {z:.{digits}g}' for x, y, z in mesh.vertices]
    L += [f'f {a + 1} {b + 1} {c + 1}' for a, b, c in mesh.faces]
    open(path, 'w').write('\n'.join(L) + '\n')

def glb_to_embedded_gltf(glb_path, gltf_path):
    b = open(glb_path, 'rb').read()
    jlen = struct.unpack('<I', b[12:16])[0]
    j = json.loads(b[20:20 + jlen])
    off = 20 + jlen; blen = struct.unpack('<I', b[off:off + 4])[0]
    binc = b[off + 8: off + 8 + blen]
    j['buffers'] = [dict(byteLength=len(binc), uri='data:application/octet-stream;base64,' + base64.b64encode(binc).decode())]
    json.dump(j, open(gltf_path, 'w'), separators=(',', ':'))
