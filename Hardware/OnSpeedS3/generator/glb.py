"""Board STEP -> GLB for the web viewer, adding simple bodies for the pressure sensors (no vendor 3D
models exist) and the enclosure STLs as optional separate meshes."""
import sys, numpy as np, trimesh, cascadio, pcbnew

step, out = sys.argv[1], sys.argv[2]
cascadio.step_to_glb(step, out + ".tmp.glb", tol_linear=0.04, tol_angular=0.4)
scene = trimesh.load(out + ".tmp.glb")
b = pcbnew.LoadBoard(sys.argv[3])
# KiCad STEP: x right, y up (= -board y), z up from the board bottom; units mm (cascadio keeps mm? check scale)
bb = scene.bounds
scale = 1000.0 if bb[1][0] - bb[0][0] < 1 else 1.0
top = 1.6


def box(sx, sy, sz, cx, cy, cz, color):
    m = trimesh.creation.box(extents=(sx, sy, sz))
    m.apply_translation((cx, cy, cz))
    m.visual.face_colors = color
    return m


for f in b.GetFootprints():
    fid = f.GetFPID().GetLibItemName().wx_str() if hasattr(f.GetFPID().GetLibItemName(), "wx_str") else str(f.GetFPID().GetLibItemName())
    x, y = pcbnew.ToMM(f.GetPosition().x), -pcbnew.ToMM(f.GetPosition().y)
    rot = f.GetOrientationDegrees()
    if "MS4525" in fid:
        parts = [box(10.2, 8.9, 6.5, 0, 0, top + 3.25, [235, 235, 235, 255])]
        for dy in (-2.05, 2.05):
            c = trimesh.creation.cylinder(radius=1.5, height=7.0)
            c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, (0, 1, 0)))
            c.apply_translation((-5.1 - 3.5, dy, top + 3.25)); c.visual.face_colors = [220, 220, 220, 255]
            parts.append(c)
    elif "SSC_SIP" in fid:
        parts = [box(10.0, 6.0, 9.0, 0, 1.0, top + 4.5, [30, 30, 30, 255])]
    else:
        continue
    m = trimesh.util.concatenate(parts)
    m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(rot), (0, 0, 1)))
    m.apply_translation((x, y, 0))
    if scale != 1.0:
        m.apply_scale(1 / scale)
    scene.add_geometry(m, node_name=f.GetReference())
scene.export(out)
print("GLB", out, scene.bounds)
