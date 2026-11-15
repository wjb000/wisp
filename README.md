# Wisp: autonomous home pet robot (design model)

**3D viewer:** https://wjb000.github.io/wisp/

Wisp is a lantern-shaped pet robot that glides on hidden wheels. It has a tilting hooded head with a round screen eye and a camera, a glowing light band, a wagging soft tail, and a crescent dock it finds by infrared to charge itself.

This repo is a **design model for review**. Nothing has been built or tested.

| | |
|---|---|
| Size | 219 mm tall, 151 mm wide, about 0.54 kg (estimated from CAD) |
| Printed parts (7) | chassis, glow_ring, mantle, dome, face_frame, tail (TPU), dock |
| Brains | XIAO ESP32-S3 Sense in the head (camera, mic, wake word, personality), Raspberry Pi Pico in the base (motors, sensors, docking) |
| Sensors | VL53L7CX 8x8 time-of-flight, 3 cliff sensors, 2 IR dock receivers, 6-axis IMU, camera, microphone |
| Motion | 2 N20 encoder gearmotors (hidden 60 mm wheels), head-tilt servo, tail servo |

## Files
- `cad/wisp.py`: parametric CadQuery model of every printed part. Edit the numbers at the top.
- `cad/elec.py`: bought parts placed inside the robot. The Pico and XIAO use the makers' STEP files (`python cad/fetch_vendor.py`). Everything else is modelled from datasheet dimensions.
- `cad/export.py`: builds STEP/STL, the viewer glTF, and interference, head-tilt and tail-wag clearance plus mass checks (`cad/reports/`).
- `cad/stl/`: printed parts as ASCII STL, oriented for printing.
- `site/`: the three.js viewer (layers, labels, X-ray, explode, standard views).

Rebuild: `pip install cadquery trimesh manifold3d && python cad/fetch_vendor.py && python cad/export.py`
