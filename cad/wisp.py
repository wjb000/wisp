"""Wisp: parametric CadQuery model of every printed part.
Frame: x forward, y left, z up, millimetres, floor at z = 0. All printed parts are built in assembly position;
export.py re-orients them for printing. Change a number here and re-run `python cad/export.py`.

Printed parts (7): chassis, glow_ring, mantle, dome, face_frame, tail, dock."""
import math
import cadquery as cq

# ---------------------------------------------------------------- key dimensions
WHEEL_R, WHEEL_W = 30.0, 8.0          # Pololu 60 x 8 mm wheel (item 1420)
AXLE_Z = WHEEL_R
AXLE_X = 12.0                         # axle ahead of centre: CoM sits well behind it (no nose-dive when braking)
TRACK = 96.0                          # wheel centre to wheel centre
WY = TRACK / 2
FLOOR_Z, FLOOR_T = 8.0, 2.4           # chassis floor underside / thickness (8 mm ground clearance)
WALL = 2.4
CHASSIS_TOP = 58.0
RING_Z0, RING_Z1 = 58.0, 68.0         # translucent light band
MANTLE_TOP = 122.0                    # neck collar plane = head pivot height
NECK_R = 50.0                         # neck opening radius in the mantle collar
PIVOT = (0.0, MANTLE_TOP)             # head tilt axis: along y through (x=0, z=122)
DOME_R, DOME_RZ = 58.0, 56.0          # head ellipsoid (radius in x/y, half-height)
DOME_RIM_R = 40.0                     # head rim radius (sits inside the neck opening)
DOME_CZ = MANTLE_TOP + DOME_RZ * math.sqrt(1 - (DOME_RIM_R / DOME_R) ** 2)
DOME_T = 2.2
FACE_R = 21.0                         # face window radius (1.28" round LCD, 32.4 mm active area)
FACE_Z = DOME_CZ - 4.0                # face window centre height
CAM_Z = FACE_Z + 33.0                 # camera lens hole centre height ("brow eye")
LCD_X = 46.0                          # back face of the round LCD module
TILT_DOWN, TILT_UP = 9.0, 10.0       # head tilt limits (deg) - checked for clearance in export.py
TAIL_AXIS = (-47.0, 0.0)              # tail servo output shaft (vertical)
TAIL_WAG = 35.0
CASTER = (-56.0, 0.0)
BATT = dict(x0=-39.0, x1=-20.5, y=34.5, z0=FLOOR_Z + FLOOR_T, z1=FLOOR_Z + FLOOR_T + 37.0)   # Adafruit 354 pack on edge
MOTOR = dict(w=12.0, h=10.0, y_out=WY - WHEEL_W / 2 - 2.0, length=29.5)                       # N20 + encoder
CLIFF = [(56.0, 30.0), (56.0, -30.0), (-60.0, 28.0)]
POGO_Z, POGO_Y = 16.0, 12.0
DOCK_SHOW = 70.0                      # viewer: dock drawn this far in front of the robot (docked = 0)

# outer radius profiles (r, z) - smooth splines
CHASSIS_PROF = [(60.0, FLOOR_Z), (68.0, FLOOR_Z + 2.0), (72.5, 16.0), (75.0, 30.0), (75.5, 45.0), (75.5, CHASSIS_TOP)]
MANTLE_PROF = [(75.5, RING_Z1), (74.5, 82.0), (71.0, 96.0), (64.0, 108.0), (57.0, 116.5), (NECK_R + 3.0, MANTLE_TOP)]


def _offset(prof, t):
    """Inward normal offset of an (r, z) polyline."""
    out = []
    n = len(prof)
    for i, (r, z) in enumerate(prof):
        a = prof[max(i - 1, 0)]; b = prof[min(i + 1, n - 1)]
        dr, dz = b[0] - a[0], b[1] - a[1]; L = math.hypot(dr, dz)
        nr, nz = dz / L, -dr / L          # rotate tangent -90deg: points inward for an upward-going profile
        out.append((r - t * nr, z - t * nz))
    return out


def _revolve_wall(prof, t, close_bottom=False, bottom_t=0.0):
    inner = _offset(prof, t)
    w = cq.Workplane('XZ').moveTo(*prof[0]).spline(prof[1:], includeCurrent=True)
    w = w.lineTo(*inner[-1]).spline(list(reversed(inner[:-1])), includeCurrent=True).close()
    s = w.revolve(360, (0, 0, 0), (0, 1, 0))
    if close_bottom:
        s = s.union(cq.Workplane('XY').workplane(offset=prof[0][1]).circle(inner[0][0] + 0.5).extrude(bottom_t))
    return s


def _ellipse_solid(r, rz, cz):
    return (cq.Workplane('XZ').moveTo(0, cz - rz).ellipseArc(r, rz, -90, 90, startAtCurrent=True)
            .close().revolve(360, (0, 0, 0), (0, 1, 0)))


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Workplane('XY').box(x1 - x0, y1 - y0, z1 - z0).translate(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2))


def cyl_y(x, z, r, y0, y1):
    return cq.Workplane('XZ', origin=(0, y1, 0)).center(x, z).circle(r).extrude(y1 - y0)


def cyl_z(x, y, r, z0, z1):
    return cq.Workplane('XY', origin=(0, 0, z0)).center(x, y).circle(r).extrude(z1 - z0)


def cyl_x(y, z, r, x0, x1):
    return cq.Workplane('YZ', origin=(x0, 0, 0)).center(y, z).circle(r).extrude(x1 - x0)


# ---------------------------------------------------------------- 1. chassis (drive tub)
def chassis():
    s = _revolve_wall(CHASSIS_PROF, WALL, close_bottom=True, bottom_t=FLOOR_T)
    fz = FLOOR_Z + FLOOR_T
    # motor saddles (Pololu 989 brackets screw to these), one each side
    for sy in (1, -1):
        y0, y1 = sorted((sy * (WY - WHEEL_W / 2 - 1.0), sy * (WY - WHEEL_W / 2 - 1.0 - 22.0)))
        s = s.union(box(AXLE_X - 9, AXLE_X + 9, y0, y1, fz, AXLE_Z - MOTOR['h'] / 2))
    # battery cradle rails, caster boss, Pico / power-board standoffs, tail servo pillar, mantle screw bosses
    s = s.union(box(BATT['x0'] - 2.4, BATT['x0'], -36, 36, fz, fz + 14)).union(box(BATT['x1'], BATT['x1'] + 2.4, -36, 36, fz, fz + 14))
    for (x, y) in [(6, 8.5), (6, -8.5), (53, 8.5), (53, -8.5), (29, -20), (38, -37)]:
        s = s.union(cyl_z(x, y, 2.6, fz, fz + 4.0))
    s = s.union(box(-66, -58, -9, 9, fz, 34))                     # tail servo pillar (SG92R tabs screw on top)
    for a in (45, 135, 225, 315):
        x, y = 70 * math.cos(math.radians(a)), 70 * math.sin(math.radians(a))
        s = s.union(cyl_z(x, y, 3.2, fz, CHASSIS_TOP).cut(cyl_z(x, y, 1.1, fz + 6, CHASSIS_TOP + 1)))
    # cut-outs: wheel slots, caster hole, cliff windows, ToF + IR windows, pogo holes, USB-C service port, motor pockets
    for sy in (1, -1):
        s = s.cut(box(AXLE_X - 32, AXLE_X + 32, sy * WY - 5.5, sy * WY + 5.5, 0, AXLE_Z + WHEEL_R + 2))
        y_in = sy * (WY - WHEEL_W / 2 - 2.0 - MOTOR['length'])
        s = s.cut(box(AXLE_X - MOTOR['w'] / 2 - .2, AXLE_X + MOTOR['w'] / 2 + .2, min(sy * MOTOR['y_out'], y_in), max(sy * MOTOR['y_out'], y_in), AXLE_Z - MOTOR['h'] / 2 - .2, AXLE_Z + MOTOR['h'] / 2 + .2))
    s = s.cut(cyl_z(*CASTER, 9.0, 0, fz + 1))
    for (x, y) in CLIFF:
        s = s.cut(box(x - 4, x + 4, y - 6.5, y + 6.5, 0, fz + 1))
    s = s.cut(box(60, 80, -10, 10, 40, 52))                       # VL53L7CX window (90 deg FOV)
    for sy in (1, -1):
        a = math.radians(sy * 25); x, y = 74 * math.cos(a), 74 * math.sin(a)
        s = s.cut(cyl_x(0, 0, 3.4, -6, 6).rotate((0, 0, 0), (0, 0, 1), sy * 25).translate((x, y, 48)))
        s = s.cut(cyl_x(sy * POGO_Y, POGO_Z, 1.1, 60, 80))
    s = s.cut(box(-80, -66, 22, 34, 14, 24))                       # USB-C + power switch service port (rear left)
    return s


# ---------------------------------------------------------------- 2. glow ring (translucent band, LED strip inside)
def glow_ring():
    s = cyl_z(0, 0, 75.5, RING_Z0, RING_Z1).cut(cyl_z(0, 0, 73.5, RING_Z0 - 1, RING_Z1 + 1))
    s = s.union(cyl_z(0, 0, 73.5, RING_Z0, RING_Z0 + 1.6).cut(cyl_z(0, 0, 69.0, RING_Z0 - 1, RING_Z1)))   # ledge sits on chassis rim
    # tail slot at the back (tail wags +-35 deg about its own shaft)
    s = s.cut(box(-80, -64, -25, 25, RING_Z0 + 2.0, RING_Z1 + 1))
    return s


# ---------------------------------------------------------------- 3. mantle (upper shell + neck collar + head-servo post)
def mantle():
    s = _revolve_wall(MANTLE_PROF, WALL)
    s = s.cut(cyl_z(0, 0, NECK_R, MANTLE_TOP - 6, MANTLE_TOP + 1))
    s = s.union(cyl_z(0, 0, NECK_R + 3.0, MANTLE_TOP - 3, MANTLE_TOP).cut(cyl_z(0, 0, NECK_R, MANTLE_TOP - 4, MANTLE_TOP + 1)))
    # head-servo post: a bridge across the neck carrying the MG92B (on its side, output along +y at the pivot)
    s = s.union(box(-4.5, 4.5, -NECK_R - 2, NECK_R + 2, MANTLE_TOP - 12, MANTLE_TOP - 9).intersect(cyl_z(0, 0, NECK_R + 2, 0, 200)))
    s = s.union(box(-20, 9, -2, 26, MANTLE_TOP - 9.0, MANTLE_TOP - 6.2))                    # servo saddle
    s = s.union(box(-20, -17, -2, 26, MANTLE_TOP - 6.2, MANTLE_TOP + 4))                    # saddle end wall (screw tab)
    s = s.union(box(-4, 4, -37, -32, MANTLE_TOP - 9.0, PIVOT[1] + 6).union(cyl_y(0, PIVOT[1], 4, -37, -32)))  # idler ear (right)
    s = s.cut(cyl_y(0, PIVOT[1], 1.6, -40, -28))                                            # M3 pivot screw hole
    # speaker-free mantle: tail exit notch at the back (continues the glow-ring slot), side vents for the boost converter
    s = s.cut(box(-80, -60, -25, 25, RING_Z1 - 1, RING_Z1 + 10))
    for i in range(5):
        z = 84 + 4.2 * i
        s = s.cut(box(-6, 6, -80, -60, z, z + 2.0).rotate((0, 0, 0), (0, 0, 1), -60))
    return s


# ---------------------------------------------------------------- 4. dome (head shell, smoked-translucent)
def dome():
    out = _ellipse_solid(DOME_R, DOME_RZ, DOME_CZ)
    inn = _ellipse_solid(DOME_R - DOME_T, DOME_RZ - DOME_T, DOME_CZ)
    s = out.cut(inn).cut(box(-100, 100, -100, 100, 0, MANTLE_TOP + 0.6))
    # visor brim over the face: a soft hood that gives the head its silhouette
    brim = _ellipse_solid(DOME_R + 5.0, DOME_RZ + 4.0, DOME_CZ).cut(out)
    brim = brim.intersect(box(18, 80, -48, 48, FACE_Z + FACE_R + 2, FACE_Z + FACE_R + 9))
    s = s.union(brim)
    s = s.cut(cyl_x(0, FACE_Z, FACE_R, 20, 80))                      # face window (round LCD)
    s = s.cut(cyl_x(0, CAM_Z, 4.2, 20, 80))                          # camera lens hole above the brim
    for sy in (1, -1):                                               # mic pin-holes + speaker slots under the chin
        s = s.cut(cyl_x(sy * 10, CAM_Z - 1, 0.7, 20, 80))
    for i in range(4):
        s = s.cut(box(10, 60, -14 + 8 * i, -10 + 8 * i, MANTLE_TOP + 4, MANTLE_TOP + 9).rotate((0, 0, 0), (0, 1, 0), 0))
    # two clip lugs that snap the dome onto the face frame
    for sy in (1, -1):
        s = s.union(box(16, 24, sy * 30 - 3, sy * 30 + 3, MANTLE_TOP + 1, MANTLE_TOP + 7).intersect(inn))
    return s


# ---------------------------------------------------------------- 5. face frame (rotates with the head; holds LCD, XIAO, amp, speaker)
def face_frame():
    pz = PIVOT[1]
    s = cyl_x(0, FACE_Z, 22, LCD_X - 3, LCD_X)                                                      # LCD back plate
    s = s.union(box(LCD_X - 3, LCD_X, -12, 12, FACE_Z, FACE_Z + 18))                               # web up to the camera mount
    s = s.union(box(36, 39, -12, 12, CAM_Z - 13, CAM_Z + 5))                                      # XIAO ESP32S3 Sense mount plate
    s = s.union(box(8, 38, -30, 30, pz + 7, pz + 10).intersect(cyl_z(0, 0, 39, 0, 300)))           # floor plate (speaker sits on it)
    for sy in (1, -1):
        s = s.union(box(-8, 8, sy * 30 - 1.5, sy * 30 + 1.5, pz - 6, pz + 6)).union(box(-2, 10, sy * 30 - 1.5, sy * 30 + 1.5, pz, pz + 10))  # pivot cheeks
        s = s.union(cyl_y(0, pz, 6.5, sy * 28.5, sy * 31.5) if sy > 0 else cyl_y(0, pz, 6.5, -31.5, -28.5))
        s = s.union(box(8, LCD_X - 1, sy * 20 - 1.5, sy * 20 + 1.5, pz + 7, FACE_Z + 4))         # side ribs up to the LCD plate
        s = s.union(box(30, 39, sy * 13.5 - 1.5, sy * 13.5 + 1.5, FACE_Z + 10, CAM_Z - 10))       # posts to the XIAO plate
    s = s.cut(cyl_y(0, pz, 1.6, -40, 40))
    s = s.cut(box(LCD_X - 4, LCD_X + 1, -6, 6, FACE_Z - 6, FACE_Z + 6))                             # LCD connector pass-through
    return s


# ---------------------------------------------------------------- 6. tail (TPU 95A, flexible whip)
TAIL_PTS = [(TAIL_AXIS[0], 0, 66.0), (-72.0, 0, 64.5), (-92.0, 0, 70.0), (-104.0, 0, 88.0), (-106.0, 0, 110.0), (-99.0, 0, 128.0)]


def tail():
    path = cq.Workplane('XY').spline(TAIL_PTS)
    p0 = cq.Vector(*TAIL_PTS[0]); p1 = cq.Vector(*TAIL_PTS[1])
    plane = cq.Plane(origin=p0, xDir=(0, 1, 0), normal=(p1 - p0).normalized())
    s = cq.Workplane(plane).circle(4.0).sweep(path)
    s = s.union(cq.Workplane('XY').sphere(7.5).translate(TAIL_PTS[-1]))
    s = s.union(cyl_z(TAIL_AXIS[0], 0, 7.0, 62.5, 69.5).union(box(TAIL_AXIS[0] - 1, -66, -5, 5, 62.5, 69.5)))   # horn clamp hub
    s = s.cut(cyl_z(TAIL_AXIS[0], 0, 2.6, 60, 72))
    return s


# ---------------------------------------------------------------- 7. dock (crescent charger, IR beacon)
DOCK_X = 96.0       # dock contact face x when the robot is docked (front of robot at x = 75.5 + pogo 2 mm)


def dock():
    R_in, R_out = 78.5, 92.0
    s = cyl_z(0, 0, R_out, 0, 38).cut(cyl_z(0, 0, R_in, -1, 39)).intersect(box(20, 120, -70, 70, 0, 38))
    s = s.union(cyl_z(0, 0, 120, 0, 3).cut(cyl_z(0, 0, R_in - 14, -1, 4)).intersect(box(40, 120, -70, 70, 0, 3)))   # floor plate / ramp
    s = s.edges('|Z').fillet(1.0) if False else s
    s = s.cut(cyl_z(0, 0, R_out - 2.0, 3.0, 35.5).cut(cyl_z(0, 0, R_in + 2.0, 0, 40)).intersect(box(44, 120, -60, 60, 0, 40)))   # hollow shell
    for sy in (1, -1):
        s = s.cut(box(R_in - 1.0, R_in + 0.6, sy * POGO_Y - 4.5, sy * POGO_Y + 4.5, POGO_Z - 5, POGO_Z + 5))      # copper contact pads
    s = s.cut(box(R_in + 2, R_out + 1, -9, 9, 24, 34))                 # IR window (two 940 nm LEDs behind a split baffle)
    s = s.union(box(R_in + 1, R_out - 2, -0.8, 0.8, 22, 36))           # baffle: left/right beams overlap only on the centre line
    s = s.cut(box(R_out - 6, R_out + 2, -5, 5, 4, 8))                  # USB-C inlet
    return s


PARTS = [  # name, builder, material/colour, link, print notes
    dict(name='chassis', fn=chassis, color='#2a2e35', link='base', material='PETG (graphite)'),
    dict(name='glow_ring', fn=glow_ring, color='#ffe2b8', link='base', material='PETG (natural/translucent)'),
    dict(name='mantle', fn=mantle, color='#efe9df', link='base', material='PLA/PETG (bone white)'),
    dict(name='dome', fn=dome, color='#f4efe6', link='head', material='PETG (bone white; clear-smoke when moulded)'),
    dict(name='face_frame', fn=face_frame, color='#16191e', link='head', material='PETG (black)'),
    dict(name='tail', fn=tail, color='#ff7a45', link='tail', material='TPU 95A (coral)'),
    dict(name='dock', fn=dock, color='#2a2e35', link='dock', material='PETG (graphite)'),
]

JOINTS = {  # name: (parent, child, origin xyz, axis, (lo, hi) deg or None)
    'wheel_l': ('base', 'wheel_l', (AXLE_X, WY, AXLE_Z), (0, 1, 0), None),
    'wheel_r': ('base', 'wheel_r', (AXLE_X, -WY, AXLE_Z), (0, 1, 0), None),
    'head_tilt': ('base', 'head', (0, 0, PIVOT[1]), (0, 1, 0), (-TILT_UP, TILT_DOWN)),     # +y rotation = nose down
    'tail_wag': ('base', 'tail', (TAIL_AXIS[0], 0, 66.0), (0, 0, 1), (-TAIL_WAG, TAIL_WAG)),
}
