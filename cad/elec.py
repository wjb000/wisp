"""Bought parts placed in the assembly. Each entry is a CadQuery solid so the same geometry feeds the assembly STEP,
the interference check, the sim masses and the website viewer. kind = 'vendor' (manufacturer CAD), 'modelled'
(datasheet dimensions), or 'envelope' (keep-out volume only)."""
import os, math
import cadquery as cq
from wisp import *

HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(HERE, 'vendor_fetched')
_cache = {}


def vendor(name, file):
    p = os.path.join(VENDOR, file)
    if not os.path.exists(p):
        return None
    if name not in _cache:
        _cache[name] = cq.importers.importStep(p)
    return _cache[name]


def place(wp, rot=(), t=(0, 0, 0)):
    for axis, deg in rot:
        wp = wp.rotate((0, 0, 0), axis, deg)
    return wp.translate(t)


def n20_motor(sy):
    """Pololu micro metal gearmotor MP 6V 75:1 with extended shaft + magnetic encoder disc; output shaft points outward."""
    y_out = MOTOR['y_out']
    g = box(-6, 6, 0, 9.0, -5, 5)                               # gearbox (9 mm)
    m = box(-6, 6, 9.0, 24.0, -5, 5).edges('|Y').fillet(2.0)     # can (15 mm)
    e = cyl_y(0, 0, 6.0, 24.0, 29.5)                           # encoder board + magnet
    sh = cyl_y(0, 0, 1.5, -10.0, 0)                            # 3 mm D-shaft, 10 mm
    s = g.union(m).union(e).union(sh)
    if sy > 0:
        return s.mirror('XZ').translate((AXLE_X, y_out, AXLE_Z))
    return s.translate((AXLE_X, -y_out, AXLE_Z))


def wheel():
    t = cyl_y(0, 0, WHEEL_R, -WHEEL_W / 2, WHEEL_W / 2).cut(cyl_y(0, 0, WHEEL_R - 3.5, -5, 5))
    rim = cyl_y(0, 0, WHEEL_R - 3.5, -WHEEL_W / 2 + 1, WHEEL_W / 2 - 1).cut(cyl_y(0, 0, WHEEL_R - 6, -5, 5))
    hub = cyl_y(0, 0, 5.0, -WHEEL_W / 2, WHEEL_W / 2 + 1)
    spokes = box(-WHEEL_R + 5, WHEEL_R - 5, -1.5, 1.5, -1.6, 1.6).union(box(-1.6, 1.6, -1.5, 1.5, -WHEEL_R + 5, WHEEL_R - 5))
    return t.union(rim).union(hub).union(spokes)


def build():
    E = []
    fz = FLOOR_Z + FLOOR_T

    def add(key, name, solid, mass, link, kind, spec, bom=None, label=True, color='#555b66'):
        E.append(dict(key=key, name=name, solid=solid, mass=mass, link=link, kind=kind, spec=spec, bom=bom, label=label, color=color))

    for sy, s in ((1, 'l'), (-1, 'r')):
        add(f'motor_{s}', 'N20 gearmotor 75:1 + magnetic encoder', n20_motor(sy), 11.0, 'base', 'modelled',
            'Pololu 75:1 Micro Metal Gearmotor MP 6V, extended shaft, run at 5 V: ~0.8 m/s top speed on 60 mm wheels; 12 CPR magnetic encoder (x75 = 900 counts per wheel turn).',
            'motor', label=(sy > 0), color='#b8bcc4')
        bx = box(AXLE_X - 9, AXLE_X + 9, sy * (WY - WHEEL_W / 2 - 1.5), sy * (WY - WHEEL_W / 2 - 1.5 - 7.0), AXLE_Z - 5, AXLE_Z + 7.5).cut(cyl_y(0, AXLE_Z, 6.1, -60, 60) if False else box(AXLE_X - 6.1, AXLE_X + 6.1, -60, 60, AXLE_Z - 5.1, AXLE_Z + 5.1))
        add(f'bracket_{s}', 'Micro metal gearmotor bracket', bx, 1.5, 'base', 'modelled', 'Pololu bracket pair: two M2 screws each into the chassis saddles.', 'bracket', label=False, color='#20242a')
    add('wheel_l', 'Wheel 60 x 8 mm', wheel().translate((AXLE_X, WY, AXLE_Z)), 3.5, 'wheel_l', 'modelled',
        'Pololu 60 x 8 mm wheel pair, silicone tyre, press-fit on the 3 mm D-shaft. Wheels sit inside the body: Wisp glides, no visible wheels.', 'wheels', color='#1b1d21')
    add('wheel_r', 'Wheel 60 x 8 mm', wheel().translate((AXLE_X, -WY, AXLE_Z)), 3.5, 'wheel_r', 'modelled', '', 'wheels', label=False, color='#1b1d21')
    add('battery', 'Li-ion pack 3.7 V 4400 mAh (2 x 18650)', box(BATT['x0'], BATT['x1'], -BATT['y'], BATT['y'], BATT['z0'], BATT['z1']), 95.0, 'base', 'modelled',
        '16 Wh, with protection PCB. Stands on edge behind the axle to keep the centre of mass low and between the wheels and the caster. ~3 h roaming, ~10 h idle-watching (estimate).',
        'battery', color='#3b6fb6')
    pico = vendor('pico', 'Pico-R3.step')
    if pico is not None:
        ps = pico.rotate((0, 0, 0), (1, 0, 0), 90).rotate((0, 0, 0), (0, 0, 1), 90)
        bb = ps.val().BoundingBox()
        ps = ps.translate((-bb.center.x, -bb.center.y, -bb.zmin + 0.5))       # board underside on the standoffs
        ps = ps.translate((30.5, 0, fz + 4.0)); kind = 'vendor'
    else:
        ps = box(5, 56, -10.5, 10.5, fz + 4.0, fz + 5.0); kind = 'modelled'
    add('pico', 'Raspberry Pi Pico (body controller)', ps, 3.0, 'base', kind,
        'The "spinal cord": 1 kHz wheel speed loops, odometry, cliff + bump reflexes, IR dock homing, servos, light band. Talks to the head over UART at 921600 baud.',
        'pico', color='#2f8f4e')
    add('drv8833', 'DRV8833 dual motor driver', box(30, 50.3, -6.4, 6.3, fz + 13, fz + 14.6), 1.0, 'base', 'modelled', '2 x 1.2 A H-bridges, PWM from the Pico; rides on top of the Pico.', 'drv8833', color='#1f5fa8')
    add('boost', '5 V 4 A boost regulator', box(26, 41.2, -40, -17.1, fz + 4, fz + 7.0), 2.0, 'base', 'modelled', 'Battery (3.0-4.2 V) to 5 V for motors, servos, LEDs and both boards.', 'boost', color='#1f5fa8')
    add('charger', 'bq24074 charger / power path', box(-34, 2, -11, 11, BATT['z1'] + 1.5, BATT['z1'] + 3.5), 3.0, 'base', 'modelled',
        '1.5 A Li-ion charging from the dock pins or rear USB-C; runs the robot while charging. Sits on a bridge over the battery.', 'charger', color='#1a1a1a')
    tof = box(66.5, 69.0, -9, 9, 39.5, 52.5).rotate((67.75, 0, 46), (67.75, 1, 46), 5)
    add('tof', 'VL53L7CX 8 x 8 time-of-flight sensor', tof, 0.5, 'base', 'modelled',
        '64-zone depth at 15-60 Hz, 90 deg field of view, 4 m: obstacle and drop-off map in front of the robot, tilted 5 deg down.', 'tof', color='#7a2fa0')
    for sy in (1, -1):
        a = sy * 25
        ir = box(-2.5, 2.5, -3, 3, -3.5, 3.5).union(cyl_x(0, 0, 2.6, 2.5, 4.5)).rotate((0, 0, 0), (0, 0, 1), a).translate((69 * math.cos(math.radians(a)), 69 * math.sin(math.radians(a)), 48))
        add(f'irrx_{"l" if sy > 0 else "r"}', 'TSOP38238 IR receiver (x2)', ir, 0.5, 'base', 'modelled', '38 kHz receivers, +-25 deg: read the dock\'s left / right beam codes to home in.', 'irrx', label=(sy > 0), color='#111111')
    for i, (x, y) in enumerate(CLIFF):
        add(f'cliff_{i}', 'QTR-1RC cliff sensor (x3)', box(x - 3.8, x + 3.8, y - 6.3, y + 6.3, fz - 1.6, fz), 0.3, 'base', 'modelled',
            'IR reflectance under the floor: a drop under any sensor stops the wheels in < 5 ms (Pico reflex, not the brain).', 'cliff', label=(i == 0), color='#1f5fa8')
    add('imu', 'LSM6DS3TR-C 6-axis IMU', box(-17, 1, -8.9, 8.9, fz + 4, fz + 5.6), 1.0, 'base', 'modelled', 'Heading for odometry, bump / pick-up / tip detection ("put me down!").', 'imu', color='#1a1a1a')
    for sy in (1, -1):
        add(f'pogo_{"l" if sy > 0 else "r"}', 'Pogo charging contacts (x2)', cyl_x(sy * POGO_Y, POGO_Z, 1.0, 62, 77.5), 0.3, 'base', 'modelled',
            'Spring pins at the front skirt meet two copper pads in the dock: 5 V in.', 'pogo', label=(sy > 0), color='#d4a017')
    add('caster', 'Ball caster 3/8"', cyl_z(*CASTER, 7.0, 4.8, FLOOR_Z + 2).union(cq.Workplane('XY').sphere(4.76).translate((CASTER[0], CASTER[1], 4.76))), 2.0, 'base', 'modelled',
        'Rear support point. Plastic ball.', 'caster', color='#888888')
    sg = box(-64.1, -41.1, -6.1, 6.1, 34.0, 56.5).union(box(-68.4, -36.8, -6.1, 6.1, 49.0, 51.5)).union(cyl_z(TAIL_AXIS[0], 0, 2.5, 56.5, 60.5)).union(cyl_z(TAIL_AXIS[0], 0, 3.5, 60.5, 62.4))
    add('servo_tail', 'Tail servo SG92R', sg, 9.0, 'base', 'modelled', '+-35 deg wag at up to 4 Hz: happy, curious, sleepy, alert.', 'servo_tail', color='#2b5fb0')
    ring = cyl_z(0, 0, 73.4, 60.0, 66.0).cut(cyl_z(0, 0, 71.4, 59, 67)).cut(box(-80, -60, -20, 20, 59, 67))
    add('ledstrip', 'Light band: 13 RGB LEDs', ring, 5.0, 'base', 'modelled',
        'Addressable LEDs behind the translucent band: mood, battery, listening, and a soft night-light.', 'ledstrip', color='#ffb347')
    pz = PIVOT[1]
    mg = box(-17.1, 5.9, 2.5, 25.0, pz - 6.1, pz + 6.1).union(box(-21.4, 10.2, 14.5, 17.0, pz - 6.1, pz + 6.1)).union(cyl_y(0, pz, 2.5, 25.0, 28.4))
    add('servo_head', 'Head tilt servo MG92B (metal gear)', mg, 14.0, 'base', 'modelled', 'Nods the head -20/+15 deg: looks up at you, down at the floor or the cat.', 'servo_head', color='#2b5fb0')
    # ---- head
    add('lcd', '1.28" round IPS LCD (240 x 240)', cyl_x(0, FACE_Z, 18.5, LCD_X, LCD_X + 1.6).union(cyl_x(0, FACE_Z, 17.0, LCD_X + 1.6, LCD_X + 4.6)), 6.0, 'head', 'modelled',
        'GC9A01 round display: the eye. Iris, lids and glyphs are drawn by the head at 30 fps.', 'lcd', color='#0b0d10')
    xiao = vendor('xiao', 'xiao_esp32s3.step')
    if xiao is not None:
        bb = xiao.val().BoundingBox()
        xs = xiao.translate((-bb.center.x, -bb.ymin, -bb.center.z)).rotate((0, 0, 0), (0, 0, 1), 90).rotate((0, 0, 0), (1, 0, 0), 90)
        bx = xs.val().BoundingBox(); xs = xs.translate((39.0 - bx.xmin, -bx.center.y, CAM_Z - 8 - bx.center.z)); kind = 'vendor'
    else:
        xs = box(39, 44, -9, 9, CAM_Z - 14, CAM_Z + 7); kind = 'modelled'
    add('xiao', 'XIAO ESP32-S3 Sense (head brain)', xs, 6.0, 'head', kind,
        'Dual-core 240 MHz + 8 MB PSRAM: personality engine, on-device wake word (ESP-SR), person / face detection (ESP-DL), the eye animation, Wi-Fi check-in video.',
        'xiao', color='#2c2c2c')
    cam = box(42.0, 43.5, -4.5, 4.5, CAM_Z - 4.5, CAM_Z + 4.5).union(cyl_x(0, CAM_Z, 3.9, 43.5, 44.6))
    add('camera', 'OV3660 camera (on the XIAO Sense)', cam, 1.0, 'head', 'modelled', '3 MP, sits behind the brow lens hole; tilts with the head.', 'xiao', color='#111111')
    add('amp', 'MAX98357A I2S amplifier', box(36, 38, -9, 9, pz + 12, pz + 30), 1.0, 'head', 'modelled', '3 W class-D from the head\'s I2S port.', 'amp', color='#1a1a1a')
    add('speaker', 'Speaker 30 x 20 mm, 1 W', box(12, 32, -15, 15, pz + 10, pz + 15), 4.0, 'head', 'modelled', 'Chirps, purrs and a speaking voice; fires through the chin slots.', 'speaker', color='#444444')
    add('mic', 'PDM microphone (on the XIAO Sense)', box(40, 41, 8, 11, CAM_Z - 3, CAM_Z), 0.1, 'head', 'modelled', 'Wake word and sound events (glass break, dog bark: designed, not trained).', 'xiao', label=False, color='#999999')
    # ---- dock electronics (dock is shown pushed forward by DOCK_SHOW)
    add('dock_mcu', 'Dock beacon: XIAO ESP32-C3', box(R_DOCK + 3.5, R_DOCK + 5.5, -9, 9, 5, 22), 1.0, 'dock', 'modelled', 'Pulses two 940 nm IR LEDs with left / right codes on a 38 kHz carrier, and powers the pads.', 'dock_mcu', color='#2c2c2c')
    for sy in (1, -1):
        add(f'dock_led_{sy}', 'IR LED 940 nm (x2)', cyl_x(sy * 4.5, 29, 2.5, R_DOCK + 2.5, R_DOCK + 7), 0.3, 'dock', 'modelled', 'Split by the baffle so the beams only overlap on the centre line.', 'dock_led', label=(sy > 0), color='#552255')
        add(f'dock_pad_{sy}', 'Copper contact pads (x2)', box(R_DOCK - 0.95, R_DOCK + 0.55, sy * POGO_Y - 4.4, sy * POGO_Y + 4.4, POGO_Z - 4.9, POGO_Z + 4.9), 0.5, 'dock', 'modelled',
            '5 V from a USB-C supply through the dock to the robot\'s pogo pins.', 'dock_pad', label=(sy > 0), color='#c87533')
    add('dock_usb', 'USB-C inlet', box(R_DOCK + 8, R_DOCK + 13, -4.5, 4.5, 4.2, 7.8), 0.5, 'dock', 'modelled', 'Any 5 V / 2 A USB-C supply.', 'dock_usb', label=False, color='#777777')
    return E


R_DOCK = 78.5
