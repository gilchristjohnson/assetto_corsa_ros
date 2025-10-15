"""In-game Assetto Corsa plugin that surfaces telemetry and sends UDP packets."""

import ac
import acsys
import os
import sys

def _configure_windows_paths():
    """Ensure bundled Windows DLLs and libraries are on the search path."""
    base_dir = os.path.dirname(__file__)

    dll_dir = os.path.join(base_dir, 'windows-libs', 'dll')
    if dll_dir not in sys.path:
        sys.path.insert(0, dll_dir)

    lib_dir = os.path.join(base_dir, 'windows-libs', 'lib')
    if lib_dir not in sys.path:
        sys.path.insert(0, lib_dir)

    os.environ['PATH'] = os.environ['PATH'] + os.pathsep + '.'


_configure_windows_paths()

import json
import socket
import time

APP_NAME = 'Assetto Corsa Plugin'
DEST_ADDR = ('127.0.0.1', 15000)
SEND_INTERVAL = 1.0 / 30
ACK_TIMEOUT = 1.5

MARGIN = 16
TEXT_FONT_SIZE = 26
TEXT_APP_W = 520
TEXT_APP_H = 72
TEXT_INNER_X = MARGIN
TEXT_INNER_Y1 = 2 + MARGIN
TEXT_INNER_Y2 = 36 + MARGIN

dest_label = None
connection_label = None
app_text = None

sock = None
send_timer = 0.0
last_ack_time = 0.0

def _clear(app):
    """Hide title/icon/border and make background transparent."""
    try:
        ac.setTitle(app, '')
        ac.setTitlePosition(app, -10000, -10000)
        ac.setIconPosition(app, -10000, -10000)
        ac.drawBorder(app, 0)
        ac.setBackgroundOpacity(app, 0)
    except Exception:
        pass


def _add_label(app, text, x, y, size=TEXT_FONT_SIZE):
    """Create a label at the requested coordinates and set font size."""
    label = ac.addLabel(app, text)
    ac.setPosition(label, x, y)
    try:
        ac.setFontSize(label, size)
    except Exception:
        pass
    return label


def _json_default(o):
    """Serialize AC vector-like objects (x/y/z) to [x, y, z]; fallback to str."""
    if all(hasattr(o, a) for a in ('x', 'y', 'z')):
        return [float(getattr(o, a, 0.0)) for a in ('x', 'y', 'z')]
    try:
        return float(o)
    except Exception:
        return str(o)


def acMain(ac_version):
    """Initialize UI widgets and network resources."""
    global dest_label, connection_label, sock, app_text

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
    except OSError as exc:
        ac.log('Unable to create UDP socket: {}'.format(exc))
        sock = None

    app_text = ac.newApp(APP_NAME)
    _clear(app_text)
    ac.setSize(app_text, TEXT_APP_W, TEXT_APP_H)
    ac.setPosition(app_text, 0, 0)

    connection_label = _add_label(app_text, 'Assetto Corsa Bridge: Disconnected', TEXT_INNER_X, TEXT_INNER_Y1)
    dest_label = _add_label(app_text, 'Destination: {}:{}'.format(*DEST_ADDR), TEXT_INNER_X, TEXT_INNER_Y2)

    return APP_NAME


def acUpdate(deltaT):
    """Update the connection indicator and publish telemetry only when sending."""
    global send_timer, last_ack_time

    if sock is not None:
        try:
            while True:
                data, _ = sock.recvfrom(1024)
                if data:
                    last_ack_time = time.time()
                    try:
                        json.loads(data.decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
        except BlockingIOError:
            pass
        except OSError as exc:
            ac.log('UDP recv error: {}'.format(exc))
        except Exception as exc:
            ac.log('Unexpected UDP recv error: {}'.format(exc))

    if connection_label is not None:
        ac.setText(
            connection_label,
            'Bridge: Connected' if (last_ack_time and (time.time() - last_ack_time) <= ACK_TIMEOUT) else 'Bridge: Disconnected'
        )

    send_timer += deltaT
    if send_timer < SEND_INTERVAL:
        return
    send_timer = 0.0

    if sock is None:
        return

    try:
        cs, i = ac.getCarState, 0

        payload = {
            'speed_ms': float(cs(i, acsys.CS.SpeedMS)),
            'gas': float(cs(i, acsys.CS.Gas)),
            'brake': float(cs(i, acsys.CS.Brake)),
            'rpm': float(cs(i, acsys.CS.RPM)),
            'steer': float(cs(i, acsys.CS.Steer)),
            'turbo_boost': float(cs(i, acsys.CS.TurboBoost)),
            'gear': int(cs(i, acsys.CS.Gear)),
            'wheel_angular_speed': [float(v) for v in cs(i, acsys.CS.WheelAngularSpeed)],
            'world_position': [float(v) for v in cs(i, acsys.CS.WorldPosition)],
            'dynamic_pressure': [float(v) for v in cs(i, acsys.CS.DynamicPressure)],
            'current_tyres_core_temp': [float(v) for v in cs(i, acsys.CS.CurrentTyresCoreTemp)],
            'load': [float(v) for v in cs(i, acsys.CS.Load)],
            'velocity': cs(i, acsys.CS.Velocity),
            'local_velocity': cs(i, acsys.CS.LocalVelocity),
            'local_angular_velocity': cs(i, acsys.CS.LocalAngularVelocity),
            'tire_contact_normal': [cs(i, acsys.CS.TyreContactNormal, j) for j in range(4)],
            'tire_contact_point': [cs(i, acsys.CS.TyreContactPoint, j) for j in range(4)],
        }

        sock.sendto(json.dumps(payload, default=_json_default).encode('utf-8'), DEST_ADDR)
    except OSError as exc:
        ac.log('UDP send error: {}'.format(exc))
    except Exception as exc:
        ac.log('Unexpected UDP send error: {}'.format(exc))


def acShutdown():
    """Release resources when the plugin shuts down."""
    global sock
    if sock:
        try:
            sock.close()
        except Exception as exc:
            ac.log('UDP socket close error: {}'.format(exc))
        finally:
            sock = None
