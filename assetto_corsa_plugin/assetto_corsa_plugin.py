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
import shutil
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
_pending_spawn_ack = None

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


def _assetto_root():
    """Return the absolute path to the Assetto Corsa installation root."""

    return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, os.pardir, os.pardir))


def _resolve_spawn_paths():
    """Return the active track's spawn.ini path and backup destination."""

    try:
        track = ac.getTrackName(0) or ''
    except Exception:
        track = ''

    if not track:
        return None, None

    try:
        configuration = ac.getTrackConfiguration(0) or ''
    except Exception:
        configuration = ''

    track_dir = os.path.join(_assetto_root(), 'content', 'tracks', track)
    if configuration:
        track_dir = os.path.join(track_dir, configuration)

    spawn_ini = os.path.join(track_dir, 'data', 'spawn.ini')
    backup = spawn_ini + '.assetto_corsa_bridge.bak'
    return spawn_ini, backup


def _rewrite_pit_spawn(original, position, yaw):
    """Produce updated spawn.ini contents targeting only the PIT_0 section."""

    lines = original.splitlines()
    newline = '\n' if original.endswith('\n') else ''
    new_lines = []
    in_pit = False
    position_written = False
    rotation_written = yaw is None

    for line in lines:
        stripped = line.strip()
        if stripped.startswith('[') and stripped.endswith(']'):
            section = stripped[1:-1].strip().upper()
            in_pit = section == 'PIT_0'
            new_lines.append(line)
            continue

        if in_pit and stripped:
            upper = stripped.upper()
            if upper.startswith('POSITION'):
                new_lines.append('POSITION={:.6f}, {:.6f}, {:.6f}'.format(*position))
                position_written = True
                continue
            if upper.startswith('ROTATION'):
                if yaw is not None:
                    new_lines.append('ROTATION={:.6f}'.format(yaw))
                    rotation_written = True
                else:
                    new_lines.append(line)
                continue

        new_lines.append(line)

    if not position_written:
        new_lines.extend([
            '[PIT_0]',
            'POSITION={:.6f}, {:.6f}, {:.6f}'.format(*position),
        ])
        if yaw is not None:
            new_lines.append('ROTATION={:.6f}'.format(yaw))
            rotation_written = True
    elif yaw is not None and not rotation_written:
        for index in range(len(new_lines) - 1, -1, -1):
            if new_lines[index].strip().upper().startswith('POSITION'):
                insert_at = index + 1
                break
        else:
            insert_at = len(new_lines)
        new_lines.insert(insert_at, 'ROTATION={:.6f}'.format(yaw))

    updated = '\n'.join(new_lines) + newline
    return updated


def _update_pit_spawn(position, yaw):
    """Persist the provided vehicle position into PIT_0 of spawn.ini."""

    spawn_ini, backup = _resolve_spawn_paths()
    if not spawn_ini:
        return False, 'Active track information unavailable.'

    if not os.path.isfile(spawn_ini):
        return False, 'spawn.ini not found at {}'.format(spawn_ini)

    if backup and not os.path.isfile(backup):
        try:
            shutil.copyfile(spawn_ini, backup)
        except OSError as exc:
            ac.log('Unable to create spawn backup: {}'.format(exc))

    try:
        with open(spawn_ini, 'r', encoding='utf-8') as handle:
            original = handle.read()
    except OSError as exc:
        return False, 'Unable to read spawn.ini: {}'.format(exc)

    updated = _rewrite_pit_spawn(original, position, yaw)
    if updated == original:
        return True, 'Pit spawn already matches vehicle position.'

    try:
        with open(spawn_ini, 'w', encoding='utf-8') as handle:
            handle.write(updated)
    except OSError as exc:
        return False, 'Unable to write spawn.ini: {}'.format(exc)

    return True, 'Pit spawn updated.'


def _reset_spawn():
    """Restore spawn.ini from the backup captured before any modifications."""

    spawn_ini, backup = _resolve_spawn_paths()
    if not spawn_ini or not backup:
        return False, 'Active track information unavailable.'

    if not os.path.isfile(backup):
        return False, 'No spawn backup available to restore.'

    try:
        shutil.copyfile(backup, spawn_ini)
    except OSError as exc:
        return False, 'Unable to restore spawn.ini: {}'.format(exc)

    return True, 'Pit spawn reset to original location.'


def _handle_set_spawn(mode):
    """Execute a spawn request issued by the ROS bridge."""

    if mode == 'reset':
        return _reset_spawn()

    try:
        raw_position = ac.getCarState(0, acsys.CS.WorldPosition)
        position = [float(v) for v in raw_position]
    except Exception as exc:
        return False, 'Unable to read vehicle position: {}'.format(exc)

    try:
        yaw = float(ac.getCarState(0, acsys.CS.Yaw))
    except Exception:
        yaw = None

    return _update_pit_spawn(position, yaw)


def _handle_bridge_message(payload):
    """Parse bridge acknowledgements and trigger spawn updates when requested."""

    global _pending_spawn_ack

    command = payload.get('set_spawn') if isinstance(payload, dict) else None
    if command not in ('set', 'reset'):
        return

    success, message = _handle_set_spawn(command)
    _pending_spawn_ack = {
        'mode': command,
        'success': bool(success),
        'message': str(message or ''),
    }


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
    global send_timer, last_ack_time, _pending_spawn_ack

    if sock is not None:
        try:
            while True:
                data, _ = sock.recvfrom(1024)
                if data:
                    last_ack_time = time.time()
                    try:
                        payload = json.loads(data.decode('utf-8'))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    _handle_bridge_message(payload)
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

        ack = _pending_spawn_ack
        if ack is not None:
            payload['set_spawn_ack'] = ack
            _pending_spawn_ack = None

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
