import json
import threading
import sys
import urllib.error
import urllib.request
from fastapi import HTTPException
from arduino.app_utils import App, Bridge
from arduino.app_bricks.web_ui import WebUI
from controller import Controller

# Bound GIL handoff latency between the 100 Hz receiver, policy and WebUI.
sys.setswitchinterval(0.001)
controller = Controller(Bridge)
ui = WebUI(port=9527, cors_origins='')
Bridge.provide('duck_state', controller.receive)
Bridge.provide('duck_boot_pd', controller.receive_boot_pd)
Bridge.provide('duck_raw', controller.receive_raw)
Bridge.provide('duck_servo_reply', controller.receive_servo_reply)

def mode(body: dict):
    try: return controller.set_mode(body['mode'])
    except (KeyError, ValueError) as exc: raise HTTPException(400, str(exc))

def command(body: dict):
    try: return controller.command(**body)
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def filtering(body: dict):
    try:
        leg, head = controller.set_action_alphas(**body)
        return {'leg':leg,'head':head}
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def pick(_body: dict = None):
    try: return controller.start_pick()
    except ValueError as exc: raise HTTPException(400, str(exc))

def roulade(_body: dict = None):
    try: return controller.start_roulade()
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_enter(_body: dict = None):
    try: return controller.set_mode('calibrate')
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_exit(_body: dict = None):
    try: return controller.set_mode('shadow')
    except ValueError as exc: raise HTTPException(400, str(exc))

def cal_targets(body: dict):
    try: return controller.cal_targets_set(body['targets'])
    except (KeyError, TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def cal_finish(body: dict):
    try: return controller.cal_finish(body['index'])
    except (KeyError, TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def servo_enter(_body: dict = None):
    try: return controller.set_mode('servo_debug')
    except ValueError as exc: raise HTTPException(400, str(exc))

def servo_exit(_body: dict = None):
    try: return controller.set_mode('shadow')
    except ValueError as exc: raise HTTPException(400, str(exc))

def servo_action(body: dict):
    try:
        return controller.servo_op(
            body['action'], body['target_id'],
            new_id=body.get('new_id'), kp=body.get('kp', 5), kd=body.get('kd', 20),
            raw_pos=body.get('raw_pos', 2047))
    except (KeyError, TypeError, ValueError) as exc:
        raise HTTPException(400, str(exc))

# Bluetooth pairing runs in host/pad_helper.py on the host: the app container has no BlueZ
# access. Docker maps msgpack-rpc-router to the host gateway.
PAD_HELPER = 'http://msgpack-rpc-router:9528'

def helper(path, body=None, timeout=5):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(PAD_HELPER+path, data, {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        try: detail = json.loads(exc.read()).get('detail', str(exc))
        except ValueError: detail = str(exc)
        raise HTTPException(exc.code, detail)
    except OSError as exc:
        raise HTTPException(503, f'gamepad helper unreachable ({exc}); run tools/install-pad-helper.sh on the board')

def bt_status():
    return helper('/bt/status', timeout=15)

def bt_action(action, timeout=5):
    def handler(body: dict):
        return helper('/bt/'+action, body, timeout)
    return handler

def joint_sign_set(body: dict):
    try: return controller.set_joint_sign(body['id'], bool(body['flipped']))
    except (KeyError, TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def joint_sign_reset(_body: dict = None):
    try: return controller.reset_joint_signs()
    except ValueError as exc: raise HTTPException(400, str(exc))

def tuning_set(body: dict):
    try: return controller.set_tuning(body)
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def tuning_level(body: dict = None):
    try: return controller.level_imu((body or {}).get('seconds', 1.0))
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

def pad_input(body: dict):
    return controller.pad_input(body)

def pad_config_set(body: dict):
    try: return controller.set_pad_config(body.get('config', body))
    except (TypeError, ValueError) as exc: raise HTTPException(400, str(exc))

ui.expose_api('GET', '/api/status', controller.status)
ui.expose_api('POST', '/api/joint_sign', joint_sign_set)
ui.expose_api('POST', '/api/joint_sign/reset', joint_sign_reset)
ui.expose_api('GET', '/api/tuning', controller.tuning_status)
ui.expose_api('POST', '/api/tuning', tuning_set)
ui.expose_api('POST', '/api/tuning/level', tuning_level)
ui.expose_api('POST', '/api/pad', pad_input)
ui.expose_api('GET', '/api/pad/config', controller.pad_config)
ui.expose_api('POST', '/api/pad/config', pad_config_set)
ui.expose_api('GET', '/api/bt/status', bt_status)
ui.expose_api('POST', '/api/bt/scan', bt_action('scan'))
ui.expose_api('POST', '/api/bt/autopair', bt_action('autopair'))
ui.expose_api('POST', '/api/bt/pair', bt_action('pair', timeout=60))
ui.expose_api('POST', '/api/bt/connect', bt_action('connect', timeout=25))
ui.expose_api('POST', '/api/bt/disconnect', bt_action('disconnect', timeout=25))
ui.expose_api('POST', '/api/bt/remove', bt_action('remove', timeout=25))
ui.expose_api('POST', '/api/mode', mode)
ui.expose_api('POST', '/api/command', command)
ui.expose_api('POST', '/api/filter', filtering)
ui.expose_api('POST', '/api/pick', pick)
ui.expose_api('POST', '/api/roulade', roulade)
ui.expose_api('POST', '/api/cal/enter', cal_enter)
ui.expose_api('POST', '/api/cal/exit', cal_exit)
ui.expose_api('POST', '/api/cal/targets', cal_targets)
ui.expose_api('POST', '/api/cal/finish', cal_finish)
ui.expose_api('POST', '/api/servo/enter', servo_enter)
ui.expose_api('POST', '/api/servo/exit', servo_exit)
ui.expose_api('POST', '/api/servo', servo_action)
worker = threading.Thread(target=controller.run, name='policy50', daemon=True)
worker.start()
try:
    App.run()
finally:
    controller.stop()
    worker.join(timeout=2)
