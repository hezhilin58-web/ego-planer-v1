#!/usr/bin/env python3
"""Loopback-only control panel for a separately owned ROS simulation."""
import csv
import io
import argparse
import atexit
from collections import deque
import json
import math
import os
from pathlib import Path
import secrets
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).resolve().parent
ACTIVE_STATES = {'TRAVEL', 'ALIGN', 'HOVER', 'RECOVER', 'RETURN_HOME', 'DESCEND'}


def defaults():
    mission = yaml.safe_load((ROOT / 'src/planner/plan_manage/launch/inspection.yaml').read_text())
    return dict(max_vel=0.8, max_acc=0.8, height=1.0, hover_time=mission['hover_time'],
                stations=[dict(x=s['position'][0], y=s['position'][1], yaw_deg=s['yaw_deg'])
                          for s in mission['stations']])


def validate(data):
    if not isinstance(data, dict) or set(data) != {'max_vel', 'max_acc', 'height', 'hover_time', 'stations'}:
        raise ValueError('配置字段不完整，请刷新页面后重试')

    def number(value, low, high, label):
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError('%s须在 %g 到 %g 之间' % (label, low, high))
        return float(value)

    result = dict(max_vel=number(data['max_vel'], .2, 2., '最大速度'),
                  max_acc=number(data['max_acc'], .2, 2., '最大加速度'),
                  height=number(data['height'], .5, 2.5, '高度'),
                  hover_time=number(data['hover_time'], .5, 60., '悬停时间'))
    if not isinstance(data['stations'], list) or not 1 <= len(data['stations']) <= 50:
        raise ValueError('巡检点数须在 1 到 50 之间')
    result['stations'] = []
    for i, station in enumerate(data['stations'], 1):
        if not isinstance(station, dict) or set(station) != {'x', 'y', 'yaw_deg'}:
            raise ValueError('P%d 的坐标或航向字段缺失' % i)
        result['stations'].append(dict(
            x=number(station['x'], -18., 18., 'P%d X 坐标' % i),
            y=number(station['y'], -18., 18., 'P%d Y 坐标' % i),
            yaw_deg=number(station['yaw_deg'], -360., 360., 'P%d 目标航向' % i)))
    return result


class Panel:
    def __init__(self, directory=None, ros_port=11324):
        self.directory = Path(directory) if directory else ROOT / '.ros/panel'
        self.directory.mkdir(parents=True, exist_ok=True)
        self.ros_port = ros_port
        self.lock = threading.RLock()
        self.process = None
        self.log = None
        self.active_config = None
        self.started_at = None
        self.config = defaults()
        saved = self.directory / 'config.json'
        if saved.exists():
            self.config = validate(json.loads(saved.read_text()))

    def _alive(self):
        return self.process is not None and self.process.poll() is None

    def _mission(self):
        if not self._alive():
            return None
        try:
            data = json.loads((self.directory / 'status.json').read_text())
            if time.time() - data['received_at'] <= 2:
                return data
        except (OSError, ValueError, KeyError):
            pass
        return None

    def _planning(self):
        try:
            data = json.loads((self.directory / 'status.json').read_text())
            metrics = data.get('planning')
            if metrics is not None:
                return dict(metrics, archived=not self._alive() or time.time() - data['received_at'] > 2)
        except (OSError, ValueError, KeyError):
            pass
        return None

    def state(self):
        with self.lock:
            return dict(running=self._alive(), mission=self._mission(), planning=self._planning(),
                        active_config=self.active_config, started_at=self.started_at,
                        config=self.config, exit_code=self.process.poll() if self.process else None)

    def save(self, data):
        checked = validate(data)
        with self.lock:
            temporary = self.directory / 'config.tmp'
            temporary.write_text(json.dumps(checked, indent=2), encoding='utf-8')
            os.replace(temporary, self.directory / 'config.json')
            self.config = checked
        return checked

    def _environment(self):
        env = dict(os.environ, EGO_MASTER_URI='http://127.0.0.1:%d' % self.ros_port,
                   ROS_MASTER_URI='http://127.0.0.1:%d' % self.ros_port, ROS_IP='127.0.0.1')
        env.pop('ROS_HOSTNAME', None)
        return env

    def launch(self, data):
        checked = validate(data)
        with self.lock:
            mission = self._mission()
            if self._alive() and (mission is None or mission['state'] in ACTIVE_STATES):
                raise RuntimeError('仿真正在启动或巡检正在执行，请先关闭仿真再应用新配置')
            self.stop()
            # Never attach to, replace or stop an unrelated ROS master.
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    probe.bind(('127.0.0.1', self.ros_port))
                except OSError as exc:
                    raise RuntimeError('面板专用 ROS 端口已被占用，请关闭另一份面板仿真') from exc
            self.save(checked)
            mission_config = yaml.safe_load((ROOT / 'src/planner/plan_manage/launch/inspection.yaml').read_text())
            mission_config.update(hover_time=checked['hover_time'], auto_start=False,
                                  stations=[dict(position=[s['x'], s['y'], checked['height']], yaw_deg=s['yaw_deg'])
                                            for s in checked['stations']])
            # A long configured dwell must not always fail the original 45-second station timeout.
            mission_config['station_timeout'] = max(float(mission_config['station_timeout']),
                                                   checked['hover_time'] + float(mission_config['stable_time']) + 20.)
            config_file = self.directory / 'mission.yaml'
            config_file.write_text(yaml.safe_dump(mission_config, sort_keys=False), encoding='utf-8')
            status_file = self.directory / 'status.json'
            status_file.unlink(missing_ok=True)
            (self.directory / 'planning-metrics.jsonl').unlink(missing_ok=True)
            self.log = (self.directory / 'simulation.log').open('w')
            command = [str(ROOT / 'run-local.sh'), 'inspection', 'auto_start:=false',
                       '--sigint-timeout=5', '--sigterm-timeout=2',
                       'max_vel:=%g' % checked['max_vel'], 'max_acc:=%g' % checked['max_acc'],
                       'config_file:=' + str(config_file), 'panel_status_file:=' + str(status_file)]
            try:
                self.process = subprocess.Popen(command, cwd=ROOT, env=self._environment(),
                                                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                self.log.close()
                self.log = None
                raise
            self.active_config = checked
            self.started_at = time.time()

    def start_mission(self):
        with self.lock:
            mission = self._mission()
            if not mission or mission['state'] not in {'WAIT_READY', 'COMPLETE', 'FAILED'}:
                raise RuntimeError('请等待仿真就绪；执行中的巡检不能重复开始')
            try:
                result = subprocess.run(['rosservice', 'call', '/inspection/start', '{}'],
                                        env=self._environment(), capture_output=True, text=True, timeout=8)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError('任务启动请求超时，请查看仿真日志') from exc
            if result.returncode or 'success: True' not in result.stdout:
                raise RuntimeError((result.stderr or result.stdout or '任务启动失败').strip()[:300])

    def stop(self):
        with self.lock:
            if self._alive():
                # Let roslaunch shut down its independently grouped nodes and core in order.
                # Node + core cleanup can exceed the former 12-second parent timeout.
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=3)
            if self.log:
                self.log.close()
                self.log = None

    def logs(self):
        try:
            with (self.directory / 'simulation.log').open(errors='replace') as stream:
                return ''.join(deque(stream, maxlen=60))[-14000:]
        except FileNotFoundError:
            return '仿真尚未启动。'


def handler_for(panel, port, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            if args and str(args[0]).startswith('POST'):
                super().log_message(format, *args)

        def send(self, value, status=200, content_type='application/json; charset=utf-8'):
            body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode() if isinstance(value, dict) else value
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(body)

        def host_valid(self):
            return self.headers.get('Host') in {'127.0.0.1:%d' % port, 'localhost:%d' % port}

        def do_GET(self):
            if not self.host_valid():
                return self.send(dict(error='Invalid host'), 403)
            if self.path == '/api/state':
                return self.send(dict(**panel.state(), token=token))
            if self.path == '/api/planning.csv':
                output = io.StringIO()
                columns = ['seq', 'goal_id', 'time', 'stage', 'success', 'duration_ms', 'goal_response_ms', 'trajectory_id']
                writer = csv.DictWriter(output, fieldnames=columns, extrasaction='ignore')
                writer.writeheader()
                path = panel.directory / 'planning-metrics.jsonl'
                if path.exists():
                    for line in path.read_text(encoding='utf-8').splitlines():
                        try:
                            writer.writerow(json.loads(line))
                        except (ValueError, TypeError):
                            continue
                return self.send(output.getvalue().encode('utf-8-sig'), content_type='text/csv; charset=utf-8')
            if self.path == '/api/logs':
                return self.send(dict(text=panel.logs()))
            assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/app.css': ('app.css', 'text/css; charset=utf-8'),
                      '/app.js': ('app.js', 'text/javascript; charset=utf-8')}
            if self.path not in assets:
                return self.send(dict(error='Not found'), 404)
            filename, kind = assets[self.path]
            self.send((ASSETS / filename).read_bytes(), content_type=kind)

        def do_POST(self):
            origin = self.headers.get('Origin')
            allowed = {'http://127.0.0.1:%d' % port, 'http://localhost:%d' % port}
            if not self.host_valid() or (origin and origin not in allowed) or self.headers.get('X-Panel-Token') != token:
                return self.send(dict(error='请在本机面板中操作并刷新页面'), 403)
            if self.headers.get('Content-Type') != 'application/json':
                return self.send(dict(error='Expected JSON'), 415)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 65536:
                    raise ValueError('请求大小无效')
                data = json.loads(self.rfile.read(length))
                if self.path == '/api/config':
                    panel.save(data)
                elif self.path == '/api/launch':
                    panel.launch(data)
                elif self.path == '/api/start':
                    panel.start_mission()
                elif self.path == '/api/stop':
                    panel.stop()
                else:
                    return self.send(dict(error='Not found'), 404)
                self.send(dict(ok=True, **panel.state()))
            except (ValueError, TypeError) as exc:
                self.send(dict(error=str(exc)), 400)
            except RuntimeError as exc:
                self.send(dict(error=str(exc)), 409)
            except OSError:
                self.send(dict(error='本地文件或仿真进程操作失败，请查看面板终端'), 500)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='EGO-Planner 本地巡检仿真面板')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--ros-port', type=int, default=11324)
    args = parser.parse_args()
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    panel = Panel(ros_port=args.ros_port)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(panel, args.port, secrets.token_urlsafe(32)))
    atexit.register(panel.stop)
    print('巡检控制台：http://127.0.0.1:%d\nCtrl+C 关闭面板及其管理的仿真。' % args.port, flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        panel.stop()


if __name__ == '__main__':
    main()
