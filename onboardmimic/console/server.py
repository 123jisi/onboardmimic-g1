"""Robot-local console. Boot and GET are read-only; setup requires an explicit POST."""
import argparse
import sys
import json
import math
import secrets
import subprocess
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs
from onboardmimic.core.client import OnboardClient
from onboardmimic.console.client import OnboardBackend

from onboardmimic.core.registry import catalog, select, select_input, DEFAULT_ROBOT, DEFAULT_BACKEND
from onboardmimic.core.paths import ROOT

class Console:
    def __init__(self, backend, static, runner=subprocess.run, connection=None, tracking=None,
                 robot_id=DEFAULT_ROBOT, backend_id=DEFAULT_BACKEND):
        robot, controller = select(robot_id, backend_id)
        self.selection = {'robot_id': robot.id, 'backend_id': controller.id}
        self.backend, self.static, self.runner = backend, Path(static), runner
        self.token = secrets.token_urlsafe(32)
        self.input_mode = "planner"
        self.mode_lock = threading.RLock()
        self.tracking = tracking or select_input("pico")
        self.pnlink = select_input('pnlink')
        self.connection = connection or robot.connection()

    def platform(self):
        """Describe installed adapters without connecting or changing control ownership."""
        return dict(catalog(), schema_version=1, selected=dict(self.selection),
                    switching={'supported': False, 'reason': 'service_configuration_required'})

    @property
    def connect_status(self):
        return self.connection.connect_status

    @connect_status.setter
    def connect_status(self, value):
        self.connection.connect_status = value

    def systemctl(self, *args):
        return self.runner(['systemctl','--user',*args], capture_output=True, text=True, timeout=4)

    def status(self):
        s = dict(self.backend.status())
        s = self.connection.status(s,self.runner)
        # Gateway is the sole mode authority.
        self.input_mode = s.get('input_mode', 'planner')
        s['input_mode'] = self.input_mode
        s.setdefault('pose_adapter_ready', False)
        s['input_mode_switch_available'] = bool(s.get('pose_adapter_ready') and s.get('input_mode_switch_available'))
        s['planner_control_allowed'] = bool(s.get('planner_control_allowed',True) and self.input_mode=='planner')
        if 'capabilities' in s:
            s['planner_control_allowed'] &= 'planner' in s['capabilities']
            s['input_mode_switch_available'] &= 'pose' in s['capabilities']
        s['tracking_input'] = {'id':self.tracking.id,'label':self.tracking.label,**self.tracking.status()}
        s['tracking_inputs'] = {self.tracking.id:s['tracking_input'], 'pnlink':dict(id='pnlink', label='PN Link', **self.pnlink.status())}
        if s.get('pnlink_session') is not None:
            s['tracking_inputs']['pnlink']['service'] = self.pnlink.service_status()
        s['pico'] = s['tracking_input']  # Legacy console API alias.
        return s

    @property
    def pico_runs(self):
        return self.tracking.runs

    @pico_runs.setter
    def pico_runs(self, path):
        self.tracking.runs = Path(path)
        self.tracking.status_file=None

    def pico_status(self):
        return self.tracking.status()

    def pico_skeleton(self):
        return self.tracking.skeleton()

    @staticmethod
    def authority(x):
        return {k:x[k] for k in ('actor','operator_generation','expected_revision') if k in x}

    def api(self, route, x=None):
        if route not in ('/api/operator','/api/motions','/api/imports','/api/playback','/api/inputs/pnlink'):
            raise ValueError('未知接口')
        return self.backend.api(route, x)

    def command(self, x):
        # Serialize selection with all starts and control POSTs. Estop bypasses this lock.
        if x.get('action') == 'estop':
            return self._command(x)
        with self.mode_lock:
            if x.get('action') in ('select_pnlink','pause_pnlink','resume_pnlink'):
                if 'alignment_confirmed' in x and type(x['alignment_confirmed']) is not bool:
                    raise ValueError('无效对齐确认')
                self.backend.command(x['action'], alignment_confirmed=x.get('alignment_confirmed',False), **self.authority(x))
                return self.status()
            if x.get('action')=='select_pico':
                if x.get('pico_mode','pose') not in ('pose','planner','frozen','vr3pt'):
                    raise ValueError('未知 PICO 子模式')
                if type(x.get('alignment_confirmed',False)) is not bool:raise ValueError('无效对齐确认')
                self.backend.command('select_pico',pico_mode=x.get('pico_mode','pose'),
                    alignment_confirmed=x.get('alignment_confirmed',False),**self.authority(x))
                return self.status()
            if x.get('action') == 'select_input_mode':
                mode = x.get('input_mode')
                if mode not in ('planner', 'pose'): raise ValueError('未知输入模式')
                status = self.status()
                available = (status.get('state') == 'active' and not status.get('mode_switch_pending')) if mode == 'planner' else status['input_mode_switch_available']
                if not available:
                    raise ValueError('请先启动 控制器，并等待当前模式切换完成')
                self.backend.command('select_pose' if mode=='pose' else 'select_planner', **self.authority(x))
                return self.status()
            if self.input_mode == 'pose' and x.get('action') in ('prepare_pose', 'start_policy'):
                raise ValueError('请先返回 Planner 模式，再执行回正或启动')
            self._command(x)
            return self.status()

    def _command(self, x):
        action = x.get('action')
        if action == 'connect_services':
            s = self.status()
            if self.authority(x) or s.get('operator',{}).get('owner'):
                self.backend.authorize(self.authority(x))
            if not s['setup_available']: raise ValueError('请先停止 控制器，或等待连接完成')
            # Explicit operator request may release vendor mode; never prepares or starts 控制器.
            result = self.connection.connect(self.runner)
            if result.returncode: raise RuntimeError(result.stderr.strip())
            return self.status()
        if action == 'estop':
            # Prevent pending setup continuing after the operator cancels it.
            result = self.connection.cancel(self.runner)
            if result.returncode: raise RuntimeError('无法取消连接流程：'+result.stderr.strip())
        elif action in ('prepare_pose','start_policy'):
            if not self.status()['setup_ready']: raise ValueError('请先连接控制服务')
        else: raise ValueError('未知操作')
        return self.backend.command(action, **self.authority(x))

    def control(self, x):
        with self.mode_lock:
            if self.input_mode != 'planner' and not x.get('stop'):
                raise ValueError('当前选择 Pose，网页的 Planner 指令已禁用')
            return self._control(x)

    def _control(self, x):
        if set(x)-{'client','seq','move','turn','speed','stop','actor','operator_generation','expected_revision'}:
            raise ValueError('不支持的 Planner 参数')
        return self.backend.control(x)


def make_server(console, host='127.0.0.1', port=0):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def reply(self, data, code=200, kind='application/json; charset=utf-8'):
            if not isinstance(data,bytes): data=json.dumps(data,ensure_ascii=False).encode()
            self.send_response(code); self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(data))); self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff'); self.end_headers();self.wfile.write(data)
        def do_GET(self):
            route=urlsplit(self.path).path
            try:
                if route=='/api/session':return self.reply({'token':console.token})
                if route=='/api/platform':return self.reply(console.platform())
                if route == '/api/controller':return self.reply(console.status())
                if route == '/api/motions':return self.reply(console.api(route))
                if route=='/api/input/skeleton' and parse_qs(urlsplit(self.path).query).get('source')==['pnlink']:
                    return self.reply(console.pnlink.skeleton())
                if route in ('/api/pico/skeleton','/api/input/skeleton'):return self.reply(console.pico_skeleton())
                if route in ('/input-preview.js','/pico-skeleton.js'):
                    preview=console.tracking.preview_script()
                    return self.reply(preview.read_bytes() if preview else b'',kind='text/javascript; charset=utf-8')
                paths={'/':'index.html','/real':'index.html','/remote':'mobile.html','/real/remote':'mobile.html','/mobile.css':'mobile.css',
                       '/i18n.js':'i18n.js','/motions.js':'motions.js','/app.js':'app.js','/platform.js':'platform.js','/pnlink.js':'pnlink.js','/style.css':'style.css'}
                if route not in paths:return self.reply({'error':'Not found'},404)
                path=console.static/paths[route];data=path.read_text()
                kind={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8'}[path.suffix]
                self.reply(data.encode(),kind=kind)
            except Exception as exc:self.reply({'error':str(exc)},503)
        def do_POST(self):
            try:
                if self.path not in ('/api/controller','/api/control','/api/operator','/api/motions','/api/imports','/api/playback','/api/inputs/pnlink'):return self.reply({'error':'Not found'},404)
                origin=self.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host'):return self.reply({'error':'跨站控制已拒绝'},403)
                if not secrets.compare_digest(self.headers.get('X-Control-Token',''),console.token):return self.reply({'error':'请刷新控制页'},403)
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('无效请求长度')
                x=json.loads(self.rfile.read(size))
                if not isinstance(x,dict):raise ValueError('无效请求')
                self.reply(console.command(x) if self.path=='/api/controller' else
                           console.control(x) if self.path=='/api/control' else console.api(self.path,x))
            except ValueError as exc:self.reply({'error':str(exc)},400)
            except Exception as exc:self.reply({'error':str(exc)},503)
    return ThreadingHTTPServer((host,port),Handler)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--host',default='127.0.0.1');p.add_argument('--port',type=int,default=8769)
    p.add_argument('--pose-status',type=Path,help='Read the same live status file as runtime')
    p.add_argument('--pnlink-status',type=Path,help='Same local PN Link snapshot as runtime')
    p.add_argument('--pico-runs',type=Path);p.add_argument('--token-file',type=Path,required=True)
    p.add_argument('--input',default='pico');p.add_argument('--robot',default=DEFAULT_ROBOT);p.add_argument('--backend',default=DEFAULT_BACKEND)
    a=p.parse_args()
    robot, controller = select(a.robot,a.backend)
    if a.token_file.stat().st_mode & 0o077: p.error('Token file must be private')
    token=a.token_file.read_text().strip()
    if len(token)<32:p.error('Invalid token')
    backend=OnboardBackend(OnboardClient('http://127.0.0.1:18921',token,timeout=2),robot.id,controller.id)
    console=Console(backend,Path(__file__).parent/'static',connection=robot.connection(),tracking=select_input(a.input),
                    robot_id=robot.id,backend_id=controller.id)
    if a.pnlink_status: console.pnlink.status_file=a.pnlink_status
    if a.pico_runs: console.pico_runs=a.pico_runs
    if a.pose_status: console.tracking.status_file=a.pose_status
    make_server(console,a.host,a.port).serve_forever()
