"""Systemd connection presentation, independent of robot protocols."""
import json


def prepare_blockers(status):
    """Explain existing admission flags; never change them or run a guard."""
    issues=[]
    def add(code,message,**details):issues.append(dict(code=code,message=message,**details))
    if not status.get('setup_ready'):
        add('services',status.get('setup_error') or status.get('setup_message') or '控制服务未连接')
    if status.get('state') not in ('stopped','error'):
        add('controller_state','当前控制器状态不允许准备',state=status.get('state'))
    if status.get('pids'):
        add('controller_running','控制器进程仍在运行，请先停止控制器')
    telemetry=status.get('telemetry') or {}
    if not telemetry.get('fresh'):
        add('feedback_stale','机器人状态反馈已过期或尚未收到')
    elif not telemetry.get('valid'):
        add('feedback_invalid','机器人关节或 IMU 反馈无效')
    else:
        details={row['index']:row for row in telemetry.get('fault_details',[])}
        for index in telemetry.get('faults',[]):
            fault=details.get(index,{})
            label=('（'+fault['name']+'）') if fault.get('name') else ''
            message='电机 %s%s 报告故障'%(index,label)
            if fault.get('code') is not None:
                message+='：%s'%fault['code']
                if fault.get('code_hex'):message+='（'+fault['code_hex']+'）'
            if fault.get('temperature') is not None:
                message+='；温度反馈 %s °C'%fault['temperature']
            add('motor_fault',message,index=index,motorstate=fault.get('code'),
                **{k:v for k,v in fault.items() if k not in ('index','code')})
        if telemetry.get('max_speed_rad_s',0)>12.5:
            add('joint_speed','关节速度超过准备允许范围',max_speed_rad_s=telemetry['max_speed_rad_s'])
    continuity=status.get('feedback_continuity')
    if continuity is not None and not continuity.get('ready'):
        add('feedback_continuity','机器人反馈链路未就绪')
    if not issues and not status.get('prepare_available'):
        add('backend','后台暂不允许准备，请查看控制器日志')
    return issues

class ServiceConnection:
    def __init__(self, connect_unit, bridge_unit, status_path):
        self.connect_unit, self.bridge_unit = connect_unit, bridge_unit
        self.connect_status = status_path

    def systemctl(self, runner, *args):
        return runner(['systemctl','--user',*args],capture_output=True,text=True,timeout=4)

    def status(self, base, runner):
        s = dict(base)
        result = self.systemctl(runner, 'show', self.connect_unit, '--property=ActiveState,Result', '--no-pager')
        props = dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)
        setup = props.get('ActiveState','unknown')
        bridge = self.systemctl(runner, 'show', self.bridge_unit, '--property=ActiveState,MainPID', '--no-pager')
        bridge_props = dict(line.split('=',1) for line in bridge.stdout.splitlines() if '=' in line)
        connected = (not result.returncode and not bridge.returncode and setup == 'active'
                     and props.get('Result') == 'success' and bridge_props.get('ActiveState') == 'active'
                     and bridge_props.get('MainPID', '0').isdigit() and int(bridge_props.get('MainPID', '0')) > 0)
        s['setup_state'] = setup
        s['setup_available'] = s.get('state') in ('stopped','error') and not s.get('pids') and setup != 'activating'
        s['setup_error'], s['setup_message'] = '', ''
        try:
            progress = json.loads(self.connect_status.read_text())
            if not isinstance(progress, dict): progress = {}
        except (OSError, ValueError):
            progress = {}
        connected = bool(connected and progress.get('phase') == 'ready'
                         and progress.get('bridge_pid') == int(bridge_props.get('MainPID','0')))
        s['setup_ready'] = connected
        if setup == 'activating':
            s['setup_message'] = progress.get('detail') if progress.get('phase') in ('feedback','release','bridge') else '正在连接控制服务'
        elif setup == 'failed' or props.get('Result') not in ('','success',None):
            detail = progress.get('detail') if progress.get('phase') == 'error' else props.get('Result','unknown')
            s['setup_error'] = '连接服务失败：'+str(detail)
        elif setup == 'active' and not connected:
            s['setup_error'] = '桥接未就绪，请重新连接服务'
        elif connected and s.get('state') in ('stopped','error'):
            s['setup_message'] = '控制服务已连接'
        if not connected:
            s['prepare_available'] = False
            s['pose_ready'] = False
        s['prepare_blockers']=prepare_blockers(s) if not s.get('prepare_available') else []
        return s

    def connect(self, runner):
        return self.systemctl(runner,'restart','--no-block',self.connect_unit)

    def cancel(self, runner):
        return self.systemctl(runner,'stop',self.connect_unit)
