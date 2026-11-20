"""Back up/retarget ONLY PN capture UDP destination; never change Wi-Fi or robot.

Default is a read-only preview. --apply explicitly submits capture configuration.
Device restart is separate and never implied by apply. No raw HTML/passwords saved.
"""
import argparse
from datetime import datetime, timezone
from html.parser import HTMLParser
import ipaddress
import json
import os
from pathlib import Path
from urllib.request import Request, urlopen
from urllib.parse import urlsplit


class CaptureFields(HTMLParser):
    IDS={'udp-local-port','udp-remote-port','capture-udp-local-ip','capture-udp-remote-ip',
         'sensor-host-ip','group-tag','save-gyro-bias','send-sensor-ned','enable-ptp',
         'tcp-remote-address','tcp-remote-port'}
    def __init__(self):
        super().__init__();self.values={};self.radios={};self.options=[];self.select=None
    def handle_starttag(self,tag,attrs):
        a=dict(attrs)
        if tag=='input':
            if a.get('id') in self.IDS:self.values.setdefault(a['id'],[]).append(a)
            if a.get('name') in ('pose-mode','protocol-mode') and 'checked' in a:self.radios[a['name']]=a['value']
        if tag=='select':self.select=a.get('id')
        if tag=='option' and self.select=='rotation-order':self.options.append((a.get('value'),'selected' in a))
    def handle_endtag(self,tag):
        if tag=='select':self.select=None
    def config(self):
        def val(key):return self.values.get(key,[{}])[0].get('value','')
        def flag(key):return 'checked' in self.values.get(key,[{}])[0]
        def ip(key):return str(ipaddress.IPv4Address('.'.join(a['value'] for a in self.values[key])))
        order=next((v for v,selected in self.options if selected),self.options[0][0] if self.options else None)
        if self.radios.get('protocol-mode')!='udp' or not order:raise ValueError('Require an existing UDP capture configuration')
        tcp={'remoteAddress':val('tcp-remote-address')}
        if val('tcp-remote-address'):tcp['remoteAddress']=val('tcp-remote-address')
        if val('tcp-remote-port'):tcp['remotePort']=int(val('tcp-remote-port'))
        return dict(captureMode=self.radios['pose-mode'],groupTag=val('group-tag'),
                    keepGyroBias=flag('save-gyro-bias'),ptpEnabled=flag('enable-ptp'),
                    localAddress=ip('capture-udp-local-ip'),
                    mocap=dict(protocol='udp',rotationOrder=order,tcp=tcp,
                               udp=dict(localPort=int(val('udp-local-port')),remoteAddress=ip('capture-udp-remote-ip'),remotePort=int(val('udp-remote-port')))),
                    secap=dict(hostAddress=ip('sensor-host-ip'),convNed=flag('send-sensor-ned')))


def read_config(url):
    parser=CaptureFields()
    with urlopen(url,timeout=5) as response:parser.feed(response.read(512*1024).decode())
    return parser.config()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device-url',required=True)
    p.add_argument('--target-ip');p.add_argument('--target-port',type=int,default=8002)
    p.add_argument('--expected-target',help='Refuse to apply if destination changed since inspection')
    p.add_argument('--backup',type=Path,required=True)
    p.add_argument('--restore',action='store_true');p.add_argument('--apply',action='store_true')
    p.add_argument('--restart-device',action='store_true')
    a=p.parse_args();url=a.device_url.rstrip('/')
    parsed=urlsplit(url)
    if parsed.scheme!='http' or not parsed.hostname or parsed.username or parsed.password or parsed.path:p.error('Use a plain local device HTTP origin')
    ipaddress.IPv4Address(parsed.hostname)
    current=read_config(url)
    if a.restore:
        backup=json.loads(a.backup.read_text())
        if backup['device_url']!=url:raise ValueError('Backup belongs to a different device URL')
        wanted=backup['capture_configuration']
    else:
        if a.target_ip is None:p.error('--target-ip is required')
        ipaddress.IPv4Address(a.target_ip)
        if not 1<=a.target_port<=65535:p.error('Invalid target port')
        wanted=json.loads(json.dumps(current))
        wanted['mocap']['udp'].update(remoteAddress=a.target_ip,remotePort=a.target_port)
    if a.expected_target and current['mocap']['udp']['remoteAddress']!=a.expected_target:
        raise ValueError('Device destination changed; inspect again before applying')
    if a.apply:
        if not a.restore:
            # Exclusive create: never overwrite the sole original configuration.
            a.backup.parent.mkdir(parents=True,exist_ok=True)
            fd=os.open(a.backup,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
            with os.fdopen(fd,'w') as out:json.dump(dict(device_url=url,captured_at=datetime.now(timezone.utc).isoformat(),capture_configuration=current),out,indent=2)
        req=Request(url+'/apply-capture-config',data=json.dumps(wanted).encode(),headers={'Content-Type':'application/json'},method='POST')
        with urlopen(req,timeout=5) as response:result=json.load(response)
        if result.get('success') is not True:raise RuntimeError('Device rejected capture configuration')
        actual=read_config(url)
        if actual!=wanted:raise RuntimeError('Device configuration readback differs; use the saved backup')
        if a.restart_device:
            with urlopen(Request(url+'/restart-device',data=b'',method='POST'),timeout=5) as response:result=json.load(response)
            if result.get('success') is not True:raise RuntimeError('Device restart not confirmed')
    elif a.restart_device:p.error('--restart-device requires --apply')
    print(json.dumps(dict(applied=a.apply,device_url=url,old_udp=current['mocap']['udp'],new_udp=wanted['mocap']['udp'],restart_requested=a.restart_device,robot_output_enabled=False),indent=2))


if __name__=='__main__':main()
