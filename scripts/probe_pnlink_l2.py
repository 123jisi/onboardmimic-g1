"""Bounded ARP test for explicitly named devices. Does not change network state.

Requires CAP_NET_RAW/root. Use to check whether PN and the current Wi-Fi share
an Ethernet broadcast domain, independently of their configured IPv4 subnets.
"""
import argparse
import ipaddress
import json
from pathlib import Path
import socket
import struct
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',required=True)
    p.add_argument('--source-ip',required=True)
    p.add_argument('targets',nargs='+')
    a=p.parse_args()
    if not 1<=len(a.targets)<=4:p.error('Specify 1..4 known device IPs; no subnet scans')
    if '/' in a.interface or not Path('/sys/class/net',a.interface).is_dir():p.error('Invalid local interface')
    src=socket.inet_aton(str(ipaddress.IPv4Address(a.source_ip)))
    targets={socket.inet_aton(str(ipaddress.IPv4Address(ip))):ip for ip in a.targets}
    mac=bytes.fromhex(Path('/sys/class/net',a.interface,'address').read_text().strip().replace(':',''))
    found={}
    with socket.socket(socket.AF_PACKET,socket.SOCK_RAW,socket.htons(0x0806)) as conn:
        conn.bind((a.interface,0));conn.settimeout(.1)
        for _ in range(3):
            for target in targets:
                packet=b'\xff'*6+mac+b'\x08\x06'+struct.pack('!HHBBH6s4s6s4s',1,0x0800,6,4,1,mac,src,b'\0'*6,target)
                conn.send(packet)
            deadline=time.monotonic()+.7
            while time.monotonic()<deadline:
                try:raw=conn.recv(2048)
                except socket.timeout:continue
                if len(raw)<42 or raw[12:14]!=b'\x08\x06':continue
                values=struct.unpack('!HHBBH6s4s6s4s',raw[14:42])
                if values[4]==2 and values[6] in targets and values[8]==src and values[5]!=mac:
                    found[targets[values[6]]]=':'.join('%02x'%b for b in values[5])
    print(json.dumps(dict(interface=a.interface,source_ip=a.source_ip,known_targets=a.targets,replies=found,network_configuration_changed=False),indent=2))


if __name__=='__main__':main()
