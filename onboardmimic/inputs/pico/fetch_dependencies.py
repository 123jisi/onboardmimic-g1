"""Fetch pinned receive-only dependencies; extracts no packages and starts nothing."""
import hashlib,json,urllib.request
from pathlib import Path

def main():
    root=Path(__file__).resolve().parent
    items=[]
    for manifest in ('dependencies.json','build_wheels.json'):
        items.extend(json.loads((root/manifest).read_text())['artifacts'])
    for item in items:
        dest=root/'vendor'/item['file'];dest.parent.mkdir(parents=True,exist_ok=True)
        if not dest.exists():
            tmp=dest.with_suffix(dest.suffix+'.part')
            with urllib.request.urlopen(item['url'],timeout=120) as r,tmp.open('wb') as out:
                while True:
                    data=r.read(1024*1024)
                    if not data:break
                    out.write(data)
            tmp.replace(dest)
        h=hashlib.sha256(dest.read_bytes()).hexdigest()
        if h!=item['sha256']:raise SystemExit('Dependency checksum mismatch: '+dest.name)
        print(dest.name+': SHA256 OK')
if __name__=='__main__':main()
