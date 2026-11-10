"""修复旧版FreeMASTER将GBK中文字节写成XML实体的问题；先备份，不改变量或通信配置。"""
from pathlib import Path
import re
import shutil
import sys
import xml.etree.ElementTree as ET

RUN = re.compile(r'(?:&#x[89a-fA-F][0-9a-fA-F];){2,}')


def repair(path):
    original=path.read_text(encoding='utf-8-sig')
    def replace(match):
        data=bytes(int(value,16) for value in re.findall(r'&#x([0-9a-fA-F]+);',match.group()))
        try:
            text=data.decode('gbk')
        except UnicodeDecodeError:
            return match.group()
        return text if any('\u4e00'<=c<='\u9fff' for c in text) else match.group()
    fixed=RUN.sub(replace,original)
    if fixed==original:
        print(path.name+'：无需修复')
        return
    ET.fromstring(fixed)
    shutil.copy2(path,str(path)+'.bak')
    path.write_text(fixed,encoding='utf-8',newline='\n')
    print(path.name+'：已修复，原文件备份为.pmpx.bak')


if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    paths=[Path(p) for p in sys.argv[1:]] or list(Path(__file__).resolve().parents[1].glob('*.pmpx'))
    for path in paths:
        repair(path)
