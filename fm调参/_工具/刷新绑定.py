"""构建后只刷新现有PMPX的地址/类型，保留布局。不存在的符号报错，不留下假地址。"""
from pathlib import Path
import argparse
import importlib.util
import json
import hashlib
import os
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
spec = importlib.util.spec_from_file_location('fm_generator', HERE / '生成工程.py')
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)
from fm_xml import validate


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--elf', required=True)
    args = parser.parse_args()
    elf = Path(args.elf).resolve()
    symbols = generator.Symbols(elf)
    pending = []
    try:
        for path in sorted(HERE.parent.glob('*.pmpx')):
            tree = ET.parse(path)
            count = 0
            for obj in tree.findall('./objects/object'):
                if obj.get('type') == 'CVariable':
                    get = lambda name: obj.find(f'member[@name="{name}"]')
                    name = get('name').text
                    if get('virtual') is not None and get('virtual').text not in (None, '0'):
                        continue
                    try:
                        address, width, treat = symbols.resolve(name)
                    except Exception as error:
                        raise ValueError(f'{path.name}: {name}: {error}') from error
                    for key, value in [('address', hex(address)), ('last_addr', hex(address)),
                                       ('byte_size', str(width)), ('treat_as', str(treat))]:
                        get(key).text = value
                    count += 1
                elif obj.get('type') == 'CPrjDoc_MapFileInfo':
                    obj.find('member[@name="file_name"]').text = str(elf)
            pending.append((path, tree, count))
        records = []
        for path, tree, count in pending:
            temporary = path.with_suffix('.pmpx.refresh.tmp')
            tree.write(temporary, encoding='utf-8', xml_declaration=True)
            # Address refresh does not rewrite object IDs or existing layout.
            # Validate before replacing, so malformed files cannot be silently published.
            try:
                validate(temporary)
                os.replace(temporary, path)
            except Exception:
                temporary.unlink(missing_ok=True)
                raise
            records.append({'file': path.name, 'variables': count})
            print(f'FreeMASTER绑定已刷新: {path.name} ({count}变量)')
        (HERE / '自动刷新校验.json').write_text(json.dumps({
            'elf': str(elf), 'sha256': hashlib.sha256(elf.read_bytes()).hexdigest(),
            'files': records}, ensure_ascii=False, indent=2), encoding='utf-8')
    finally:
        symbols.stream.close()


if __name__ == '__main__':
    main()
