"""Offline distribution integrity check. No instrument access or FPGA build."""
import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def main():
    manifest = json.loads((ROOT / 'PACKAGE_MANIFEST.json').read_text())
    errors = []
    for name, expected in manifest['sha256'].items():
        path = ROOT / name
        if not path.is_file():
            errors.append(f'Missing: {name}')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            errors.append(f'Changed: {name}')
    for path in ROOT.rglob('*.py'):
        if any(p.startswith('.') for p in path.relative_to(ROOT).parts):
            continue
        try:
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        except SyntaxError as exc:
            errors.append(str(exc))
    for entry in ET.parse(ROOT / 'OpenMCS_Primer25K.gprj').findall('.//File'):
        if not (ROOT / entry.attrib['path']).is_file():
            errors.append('Missing project source: ' + entry.attrib['path'])
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1
    print(f"PASS: {len(manifest['sha256'])} file hashes, Python syntax, and FPGA project paths.")
    print('This check does not validate hardware operation or FPGA timing.')
    return 0

if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        raise SystemExit(1)
