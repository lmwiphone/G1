"""Apply a context patch only after git apply --check succeeds; preserve local changes."""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('lightning_root', type=Path)
p.add_argument('--apply', action='store_true', help='Apply after successful check; default prints the check result only')
args = p.parse_args()
root = args.lightning_root.expanduser().resolve()
patch = Path(__file__).resolve().parents[1]/'lightning_integration.patch'
subprocess.run(['git', 'apply', '--check', str(patch)], cwd=root, check=True)
print('Patch matches; existing changes outside these hunks are preserved.')
if args.apply:
    subprocess.run(['git', 'apply', str(patch)], cwd=root, check=True)
    print('Applied. Review with git diff, then rebuild lightning.')
