"""Fetch a checksum-pinned Open3D SDK into a user-owned dependency directory."""
import argparse
import hashlib
import importlib.util
import pathlib
import shutil
import subprocess
import tarfile

parser = argparse.ArgumentParser()
parser.add_argument('--directory', type=pathlib.Path,
                    default=pathlib.Path(__file__).resolve().parents[1] / '.dependencies')
parser.add_argument('--archive', type=pathlib.Path,
                    help='Use an already-downloaded SDK archive (checksum verified)')
parser.add_argument('--runtime-directory', type=pathlib.Path)
args = parser.parse_args()
args.directory.mkdir(parents=True, exist_ok=True)
name = 'open3d-devel-linux-x86_64-cxx11-abi-0.19.0'
archive = args.archive if args.archive is not None else args.directory / (name + '.tar.xz')
expected = '2e525fd2afe7e80907d2b6e3c66b69e2ef1481dbceed5727b749a6e65cba5720'
config = args.directory / name / 'lib/cmake/Open3D/Open3DConfig.cmake'
if not config.exists():
    if not archive.exists():
        if args.archive is not None:
            raise FileNotFoundError(archive)
        subprocess.run(['curl', '-fLsS', '--retry', '3',
                        'https://github.com/isl-org/Open3D/releases/download/v0.19.0/' + archive.name,
                        '-o', str(archive)], check=True)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise RuntimeError('Open3D archive checksum mismatch')
    with tarfile.open(archive) as tar:
        tar.extractall(args.directory, filter='data')
runtime = args.runtime_directory
if runtime is None:
    spec = importlib.util.find_spec('open3d')
    if spec is not None:
        runtime = pathlib.Path(spec.origin).parent
if runtime is not None:
    for library in ('libc++.so.1', 'libc++abi.so.1'):
        source = runtime / library
        if source.exists():
            shutil.copyfile(source, args.directory / name / 'lib' / library)
print(args.directory / name)
print('The SDK requires libc++ and libc++abi. Matching wheel runtimes were copied when available; otherwise install the system runtime libraries.')
