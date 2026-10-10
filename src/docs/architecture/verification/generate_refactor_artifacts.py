"""Generate a source-complete review bundle and current package topology."""
import ast
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess
import xml.etree.ElementTree as ET

SOURCE = Path(__file__).resolve().parents[3]
REPOSITORY = SOURCE.parent
OUTPUT = SOURCE / 'docs/architecture'


def git_paths(directory, arguments, prefix=Path()):
    result = subprocess.run(['git', *arguments], cwd=directory,
                            check=True, capture_output=True, text=True)
    return [prefix / name for name in result.stdout.splitlines()]


def source_bundle():
    paths = git_paths(REPOSITORY, ['diff', '--name-only', '--', 'src'])
    paths += git_paths(REPOSITORY, ['ls-files', '--others', '--exclude-standard', 'src'])
    vendor = Path('src/perception/perception/BGNet')
    paths += git_paths(REPOSITORY / vendor, ['diff', '--name-only'], vendor)
    paths += git_paths(REPOSITORY / vendor, ['ls-files', '--others', '--exclude-standard'], vendor)
    selected = []
    for path in sorted(set(paths)):
        absolute = REPOSITORY / path
        if not absolute.is_file() or '__pycache__' in path.parts:
            continue
        if path.suffix not in {'.py', '.cpp', '.hpp', '.h', '.xml', '.cfg'} and (
                path.name != 'CMakeLists.txt' and 'resource' not in path.parts):
            continue
        selected.append(path)
    text = ['# Complete refactoring code bundle', '',
            'Full current contents of changed/new source, build, manifest, resource and verification files.',
            'Documentation is linked from [the architecture index](../new_arch/README.md).',
            'This generated review artifact contains no omitted code. Protocol `...` bodies are actual abstract declarations.',
            'The optimizer moved to `auv_experiments`; the separately removed legacy planner Tools are not restored.', '']
    manifest = []
    languages = {'.py': 'python', '.cpp': 'cpp', '.hpp': 'cpp', '.h': 'cpp', '.xml': 'xml', '.cfg': 'ini'}
    for path in selected:
        data = (REPOSITORY / path).read_bytes()
        content = data.decode('utf-8')
        digest = hashlib.sha256(data).hexdigest()
        manifest.append({'path': str(path), 'sha256': digest, 'bytes': len(data)})
        language = 'cmake' if path.name == 'CMakeLists.txt' else languages.get(path.suffix, 'text')
        fence = '`' * max(4, max((len(match[0]) for match in re.finditer(r'`+', content)), default=0) + 1)
        text += [f'### NBV_paper_ros2/{path}', '', f'SHA-256: `{digest}`', '',
                 fence + language, content[:-1] if content.endswith('\n') else content, fence, '']
    (OUTPUT / 'refactoring_complete_code.md').write_text('\n'.join(text))
    (OUTPUT / 'refactoring_code_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return len(selected)


def cpp_class_counts(path):
    source = path.read_text()
    masked = re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"',
                    lambda match: ' ' * len(match[0]), source, flags=re.S)
    concrete = abstract = 0
    for match in re.finditer(r'\b(?:class|struct)\s+\w+[^;{}]*\{', masked):
        if re.search(r'enum\s*$', masked[:match.start()]):
            continue
        index = match.end()
        level = 1
        while level and index < len(masked):
            level += (masked[index] == '{') - (masked[index] == '}')
            index += 1
        body = masked[match.end():index - 1]
        is_abstract = bool(re.search(r'virtual\b[^;{}]*=\s*0\s*;', body))
        abstract += is_abstract
        concrete += 1
    return abstract, concrete


def topology():
    packages = {ET.parse(path).getroot().findtext('name'): path.parent
                for path in SOURCE.glob('*/package.xml')}
    edges = {name: set() for name in packages}
    counts = {name: [0, 0] for name in packages}
    for name, directory in packages.items():
        manifest = ET.parse(directory / 'package.xml').getroot()
        edges[name].update(element.text for element in manifest
                           if element.tag in {'depend', 'exec_depend', 'build_depend', 'build_export_depend'}
                           and element.text in packages and element.text != name)
        for path in directory.rglob('*'):
            if not path.is_file() or any(part in {'test', '__pycache__', '.git'} for part in path.parts):
                continue
            if path.suffix == '.py':
                tree = ast.parse(path.read_text())
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        imports = [alias.name.split('.')[0] for alias in node.names]
                    elif isinstance(node, ast.ImportFrom) and not node.level:
                        imports = [(node.module or '').split('.')[0]]
                    else:
                        imports = []
                    edges[name].update(dependency for dependency in imports
                                       if dependency in packages and dependency != name)
                    if isinstance(node, ast.ClassDef) and path.name != 'setup.py':
                        counts[name][1] += 1
                        counts[name][0] += any(ast.unparse(base).split('.')[-1] in {'Protocol', 'ABC'}
                                               for base in node.bases)
            elif path.suffix in {'.cpp', '.hpp', '.h'}:
                abstract, classes = cpp_class_counts(path)
                counts[name][0] += abstract
                counts[name][1] += classes
    cycles = []
    def visit(node, path):
        if node in path:
            cycles.append(path[path.index(node):] + [node])
            return
        for dependency in sorted(edges[node]):
            visit(dependency, path + [node])
    for name in packages:
        visit(name, [])
    assert not cycles, cycles
    rows = []
    metrics = {}
    for name in sorted(packages):
        incoming = sum(name in dependencies for dependencies in edges.values())
        outgoing = len(edges[name])
        abstract, classes = counts[name]
        instability = outgoing / (incoming + outgoing) if incoming + outgoing else 0.
        abstractness = abstract / classes if classes else 0.
        distance = abs(abstractness + instability - 1.)
        conditional = not incoming + outgoing or not classes
        status = ('Balanced' if distance <= .1 else 'Zone of Pain'
                  if abstractness + instability < 1 else 'Zone of Uselessness')
        metrics[name] = dict(fan_in=incoming, fan_out=outgoing, Na=abstract, Nc=classes,
                             I=instability, A=abstractness, D=distance, conditional=conditional)
        rows.append(f'| {name} | {incoming} | {outgoing} | {abstract} | {classes} | '
                    f'{instability:.3f}{"*" if not incoming + outgoing else ""} | '
                    f'{abstractness:.3f}{"*" if not classes else ""} | '
                    f'{distance:.3f}{"*" if conditional else ""} | {status}{"*" if conditional else ""} |')
    values = [row['D'] for row in metrics.values()]
    defined = [row['D'] for row in metrics.values() if not row['conditional']]
    violations = [(source, target) for source, dependencies in edges.items() for target in dependencies
                  if metrics[target]['I'] > metrics[source]['I']]
    text = ['# Implemented component topology and metrics', '',
            'Snapshot of the ten current ROS packages. Arrows mean dependent → dependency.',
            'Edges combine production imports and internal runtime/build manifest dependencies (including launch closure).',
            'External libraries, topic flow, tests and transitive edges are excluded.', '', '```mermaid', 'graph TD']
    for name in sorted(packages):
        text.append(f'    {name}[{name}]')
    for name in sorted(edges):
        for dependency in sorted(edges[name]):
            text.append(f'    {name} --> {dependency}')
    text += ['```', '', 'Detected cycles: **none**.',
             'Strictly increasing-instability edges: ' + ('**none**.' if not violations else str(violations)), '',
             '## Metrics', '',
             'Ca/Ce are distinct package neighbors. Nc counts production Python classes/C++ class and struct definitions;',
             'Na counts explicit Protocol/ABC definitions and pure-virtual C++ interfaces. Packaging/test classes are excluded.',
             'Vendored BGNet source remains counted as in the original audit, including checkout-only research classes.',
             '`I = Ce/(Ca+Ce)`, `A = Na/Nc`, `D = |A+I-1|`. The original audit threshold D ≤ 0.100 is retained.',
             'An asterisk marks a zero imputation for an undefined denominator and conditional D/classification.', '',
             '| Package | Fan-in | Fan-out | Na | Nc | I | A | D | Classification |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---|', *rows, '',
             f'All-package conditional mean D: {statistics.mean(values):.4f}; population variance: {statistics.pvariance(values):.4f}.',
             f'Defined-subset mean D: {statistics.mean(defined):.4f}; population variance: {statistics.pvariance(defined):.4f}.', '',
             '## Interpretation', '',
             'These are release-package metrics, not library-layer metrics. Each runtime package still contains concrete adapters',
             'alongside its policy, so a Zone of Pain label alone does not prove a remaining inward-dependency violation.',
             'The C++ policy libraries are separately reusable targets inside their existing package releases.',
             'The original seven-package mean is not directly comparable to this ten-package snapshot.',
             'Further splitting releases solely to improve D would require an explicit reuse/release decision, not extra empty interfaces.', '']
    (OUTPUT / 'refactoring_topology.md').write_text('\n'.join(text))
    (OUTPUT / 'refactoring_topology.json').write_text(json.dumps({'dependencies': {k: sorted(v) for k, v in edges.items()},
                                                               'metrics': metrics}, indent=2) + '\n')
    return len(packages), sum(map(len, edges.values()))


if __name__ == '__main__':
    packages, edges = topology()
    files = source_bundle()
    print(f'Generated complete code for {files} files; {packages} packages, {edges} edges, no cycles.')
