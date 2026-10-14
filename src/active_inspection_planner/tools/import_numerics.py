"""Import the minimal SciPy 1.16.3 sources; no Python runtime in the C++ port."""
import pathlib
import re
import shutil
import sys

source = pathlib.Path(sys.argv[1])
dest = pathlib.Path(__file__).resolve().parents[1] / 'vendor' / 'scipy'
dest.mkdir(parents=True, exist_ok=True)
shutil.copy(source / 'LICENSE.txt', dest / 'LICENSE.txt')
fit = source / 'scipy/interpolate/fitpack'
todo = ['parcur', 'splev']
seen = set()
(dest / 'fitpack').mkdir(exist_ok=True)
while todo:
    name = todo.pop()
    if name in seen:
        continue
    seen.add(name)
    text = (fit / (name + '.f')).read_text()
    shutil.copy(fit / (name + '.f'), dest / 'fitpack')
    todo.extend(re.findall(r'\bcall\s+(fp\w+)', text, re.I))
    todo.extend(re.findall(r'\b(fprati)\(', text, re.I))
kd = source / 'scipy/spatial/ckdtree/src'
(dest / 'ckdtree').mkdir(exist_ok=True)
for path in list(kd.glob('*.h')) + [kd / (n + '.cxx') for n in ('build', 'query', 'query_ball_point')]:
    text = path.read_text().replace('#include <numpy/npy_common.h>', '#include <cstdint>\n#include <vector>\n#include <limits>\n#define NPY_LIKELY(x) (x)\n#define NPY_UNLIKELY(x) (x)').replace('#define ckdtree_intp_t npy_intp', '#define ckdtree_intp_t intptr_t')
    if path.name == 'ckdtree_decl.h':
        text = '#include <cassert>\n' + text
    (dest / 'ckdtree' / path.name).write_text(text)
lb = source / 'scipy/optimize'
text = (lb / '__lbfgsb.c').read_text()
(dest / 'lbfgsb.c').write_text(text.replace('#include "__lbfgsb.h"', '#include "lbfgsb.h"'))
header = (lb / '__lbfgsb.h').read_text()
blas = header[header.index('// BLAS'):header.index('static PyObject* lbfgsb_error')]
decl = header[header.index('static void setulb'):header.index('static char doc_setulb')].replace('static void setulb', 'void setulb')
(dest / 'lbfgsb.h').write_text('#pragma once\n#include <math.h>\n' + blas + decl)
(dest / 'README.md').write_text('Sources: https://github.com/scipy/scipy/tree/v1.16.3\n\n'
    'FITPACK routines are unchanged. cKDTree replaces NumPy integer and branch macros with C++ equivalents. '
    'L-BFGS-B removes only its CPython wrapper and exports setulb; the numerical C source is unchanged. '
    'The importer records these transformations reproducibly. See LICENSE.txt and source notices.\n')
