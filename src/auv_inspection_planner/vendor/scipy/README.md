Sources: https://github.com/scipy/scipy/tree/v1.16.3

FITPACK routines are unchanged. cKDTree replaces NumPy integer and branch macros with C++ equivalents. L-BFGS-B removes only its CPython wrapper and exports setulb; the numerical C source is unchanged. The importer records these transformations reproducibly. See LICENSE.txt and source notices.
