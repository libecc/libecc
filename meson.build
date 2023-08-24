project('libecc', 'c',
    meson_version: '>=1.1.0',
    version: run_command('dunamai', 'from', 'git', '--style', 'semver', '--dirty', check: true).stdout().strip(),
    default_options: ['c_std=gnu11', 'default_library=static', 'warning_level=3' ],
    license: [ 'BSD-3-Clause', 'GPL-2.0-or-later' ],
    license_files: [ 'LICENSE' ],
)

top_dir = meson.current_source_dir()

# dunamai is used for version sync with gconf (in the same way poetry-dynamic-versioning does)
pymod = import('python')
py3 = pymod.find_installation('python3', modules: ['dunamai'])

# about libecc
# below file listings should be moved in successive small 'meson.build' file in each subdir, that
# only contains the file listing, replacing them with a lonely 'subdir('src/xxx') instead.

ecc_inc = include_directories('include')
subdir('include/libecc')

# module declaration, each module files listing is declare in its own directory
subdir('src/utils')
subdir('src/nn')
subdir('src/fp')
subdir('src/curves')
subdir('src/hash')
subdir('src/sig')
subdir('src/ecdh')
subdir('src/external_deps')

# detect if an external rand source implementation is passed
use_external_rand_opt = get_option('use_external_rand')
if use_external_rand_opt
  rand_src = files(get_option('with_rand_source'))
else
  rand_src = files()
endif

# globally used build args, TODO: to make configurable
build_args = [
  '-fno-builtin',
  '-D_FORTIFY_SOURCE=2',
  '-fstack-protector-strong',
  '-DWITH_STDLIB',
  '-DUSE_WARN_UNUSED_RET',
  '-ffreestanding'
]


# libecc libraries declaration. For each library, the library itself and the
# corresponding dependency object (includedir and library to link with) is
# also declared
#
# INFO: defaulting to static lib only (see project declaration).
# to build both static and dynamic library, use -Ddefault_library=both option
#
libarith_lib = library('arith',
    sources: [
      fp_mod_src,
      nn_mod_src,
      rand_src,
      utils_arith_src,
    ],
    include_directories: ecc_inc,
    install : true,
    c_args: build_args,
)
libarith_dep = declare_dependency(
    link_with: libarith_lib,
    include_directories: ecc_inc,
)

libec_lib = library('ecc',
  sources: [
    curves_mod_src,
    utils_ec_src,
    fp_mod_src,
    nn_mod_src,
    rand_src,
    utils_arith_src
  ],
  include_directories: ecc_inc,
  install : true,
  c_args: build_args,
)
libec_dep = declare_dependency(
    link_with: libec_lib,
    include_directories: ecc_inc,
)

libsign_lib = library('sign',
  sources: [
    hash_mod_src,
    sig_mod_src,
    key_mod_src,
    utils_sign_src,
    ecdh_mod_src,
    curves_mod_src,
    utils_ec_src,
    fp_mod_src,
    nn_mod_src,
    rand_src,
    utils_arith_src
  ],
  include_directories: ecc_inc,
  install : true,
  c_args: build_args,
)
libsign_dep = declare_dependency(
    link_with: libsign_lib,
    include_directories: ecc_inc,
)

# About tests, see src/tests/meson.build. Enabled with -Dwith_tests=true
with_tests_opt = get_option('with_tests')
if with_tests_opt
  subdir('src/tests')
endif
