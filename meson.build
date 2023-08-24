project('ecc', 'c',
    meson_version: '>=1.1.0',
    version: run_command('dunamai', 'from', 'git', '--style', 'semver', '--dirty', check: true).stdout().strip(),
    default_options: ['c_std=gnu11', 'default_library=static', 'warning_level=3' ])

top_dir = meson.current_source_dir()

# dunamai is used for version sync with gconf (in the same way poetry-dynamic-versioning does)
pymod = import('python')
py3 = pymod.find_installation('python3', modules: ['dunamai'])

# about libecc
# below file listings should be moved in successive small 'meson.build' file in each subdir, that
# only contains the file listing, replacing them with a lonely 'subdir('src/xxx') instead.

ecc_inc = include_directories('include')

# module declaration, each module files listing is declare in its own directory
subdir('src/utils')
subdir('src/nn')
subdir('src/fp')
subdir('src/curves')
subdir('src/hash')
subdir('src/sig')
subdir('src/ecdh')

# detect if an external rand source implementation is passed
use_external_rand_opt = get_option('use_external_rand')
if use_external_rand_opt
rand_src = files(get_option('with_rand_source'))
else
rand_src = files()
endif

# libecc libraries declaration. For each library, the library itself and the
# corresponding dependency object (includedir and library to link with) is
# also declared
#
# INFO: defaulting to static lib only (see project declaration).
# to build both static and dynamic library, use -Ddefault_library=both option
#
libarith_lib = library('libarith',
    sources: [ fp_mod_src, nn_mod_src, rand_src, utils_arith_src ],
    include_directories: ecc_inc,
)
libarith_dep = declare_dependency(
    link_with: libarith_lib,
    include_directories: ecc_inc,
)

libec_lib = library('libecc',
  sources: [ curves_mod_src, utils_ec_src ],
  link_with: libarith_lib,
  include_directories: ecc_inc,
)
libec_dep = declare_dependency(
    link_with: libec_lib,
    include_directories: ecc_inc,
)

libsign_lib = library('libsign',
  sources: [ hash_mod_src, sig_mod_src, key_mod_src, utils_sign_src, ecdh_mod_src ],
  link_with: libarith_lib,
  include_directories: ecc_inc,
)
libsign_dep = declare_dependency(
    link_with: libsign_lib,
    include_directories: ecc_inc,
)

# building dynamic libraries on request only
with_dynamic_libs_opt = get_option('with_dynamic_libs')
if with_dynamic_libs_opt



endif
