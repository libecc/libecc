# test suites binaries declaration. These are native tooling
# that requires libsign_dep to be build natively too, even if the
# libecc has been configured with a --cross-file argument for
# cross compilation

hostcc = meson.get_compiler('c', native: true)

# ec_self_test tests suite
ec_self_test_src = files(
  'ec_self_tests_core.c',
  'ec_self_tests.c',
)

# the executable is built with the same architecture
# as the library itself, meaning that in case of cross-compilation
# the test binary have the same architecture.
#
# Yet, it can still be executed on the build machine architecture if
# the cross file used has declared an 'exe_wrapper' entry, such as,
# for e.g. in a arm32 cross-file:
#
# [binaries]
# exe_wrapper = 'qemu-arm-static'

ec_self_test = executable(
    'ec_self_test',
    sources: [
      ec_self_test_src,
      external_deps_src,
    ],
    dependencies: libsign_dep,
    include_directories: ecc_inc,
    c_args: build_args,
    install: false,
)

test(
    'vectors',
    ec_self_test,
    args: 'vectors',
    timeout: 120,
)

# ec_tuils tests suite
ec_utils_src = files(
  'ec_utils.c',
)

ec_utils = executable(
    'ec_utils',
    sources: [
      ec_utils_src,
      external_deps_src,
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
      utils_arith_src,
    ],
    include_directories: ecc_inc,
    c_args: build_args,
    native: true,
    install: false,
)
