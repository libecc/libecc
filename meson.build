project('ecc', 'c',
    meson_version: '>=1.1.0',
    version: run_command('dunamai', 'from', 'git', '--style', 'semver', '--dirty', check: true).stdout().strip(),
    default_options: ['c_std=gnu11' ])

top_dir = meson.current_source_dir()

# dunamai is used for version sync with gconf (in the same way poetry-dynamic-versioning does)
pymod = import('python')
py3 = pymod.find_installation('python3', modules: ['dunamai'])

# about libecc
# below file listings should be moved in successive small 'meson.build' file in each subdir, that
# only contains the file listing, replacing them with a lonely 'subdir('src/xxx') instead.

ecc_inc = include_directories('include')


utils_arith_src = files(
  'src/utils/utils.c',
  'src/utils/utils_rand.c',
  'src/utils/print_buf.c',
  'src/utils/print_fp.c',
  'src/utils/print_nn.c',
  )

utils_ec_src = files(
  'src/utils/print_curves.c',
  )

utils_sign_src = files(
  'src/utils/print_keys.c',
  )

# module declaration, can be pushed back to each module dir
nn_mod_src = files(
  'src/nn/nn_add.c',
  'src/nn/nn_add.h',
  'src/nn/nn.c',
  'src/nn/nn_config.h',
  'src/nn/nn_div.c',
  'src/nn/nn_div.h',
  'src/nn/nn_logical.c',
  'src/nn/nn_modinv.c',
  'src/nn/nn_mod_pow.c',
  'src/nn/nn_mul.c',
  'src/nn/nn_mul.h',
  'src/nn/nn_mul_redc1.c',
  'src/nn/nn_rand.c',
  )

fp_mod_src = files(
  'src/fp/fp_add.c',
  'src/fp/fp.c',
  'src/fp/fp_config.h',
  'src/fp/fp_montgomery.c',
  'src/fp/fp_mul.c',
  'src/fp/fp_mul_redc1.c',
  'src/fp/fp_mul_redc1.h',
  'src/fp/fp_pow.c',
  'src/fp/fp_rand.c',
  'src/fp/fp_sqrt.c',
  )

curves_mod_src = files(
  'src/curves/aff_pt.c',
  'src/curves/aff_pt_montgomery.c',
  'src/curves/ec_edwards.c',
  'src/curves/ec_montgomery.c',
  'src/curves/ec_params.c',
  'src/curves/ec_shortw.c',
  'src/curves/aff_pt_edwards.c',
  'src/curves/curves.c',
  'src/curves/ec_edwards.h',
  'src/curves/ec_montgomery.h',
  'src/curves/ec_params.h',
  'src/curves/prj_pt.c',
  )

hash_mod_src = files(
  'src/hash/hash_algs.c',
  'src/hash/sm3.c',
  'src/hash/streebog.c',
  'src/hash/ripemd160.c',
  'src/hash/belt-hash.c',
  'src/hash/hmac.c',
  'src/hash/bash224.c',
  'src/hash/bash224.h',
  'src/hash/bash256.c',
  'src/hash/bash256.h',
  'src/hash/bash384.c',
  'src/hash/bash384.h',
  'src/hash/bash512.c',
  'src/hash/bash512.h',
  'src/hash/bash.c',
  'src/hash/bash.h',
  'src/hash/sha224.c',
  'src/hash/sha224.h',
  'src/hash/sha256.c',
  'src/hash/sha256.h',
  'src/hash/sha2.h',
  'src/hash/sha3-224.c',
  'src/hash/sha3-224.h',
  'src/hash/sha3-256.c',
  'src/hash/sha3-256.h',
  'src/hash/sha3-384.c',
  'src/hash/sha3-384.h',
  'src/hash/sha3-512.c',
  'src/hash/sha3-512.h',
  'src/hash/sha384.c',
  'src/hash/sha384.h',
  'src/hash/sha3.c',
  'src/hash/sha3.h',
  'src/hash/sha512-224.c',
  'src/hash/sha512-224.h',
  'src/hash/sha512-256.c',
  'src/hash/sha512-256.h',
  'src/hash/sha512.c',
  'src/hash/sha512_core.c',
  'src/hash/sha512_core.h',
  'src/hash/sha512.h',
  'src/hash/shake256.c',
  'src/hash/shake256.h',
  'src/hash/shake.c',
  'src/hash/shake.h',
  )

sig_mod_src = files(
  'src/sig/decdsa.c',
  'src/sig/decdsa.h',
  'src/sig/ecdsa.c',
  'src/sig/ecdsa.h',
  'src/sig/ecfsdsa.c',
  'src/sig/ecfsdsa.h',
  'src/sig/ecgdsa.c',
  'src/sig/ecgdsa.h',
  'src/sig/eckcdsa.c',
  'src/sig/eckcdsa.h',
  'src/sig/ecosdsa.c',
  'src/sig/ecosdsa.h',
  'src/sig/ecrdsa.c',
  'src/sig/ecrdsa.h',
  'src/sig/ecsdsa.c',
  'src/sig/ecsdsa.h',
  'src/sig/eddsa.c',
  'src/sig/eddsa.h',
  'src/sig/fuzzing_ecdsa.c',
  'src/sig/fuzzing_ecdsa.h',
  'src/sig/fuzzing_ecgdsa.c',
  'src/sig/fuzzing_ecgdsa.h',
  'src/sig/fuzzing_ecrdsa.c',
  'src/sig/fuzzing_ecrdsa.h',
  'src/sig/ecdsa_common.c',
  'src/sig/ecsdsa_common.c',
  'src/sig/sig_algs.c',
  'src/sig/sm2.c',
  'src/sig/bign_common.c',
  'src/sig/bign.c',
  'src/sig/dbign.c',
  'src/sig/bip0340.c',
  )

ecdh_mod_src = files(
  'src/ecdh/ecccdh.c',
  'src/ecdh/ecccdh.h',
  'src/ecdh/x25519_448.c',
  'src/ecdh/x25519_448.h',
  )


use_external_rand_opt = get_option('use_external_rand')
if use_external_rand_opt
rand_src = files(get_option('with_rand_source'))
else
rand_src = files()
endif

key_mod_src = files('src/sig/ec_key.c')

libarith_lib = static_library('libarith',
    sources: [ fp_mod_src, nn_mod_src, rand_src, utils_arith_src ],
    include_directories: ecc_inc,
)

libarith_dep = declare_dependency(
    link_with: libarith_lib,
    include_directories: ecc_inc,
)

libec_lib = static_library('libecc',
  sources: [ curves_mod_src, utils_ec_src ],
  link_with: libarith_lib,
  include_directories: ecc_inc,
)

libsign_lib = static_library('libsign',
  sources: [ hash_mod_src, sig_mod_src, key_mod_src, utils_sign_src, ecdh_mod_src ],
  link_with: libarith_lib,
  include_directories: ecc_inc,
)
