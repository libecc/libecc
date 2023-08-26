project('libecc', 'c',
    meson_version: '>=1.1.0',
    version: run_command('dunamai', 'from', 'git', '--style', 'semver', '--dirty', check: true).stdout().strip(),
    default_options: ['c_std=gnu11', 'default_library=static', 'warning_level=3' ],
    license: 'BSD-3-Clause OR GPL-2.0-or-later',
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
  '-DUSE_WARN_UNUSED_RET',
  '-ffreestanding'
]


# About configurable options, based on makefiles

# 1. build-relative options first

# info compilers arguments (such as m32/m64 should be cross-file related.
# just using a different cross-file (event for m32 on m64 arch for e.g. is enough
# to change the behavior
with_wordsize_opt = get_option('with_wordsize')
if with_wordsize_opt == '16'
  build_args += '-DWORDSIZE=16'
elif with_wordsize_opt == '32'
  build_args += '-DWORDSIZE=32'
elif with_wordsize_opt == '64'
  build_args += '-DWORDSIZE=64'
endif

with_stdlib_opt = get_option('with_stdlib')
if with_stdlib_opt
  build_args += '-DWITH_STDLIB'
endif

# 2. security relative options

with_sig_blinding_opt = get_option('with_sig_blinding')
if with_sig_blinding_opt
  build_args += '-DUSE_SIG_BLINDING'
endif

with_complete_formulas_opt = get_option('with_complete_formulas')
if with_complete_formulas_opt
  build_args += '-DNO_USE_COMPLETE_FORMULAS'
endif

with_double_add_opt = get_option('with_double_add')
if with_double_add_opt == 'true'
  build_args += '-DUSE_DOUBLE_ADD_ALWAYS'
elif with_double_add_opt == 'false'
  build_args += '-DUSE_MONTY_LADDER'
endif

with_monty_ladder_opt = get_option('with_monty_ladder')
if with_monty_ladder_opt == 'true'
  build_args += '-DUSE_MONTY_LADDER'
elif with_monty_ladder_opt == 'false'
  build_args += '-DUSE_DOUBLE_ADD_ALWAYS'
endif

assert(not (with_double_add_opt == 'true' and with_monty_ladder_opt == 'true'), 'with_double_add and with_monty_ladder are incompatible options!')

# 3. crypto engines relative options. It is possible to fully disable all
# engines and manually select which one must be built

with_override_ecc_config_opt = get_option('with_override_ecc_config')
if with_override_ecc_config_opt
  build_args += '-DWITH_LIBECC_CONFIG_OVERRIDE'
endif

with_curve_frp256v1_opt = get_option('with_curve_frp256v1')
if with_curve_frp256v1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_FRP256V1'
endif
with_curve_secp192r1_opt = get_option('with_curve_secp192r1')
if with_curve_secp192r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP192R1'
endif
with_curve_secp224r1_opt = get_option('with_curve_secp224r1')
if with_curve_secp224r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP224R1'
endif
with_curve_secp256r1_opt = get_option('with_curve_secp256r1')
if with_curve_secp256r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP256R1'
endif
with_curve_secp384r1_opt = get_option('with_curve_secp384r1')
if with_curve_secp384r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP384R1'
endif
with_curve_secp521r1_opt = get_option('with_curve_secp521r1')
if with_curve_secp521r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP521R1'
endif
with_curve_brainpoolp192r1_opt = get_option('with_curve_brainpoolp192r1')
if with_curve_brainpoolp192r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP192R1'
endif
with_curve_brainpoolp224r1_opt = get_option('with_curve_brainpoolp224r1')
if with_curve_brainpoolp224r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP224R1'
endif
with_curve_brainpoolp256r1_opt = get_option('with_curve_brainpoolp256r1')
if with_curve_brainpoolp256r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP256R1'
endif
with_curve_brainpoolp384r1_opt = get_option('with_curve_brainpoolp384r1')
if with_curve_brainpoolp384r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP384R1'
endif
with_curve_brainpoolp512r1_opt = get_option('with_curve_brainpoolp512r1')
if with_curve_brainpoolp512r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP512R1'
endif
with_curve_gost256_opt = get_option('with_curve_gost256')
if with_curve_gost256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST256'
endif
with_curve_gost512_opt = get_option('with_curve_gost512')
if with_curve_gost512_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST512'
endif
with_curve_sm2p256test_opt = get_option('with_curve_sm2p256test')
if with_curve_sm2p256test_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SM2P256TEST'
endif
with_curve_sm2p256v1_opt = get_option('with_curve_sm2p256v1')
if with_curve_sm2p256v1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SM2P256V1'
endif
with_curve_wei25519_opt = get_option('with_curve_wei25519')
if with_curve_wei25519_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_WEI25519'
endif
with_curve_wei448_opt = get_option('with_curve_wei448')
if with_curve_wei448_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_WEI448'
endif
with_curve_gost_r3410_2012_256_paramseta_opt = get_option('with_curve_gost_r3410_2012_256_paramseta')
if with_curve_gost_r3410_2012_256_paramseta_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_256_PARAMSETA'
endif
with_curve_secp256k1_opt = get_option('with_curve_secp256k1')
if with_curve_secp256k1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP256K1'
endif
with_curve_gost_r3410_2001_testparamset_opt = get_option('with_curve_gost_r3410_2001_testparamset')
if with_curve_gost_r3410_2001_testparamset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_TESTPARAMSET'
endif
with_curve_gost_r3410_2001_cryptopro_a_paramset_opt = get_option('with_curve_gost_r3410_2001_cryptopro_a_paramset')
if with_curve_gost_r3410_2001_cryptopro_a_paramset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_CRYPTOPRO_A_PARAMSET'
endif
with_curve_gost_r3410_2001_cryptopro_b_paramset_opt = get_option('with_curve_gost_r3410_2001_cryptopro_b_paramset')
if with_curve_gost_r3410_2001_cryptopro_b_paramset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_CRYPTOPRO_B_PARAMSET'
endif
with_curve_gost_r3410_2001_cryptopro_c_paramset_opt = get_option('with_curve_gost_r3410_2001_cryptopro_c_paramset')
if with_curve_gost_r3410_2001_cryptopro_c_paramset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_CRYPTOPRO_C_PARAMSET'
endif
with_curve_gost_r3410_2001_cryptopro_xcha_paramset_opt = get_option('with_curve_gost_r3410_2001_cryptopro_xcha_paramset')
if with_curve_gost_r3410_2001_cryptopro_xcha_paramset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_CRYPTOPRO_XCHA_PARAMSET'
endif
with_curve_gost_r3410_2001_cryptopro_xchb_paramset_opt = get_option('with_curve_gost_r3410_2001_cryptopro_xchb_paramset')
if with_curve_gost_r3410_2001_cryptopro_xchb_paramset_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2001_CRYPTOPRO_XCHB_PARAMSET'
endif
with_curve_gost_r3410_2012_256_paramsetb_opt = get_option('with_curve_gost_r3410_2012_256_paramsetb')
if with_curve_gost_r3410_2012_256_paramsetb_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_256_PARAMSETB'
endif
with_curve_gost_r3410_2012_256_paramsetc_opt = get_option('with_curve_gost_r3410_2012_256_paramsetc')
if with_curve_gost_r3410_2012_256_paramsetc_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_256_PARAMSETC'
endif
with_curve_gost_r3410_2012_256_paramsetd_opt = get_option('with_curve_gost_r3410_2012_256_paramsetd')
if with_curve_gost_r3410_2012_256_paramsetd_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_256_PARAMSETD'
endif
with_curve_gost_r3410_2012_512_paramsettest_opt = get_option('with_curve_gost_r3410_2012_512_paramsettest')
if with_curve_gost_r3410_2012_512_paramsettest_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_512_PARAMSETTEST'
endif
with_curve_gost_r3410_2012_512_paramseta_opt = get_option('with_curve_gost_r3410_2012_512_paramseta')
if with_curve_gost_r3410_2012_512_paramseta_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_512_PARAMSETA'
endif
with_curve_gost_r3410_2012_512_paramsetb_opt = get_option('with_curve_gost_r3410_2012_512_paramsetb')
if with_curve_gost_r3410_2012_512_paramsetb_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_512_PARAMSETB'
endif
with_curve_gost_r3410_2012_512_paramsetc_opt = get_option('with_curve_gost_r3410_2012_512_paramsetc')
if with_curve_gost_r3410_2012_512_paramsetc_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_GOST_R3410_2012_512_PARAMSETC'
endif
with_curve_secp192k1_opt = get_option('with_curve_secp192k1')
if with_curve_secp192k1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP192K1'
endif
with_curve_secp224k1_opt = get_option('with_curve_secp224k1')
if with_curve_secp224k1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_SECP224K1'
endif
with_curve_brainpoolp192t1_opt = get_option('with_curve_brainpoolp192t1')
if with_curve_brainpoolp192t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP192T1'
endif
with_curve_brainpoolp224t1_opt = get_option('with_curve_brainpoolp224t1')
if with_curve_brainpoolp224t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP224T1'
endif
with_curve_brainpoolp256t1_opt = get_option('with_curve_brainpoolp256t1')
if with_curve_brainpoolp256t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP256T1'
endif
with_curve_brainpoolp320r1_opt = get_option('with_curve_brainpoolp320r1')
if with_curve_brainpoolp320r1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP320R1'
endif
with_curve_brainpoolp320t1_opt = get_option('with_curve_brainpoolp320t1')
if with_curve_brainpoolp320t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP320T1'
endif
with_curve_brainpoolp384t1_opt = get_option('with_curve_brainpoolp384t1')
if with_curve_brainpoolp384t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP384T1'
endif
with_curve_brainpoolp512t1_opt = get_option('with_curve_brainpoolp512t1')
if with_curve_brainpoolp512t1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BRAINPOOLP512T1'
endif
with_curve_bign256v1_opt = get_option('with_curve_bign256v1')
if with_curve_bign256v1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BIGN256V1'
endif
with_curve_bign384v1_opt = get_option('with_curve_bign384v1')
if with_curve_bign384v1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BIGN384V1'
endif
with_curve_bign512v1_opt = get_option('with_curve_bign512v1')
if with_curve_bign512v1_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_CURVE_BIGN512V1'
endif
with_hash_sha224_opt = get_option('with_hash_sha224')
if with_hash_sha224_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA224'
endif
with_hash_sha256_opt = get_option('with_hash_sha256')
if with_hash_sha256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA256'
endif
with_hash_sha384_opt = get_option('with_hash_sha384')
if with_hash_sha384_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA384'
endif
with_hash_sha512_opt = get_option('with_hash_sha512')
if with_hash_sha512_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA512'
endif
with_hash_sha512_224_opt = get_option('with_hash_sha512_224')
if with_hash_sha512_224_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA512_224'
endif
with_hash_sha512_256_opt = get_option('with_hash_sha512_256')
if with_hash_sha512_256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA512_256'
endif
with_hash_sha3_224_opt = get_option('with_hash_sha3_224')
if with_hash_sha3_224_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA3_224'
endif
with_hash_sha3_256_opt = get_option('with_hash_sha3_256')
if with_hash_sha3_256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA3_256'
endif
with_hash_sha3_384_opt = get_option('with_hash_sha3_384')
if with_hash_sha3_384_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA3_384'
endif
with_hash_sha3_512_opt = get_option('with_hash_sha3_512')
if with_hash_sha3_512_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHA3_512'
endif
with_hash_sm3_opt = get_option('with_hash_sm3')
if with_hash_sm3_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SM3'
endif
with_hash_shake256_opt = get_option('with_hash_shake256')
if with_hash_shake256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_SHAKE256'
endif
with_hash_streebog256_opt = get_option('with_hash_streebog256')
if with_hash_streebog256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_STREEBOG256'
endif
with_hash_streebog512_opt = get_option('with_hash_streebog512')
if with_hash_streebog512_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_STREEBOG512'
endif
with_hash_ripemd160_opt = get_option('with_hash_ripemd160')
if with_hash_ripemd160_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_RIPEMD160'
endif
with_hash_belt_hash_opt = get_option('with_hash_belt_hash')
if with_hash_belt_hash_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_BELT_HASH'
endif
with_hash_bash224_opt = get_option('with_hash_bash224')
if with_hash_bash224_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_BASH224'
endif
with_hash_bash256_opt = get_option('with_hash_bash256')
if with_hash_bash256_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_BASH256'
endif
with_hash_bash384_opt = get_option('with_hash_bash384')
if with_hash_bash384_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_BASH384'
endif
with_hash_bash512_opt = get_option('with_hash_bash512')
if with_hash_bash512_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HASH_BASH512'
endif
with_hmac_opt = get_option('with_hmac')
if with_hmac_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_HMAC'
endif
with_sig_ecdsa_opt = get_option('with_sig_ecdsa')
if with_sig_ecdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECDSA'
endif
with_sig_eckcdsa_opt = get_option('with_sig_eckcdsa')
if with_sig_eckcdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECKCDSA'
endif
with_sig_ecsdsa_opt = get_option('with_sig_ecsdsa')
if with_sig_ecsdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECSDSA'
endif
with_sig_ecosdsa_opt = get_option('with_sig_ecosdsa')
if with_sig_ecosdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECOSDSA'
endif
with_sig_ecfsdsa_opt = get_option('with_sig_ecfsdsa')
if with_sig_ecfsdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECFSDSA'
endif
with_sig_ecgdsa_opt = get_option('with_sig_ecgdsa')
if with_sig_ecgdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECGDSA'
endif
with_sig_ecrdsa_opt = get_option('with_sig_ecrdsa')
if with_sig_ecrdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_ECRDSA'
endif
with_sig_sm2_opt = get_option('with_sig_sm2')
if with_sig_sm2_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_SM2'
endif
with_sig_eddsa25519_opt = get_option('with_sig_eddsa25519')
if with_sig_eddsa25519_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_EDDSA25519'
endif
with_sig_eddsa448_opt = get_option('with_sig_eddsa448')
if with_sig_eddsa448_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_EDDSA448'
endif
with_sig_decdsa_opt = get_option('with_sig_decdsa')
if with_sig_decdsa_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_DECDSA'
endif
with_sig_bign_opt = get_option('with_sig_bign')
if with_sig_bign_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_BIGN'
endif
with_sig_dbign_opt = get_option('with_sig_dbign')
if with_sig_dbign_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_DBIGN'
endif
with_sig_bip0340_opt = get_option('with_sig_bip0340')
if with_sig_bip0340_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_SIG_BIP0340'
endif
with_ecccdh_opt = get_option('with_ecccdh')
if with_ecccdh_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_ECCCDH'
endif
with_x25519_opt = get_option('with_x25519')
if with_x25519_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_X25519'
endif
with_x448_opt = get_option('with_x448')
if with_x448_opt
  assert(with_override_ecc_config_opt, 'You have selected a libecc configuration override without setting with_override_ecc_config_opt to true!')
  build_args += '-DWITH_X448'
endif

# Small stack handling
with_small_stack_opt = get_option('with_small_stack')
if with_small_stack_opt
  build_args += '-DUSE_SMALL_STACK'
endif
assert(not (with_small_stack_opt and with_sig_eddsa25519_opt), 'Small stack and EdDSA are incompatible options!')
assert(not (with_small_stack_opt and with_sig_eddsa448_opt), 'Small stack and EdDSA are incompatible options!')
assert(not (with_small_stack_opt and with_x25519_opt), 'Small stack and X25519 are incompatible options!')
assert(not (with_small_stack_opt and with_x448_opt), 'Small stack and X448 are incompatible options!')


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

# in order to build native tools that depends on libsign library, we should
# check if the nominal library built has been made native or cross. If
# cross, another build must be made natively for native tooling
if meson.is_cross_build()
  native_libsign_lib = library('sign_native',
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
    install : false,
    c_args: build_args,
    native: true,
  )
  native_libsign_dep = declare_dependency(
    link_with: native_libsign_lib,
    include_directories: ecc_inc,
  )
else
    native_libsign_lib = libsign_lib
    native_libsign_dep = libsign_dep
endif

# About tests, see src/tests/meson.build. Enabled with -Dwith_tests=true
with_tests_opt = get_option('with_tests')
if with_tests_opt
  subdir('src/tests')
endif
