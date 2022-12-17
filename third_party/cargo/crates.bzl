"""
@generated
cargo-raze generated Bazel file.

DO NOT EDIT! Replaced on runs of cargo-raze
"""

load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")  # buildifier: disable=load
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")  # buildifier: disable=load
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")  # buildifier: disable=load

def raze_fetch_remote_crates():
    """This function defines a collection of repos and should be called in a WORKSPACE file"""
    maybe(
        http_archive,
        name = "raze__addr2line__0_17_0",
        url = "https://crates.io/api/v1/crates/addr2line/0.17.0/download",
        type = "tar.gz",
        sha256 = "b9ecd88a8c8378ca913a680cd98f0f13ac67383d35993f86c90a70e3f137816b",
        strip_prefix = "addr2line-0.17.0",
        build_file = Label("//third_party/cargo/remote:BUILD.addr2line-0.17.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__adler__1_0_2",
        url = "https://crates.io/api/v1/crates/adler/1.0.2/download",
        type = "tar.gz",
        sha256 = "f26201604c87b1e01bd3d98f8d5d9a8fcbb815e8cedb41ffccbeb4bf593a35fe",
        strip_prefix = "adler-1.0.2",
        build_file = Label("//third_party/cargo/remote:BUILD.adler-1.0.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__aho_corasick__0_7_18",
        url = "https://crates.io/api/v1/crates/aho-corasick/0.7.18/download",
        type = "tar.gz",
        sha256 = "1e37cfd5e7657ada45f742d6e99ca5788580b5c529dc78faf11ece6dc702656f",
        strip_prefix = "aho-corasick-0.7.18",
        build_file = Label("//third_party/cargo/remote:BUILD.aho-corasick-0.7.18.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__ansi_term__0_12_1",
        url = "https://crates.io/api/v1/crates/ansi_term/0.12.1/download",
        type = "tar.gz",
        sha256 = "d52a9bb7ec0cf484c551830a7ce27bd20d67eac647e1befb56b0be4ee39a55d2",
        strip_prefix = "ansi_term-0.12.1",
        build_file = Label("//third_party/cargo/remote:BUILD.ansi_term-0.12.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__anyhow__1_0_58",
        url = "https://crates.io/api/v1/crates/anyhow/1.0.58/download",
        type = "tar.gz",
        sha256 = "bb07d2053ccdbe10e2af2995a2f116c1330396493dc1269f6a91d0ae82e19704",
        strip_prefix = "anyhow-1.0.58",
        build_file = Label("//third_party/cargo/remote:BUILD.anyhow-1.0.58.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__aquamarine__0_1_11",
        url = "https://crates.io/api/v1/crates/aquamarine/0.1.11/download",
        type = "tar.gz",
        sha256 = "96e14cb2a51c8b45d26a4219981985c7350fc05eacb7b5b2939bceb2ffefdf3e",
        strip_prefix = "aquamarine-0.1.11",
        build_file = Label("//third_party/cargo/remote:BUILD.aquamarine-0.1.11.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__assert_cmd__1_0_8",
        url = "https://crates.io/api/v1/crates/assert_cmd/1.0.8/download",
        type = "tar.gz",
        sha256 = "c98233c6673d8601ab23e77eb38f999c51100d46c5703b17288c57fddf3a1ffe",
        strip_prefix = "assert_cmd-1.0.8",
        build_file = Label("//third_party/cargo/remote:BUILD.assert_cmd-1.0.8.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__atty__0_2_14",
        url = "https://crates.io/api/v1/crates/atty/0.2.14/download",
        type = "tar.gz",
        sha256 = "d9b39be18770d11421cdb1b9947a45dd3f37e93092cbf377614828a319d5fee8",
        strip_prefix = "atty-0.2.14",
        build_file = Label("//third_party/cargo/remote:BUILD.atty-0.2.14.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__autocfg__1_1_0",
        url = "https://crates.io/api/v1/crates/autocfg/1.1.0/download",
        type = "tar.gz",
        sha256 = "d468802bab17cbc0cc575e9b053f41e72aa36bfa6b7f55e3529ffa43161b97fa",
        strip_prefix = "autocfg-1.1.0",
        build_file = Label("//third_party/cargo/remote:BUILD.autocfg-1.1.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__autocxx_bindgen__0_59_17",
        url = "https://crates.io/api/v1/crates/autocxx-bindgen/0.59.17/download",
        type = "tar.gz",
        sha256 = "f9a9a26dd38d385d23b1bf61bd231b77f690c4368aef4c77cee1b7a6da2e2042",
        strip_prefix = "autocxx-bindgen-0.59.17",
        build_file = Label("//third_party/cargo/remote:BUILD.autocxx-bindgen-0.59.17.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__backtrace__0_3_66",
        url = "https://crates.io/api/v1/crates/backtrace/0.3.66/download",
        type = "tar.gz",
        sha256 = "cab84319d616cfb654d03394f38ab7e6f0919e181b1b57e1fd15e7fb4077d9a7",
        strip_prefix = "backtrace-0.3.66",
        build_file = Label("//third_party/cargo/remote:BUILD.backtrace-0.3.66.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__bindgen__0_58_1",
        url = "https://crates.io/api/v1/crates/bindgen/0.58.1/download",
        type = "tar.gz",
        sha256 = "0f8523b410d7187a43085e7e064416ea32ded16bd0a4e6fc025e21616d01258f",
        strip_prefix = "bindgen-0.58.1",
        build_file = Label("//third_party/cargo/remote:BUILD.bindgen-0.58.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__bitflags__1_3_2",
        url = "https://crates.io/api/v1/crates/bitflags/1.3.2/download",
        type = "tar.gz",
        sha256 = "bef38d45163c2f1dde094a7dfd33ccf595c92905c8f8f4fdc18d06fb1037718a",
        strip_prefix = "bitflags-1.3.2",
        build_file = Label("//third_party/cargo/remote:BUILD.bitflags-1.3.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__bstr__0_2_17",
        url = "https://crates.io/api/v1/crates/bstr/0.2.17/download",
        type = "tar.gz",
        sha256 = "ba3569f383e8f1598449f1a423e72e99569137b47740b1da11ef19af3d5c3223",
        strip_prefix = "bstr-0.2.17",
        build_file = Label("//third_party/cargo/remote:BUILD.bstr-0.2.17.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cc__1_0_73",
        url = "https://crates.io/api/v1/crates/cc/1.0.73/download",
        type = "tar.gz",
        sha256 = "2fff2a6927b3bb87f9595d67196a70493f627687a71d87a0d692242c33f58c11",
        strip_prefix = "cc-1.0.73",
        build_file = Label("//third_party/cargo/remote:BUILD.cc-1.0.73.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cexpr__0_4_0",
        url = "https://crates.io/api/v1/crates/cexpr/0.4.0/download",
        type = "tar.gz",
        sha256 = "f4aedb84272dbe89af497cf81375129abda4fc0a9e7c5d317498c15cc30c0d27",
        strip_prefix = "cexpr-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.cexpr-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cexpr__0_6_0",
        url = "https://crates.io/api/v1/crates/cexpr/0.6.0/download",
        type = "tar.gz",
        sha256 = "6fac387a98bb7c37292057cffc56d62ecb629900026402633ae9160df93a8766",
        strip_prefix = "cexpr-0.6.0",
        build_file = Label("//third_party/cargo/remote:BUILD.cexpr-0.6.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cfg_if__0_1_10",
        url = "https://crates.io/api/v1/crates/cfg-if/0.1.10/download",
        type = "tar.gz",
        sha256 = "4785bdd1c96b2a846b2bd7cc02e86b6b3dbf14e7e53446c4f54c92a361040822",
        strip_prefix = "cfg-if-0.1.10",
        build_file = Label("//third_party/cargo/remote:BUILD.cfg-if-0.1.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cfg_if__1_0_0",
        url = "https://crates.io/api/v1/crates/cfg-if/1.0.0/download",
        type = "tar.gz",
        sha256 = "baf1de4339761588bc0619e3cbc0120ee582ebb74b53b4efbf79117bd2da40fd",
        strip_prefix = "cfg-if-1.0.0",
        build_file = Label("//third_party/cargo/remote:BUILD.cfg-if-1.0.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__clang_sys__1_3_3",
        url = "https://crates.io/api/v1/crates/clang-sys/1.3.3/download",
        type = "tar.gz",
        sha256 = "5a050e2153c5be08febd6734e29298e844fdb0fa21aeddd63b4eb7baa106c69b",
        strip_prefix = "clang-sys-1.3.3",
        build_file = Label("//third_party/cargo/remote:BUILD.clang-sys-1.3.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__clap__2_34_0",
        url = "https://crates.io/api/v1/crates/clap/2.34.0/download",
        type = "tar.gz",
        sha256 = "a0610544180c38b88101fecf2dd634b174a62eef6946f84dfc6a7127512b381c",
        strip_prefix = "clap-2.34.0",
        build_file = Label("//third_party/cargo/remote:BUILD.clap-2.34.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__clap__3_2_12",
        url = "https://crates.io/api/v1/crates/clap/3.2.12/download",
        type = "tar.gz",
        sha256 = "ab8b79fe3946ceb4a0b1c080b4018992b8d27e9ff363644c1c9b6387c854614d",
        strip_prefix = "clap-3.2.12",
        build_file = Label("//third_party/cargo/remote:BUILD.clap-3.2.12.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__clap_lex__0_2_4",
        url = "https://crates.io/api/v1/crates/clap_lex/0.2.4/download",
        type = "tar.gz",
        sha256 = "2850f2f5a82cbf437dd5af4d49848fbdfc27c157c3d010345776f952765261c5",
        strip_prefix = "clap_lex-0.2.4",
        build_file = Label("//third_party/cargo/remote:BUILD.clap_lex-0.2.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__codespan_reporting__0_11_1",
        url = "https://crates.io/api/v1/crates/codespan-reporting/0.11.1/download",
        type = "tar.gz",
        sha256 = "3538270d33cc669650c4b093848450d380def10c331d38c768e34cac80576e6e",
        strip_prefix = "codespan-reporting-0.11.1",
        build_file = Label("//third_party/cargo/remote:BUILD.codespan-reporting-0.11.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxx__1_0_71",
        url = "https://crates.io/api/v1/crates/cxx/1.0.71/download",
        type = "tar.gz",
        sha256 = "5469a6f42296f4fd40789b397383718f9a0bd75d2f9b7cedbb249996811fba27",
        strip_prefix = "cxx-1.0.71",
        build_file = Label("//third_party/cargo/remote:BUILD.cxx-1.0.71.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxx_gen__0_7_71",
        url = "https://crates.io/api/v1/crates/cxx-gen/0.7.71/download",
        type = "tar.gz",
        sha256 = "c606d018d5f8298464809ab01eb1aaf3efc2d6a984f527477cabda650f9f8688",
        strip_prefix = "cxx-gen-0.7.71",
        build_file = Label("//third_party/cargo/remote:BUILD.cxx-gen-0.7.71.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_cmd__1_0_71",
        url = "https://crates.io/api/v1/crates/cxxbridge-cmd/1.0.71/download",
        type = "tar.gz",
        sha256 = "384d7699599cc149694e38151d20820e8ab5550037526870bee8a27b069ed922",
        strip_prefix = "cxxbridge-cmd-1.0.71",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-cmd-1.0.71.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_flags__1_0_71",
        url = "https://crates.io/api/v1/crates/cxxbridge-flags/1.0.71/download",
        type = "tar.gz",
        sha256 = "0fef2b4ffdc935c973bc7817d541fc936fdc8a85194cfdd9c761aca8387edd48",
        strip_prefix = "cxxbridge-flags-1.0.71",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-flags-1.0.71.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_macro__1_0_71",
        url = "https://crates.io/api/v1/crates/cxxbridge-macro/1.0.71/download",
        type = "tar.gz",
        sha256 = "9d3a240a54f5526967ffae81fdcda1fc80564964220d90816960b2eae2eab7f4",
        strip_prefix = "cxxbridge-macro-1.0.71",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-macro-1.0.71.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__difflib__0_4_0",
        url = "https://crates.io/api/v1/crates/difflib/0.4.0/download",
        type = "tar.gz",
        sha256 = "6184e33543162437515c2e2b48714794e37845ec9851711914eec9d308f6ebe8",
        strip_prefix = "difflib-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.difflib-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__doc_comment__0_3_3",
        url = "https://crates.io/api/v1/crates/doc-comment/0.3.3/download",
        type = "tar.gz",
        sha256 = "fea41bba32d969b513997752735605054bc0dfa92b4c56bf1189f2e174be7a10",
        strip_prefix = "doc-comment-0.3.3",
        build_file = Label("//third_party/cargo/remote:BUILD.doc-comment-0.3.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__either__1_7_0",
        url = "https://crates.io/api/v1/crates/either/1.7.0/download",
        type = "tar.gz",
        sha256 = "3f107b87b6afc2a64fd13cac55fe06d6c8859f12d4b14cbcdd2c67d0976781be",
        strip_prefix = "either-1.7.0",
        build_file = Label("//third_party/cargo/remote:BUILD.either-1.7.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__env_logger__0_8_4",
        url = "https://crates.io/api/v1/crates/env_logger/0.8.4/download",
        type = "tar.gz",
        sha256 = "a19187fea3ac7e84da7dacf48de0c45d63c6a76f9490dae389aead16c243fce3",
        strip_prefix = "env_logger-0.8.4",
        build_file = Label("//third_party/cargo/remote:BUILD.env_logger-0.8.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__env_logger__0_9_0",
        url = "https://crates.io/api/v1/crates/env_logger/0.9.0/download",
        type = "tar.gz",
        sha256 = "0b2cf0344971ee6c64c31be0d530793fba457d322dfec2810c453d0ef228f9c3",
        strip_prefix = "env_logger-0.9.0",
        build_file = Label("//third_party/cargo/remote:BUILD.env_logger-0.9.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__fastrand__1_7_0",
        url = "https://crates.io/api/v1/crates/fastrand/1.7.0/download",
        type = "tar.gz",
        sha256 = "c3fcf0cee53519c866c09b5de1f6c56ff9d647101f81c1964fa632e148896cdf",
        strip_prefix = "fastrand-1.7.0",
        build_file = Label("//third_party/cargo/remote:BUILD.fastrand-1.7.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures__0_3_21",
        url = "https://crates.io/api/v1/crates/futures/0.3.21/download",
        type = "tar.gz",
        sha256 = "f73fe65f54d1e12b726f517d3e2135ca3125a437b6d998caf1962961f7172d9e",
        strip_prefix = "futures-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_channel__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-channel/0.3.21/download",
        type = "tar.gz",
        sha256 = "c3083ce4b914124575708913bca19bfe887522d6e2e6d0952943f5eac4a74010",
        strip_prefix = "futures-channel-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-channel-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_core__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-core/0.3.21/download",
        type = "tar.gz",
        sha256 = "0c09fd04b7e4073ac7156a9539b57a484a8ea920f79c7c675d05d289ab6110d3",
        strip_prefix = "futures-core-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-core-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_executor__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-executor/0.3.21/download",
        type = "tar.gz",
        sha256 = "9420b90cfa29e327d0429f19be13e7ddb68fa1cccb09d65e5706b8c7a749b8a6",
        strip_prefix = "futures-executor-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-executor-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_io__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-io/0.3.21/download",
        type = "tar.gz",
        sha256 = "fc4045962a5a5e935ee2fdedaa4e08284547402885ab326734432bed5d12966b",
        strip_prefix = "futures-io-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-io-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_macro__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-macro/0.3.21/download",
        type = "tar.gz",
        sha256 = "33c1e13800337f4d4d7a316bf45a567dbcb6ffe087f16424852d97e97a91f512",
        strip_prefix = "futures-macro-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-macro-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_sink__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-sink/0.3.21/download",
        type = "tar.gz",
        sha256 = "21163e139fa306126e6eedaf49ecdb4588f939600f0b1e770f4205ee4b7fa868",
        strip_prefix = "futures-sink-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-sink-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_task__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-task/0.3.21/download",
        type = "tar.gz",
        sha256 = "57c66a976bf5909d801bbef33416c41372779507e7a6b3a5e25e4749c58f776a",
        strip_prefix = "futures-task-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-task-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_util__0_3_21",
        url = "https://crates.io/api/v1/crates/futures-util/0.3.21/download",
        type = "tar.gz",
        sha256 = "d8b7abd5d659d9b90c8cba917f6ec750a74e2dc23902ef9cd4cc8c8b22e6036a",
        strip_prefix = "futures-util-0.3.21",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-util-0.3.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__gimli__0_26_1",
        url = "https://crates.io/api/v1/crates/gimli/0.26.1/download",
        type = "tar.gz",
        sha256 = "78cc372d058dcf6d5ecd98510e7fbc9e5aec4d21de70f65fea8fecebcd881bd4",
        strip_prefix = "gimli-0.26.1",
        build_file = Label("//third_party/cargo/remote:BUILD.gimli-0.26.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__glob__0_3_0",
        url = "https://crates.io/api/v1/crates/glob/0.3.0/download",
        type = "tar.gz",
        sha256 = "9b919933a397b79c37e33b77bb2aa3dc8eb6e165ad809e58ff75bc7db2e34574",
        strip_prefix = "glob-0.3.0",
        build_file = Label("//third_party/cargo/remote:BUILD.glob-0.3.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__hashbrown__0_12_2",
        url = "https://crates.io/api/v1/crates/hashbrown/0.12.2/download",
        type = "tar.gz",
        sha256 = "607c8a29735385251a339424dd462993c0fed8fa09d378f259377df08c126022",
        strip_prefix = "hashbrown-0.12.2",
        build_file = Label("//third_party/cargo/remote:BUILD.hashbrown-0.12.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__heck__0_4_0",
        url = "https://crates.io/api/v1/crates/heck/0.4.0/download",
        type = "tar.gz",
        sha256 = "2540771e65fc8cb83cd6e8a237f70c319bd5c29f78ed1084ba5d50eeac86f7f9",
        strip_prefix = "heck-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.heck-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__hermit_abi__0_1_19",
        url = "https://crates.io/api/v1/crates/hermit-abi/0.1.19/download",
        type = "tar.gz",
        sha256 = "62b467343b94ba476dcb2500d242dadbb39557df889310ac77c5d99100aaac33",
        strip_prefix = "hermit-abi-0.1.19",
        build_file = Label("//third_party/cargo/remote:BUILD.hermit-abi-0.1.19.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__humantime__2_1_0",
        url = "https://crates.io/api/v1/crates/humantime/2.1.0/download",
        type = "tar.gz",
        sha256 = "9a3a5bfb195931eeb336b2a7b4d761daec841b97f947d34394601737a7bba5e4",
        strip_prefix = "humantime-2.1.0",
        build_file = Label("//third_party/cargo/remote:BUILD.humantime-2.1.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__indexmap__1_9_1",
        url = "https://crates.io/api/v1/crates/indexmap/1.9.1/download",
        type = "tar.gz",
        sha256 = "10a35a97730320ffe8e2d410b5d3b69279b98d2c14bdb8b70ea89ecf7888d41e",
        strip_prefix = "indexmap-1.9.1",
        build_file = Label("//third_party/cargo/remote:BUILD.indexmap-1.9.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__indoc__1_0_6",
        url = "https://crates.io/api/v1/crates/indoc/1.0.6/download",
        type = "tar.gz",
        sha256 = "05a0bd019339e5d968b37855180087b7b9d512c5046fbd244cf8c95687927d6e",
        strip_prefix = "indoc-1.0.6",
        build_file = Label("//third_party/cargo/remote:BUILD.indoc-1.0.6.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__instant__0_1_12",
        url = "https://crates.io/api/v1/crates/instant/0.1.12/download",
        type = "tar.gz",
        sha256 = "7a5bbe824c507c5da5956355e86a746d82e0e1464f65d862cc5e71da70e94b2c",
        strip_prefix = "instant-0.1.12",
        build_file = Label("//third_party/cargo/remote:BUILD.instant-0.1.12.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__is_ci__1_1_1",
        url = "https://crates.io/api/v1/crates/is_ci/1.1.1/download",
        type = "tar.gz",
        sha256 = "616cde7c720bb2bb5824a224687d8f77bfd38922027f01d825cd7453be5099fb",
        strip_prefix = "is_ci-1.1.1",
        build_file = Label("//third_party/cargo/remote:BUILD.is_ci-1.1.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__itertools__0_10_3",
        url = "https://crates.io/api/v1/crates/itertools/0.10.3/download",
        type = "tar.gz",
        sha256 = "a9a9d19fa1e79b6215ff29b9d6880b706147f16e9b1dbb1e4e5947b5b02bc5e3",
        strip_prefix = "itertools-0.10.3",
        build_file = Label("//third_party/cargo/remote:BUILD.itertools-0.10.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__itertools__0_9_0",
        url = "https://crates.io/api/v1/crates/itertools/0.9.0/download",
        type = "tar.gz",
        sha256 = "284f18f85651fe11e8a991b2adb42cb078325c996ed026d994719efcfca1d54b",
        strip_prefix = "itertools-0.9.0",
        build_file = Label("//third_party/cargo/remote:BUILD.itertools-0.9.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__itoa__1_0_2",
        url = "https://crates.io/api/v1/crates/itoa/1.0.2/download",
        type = "tar.gz",
        sha256 = "112c678d4050afce233f4f2852bb2eb519230b3cf12f33585275537d7e41578d",
        strip_prefix = "itoa-1.0.2",
        build_file = Label("//third_party/cargo/remote:BUILD.itoa-1.0.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__lazy_static__1_4_0",
        url = "https://crates.io/api/v1/crates/lazy_static/1.4.0/download",
        type = "tar.gz",
        sha256 = "e2abad23fbc42b3700f2f279844dc832adb2b2eb069b2df918f455c4e18cc646",
        strip_prefix = "lazy_static-1.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.lazy_static-1.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__lazycell__1_3_0",
        url = "https://crates.io/api/v1/crates/lazycell/1.3.0/download",
        type = "tar.gz",
        sha256 = "830d08ce1d1d941e6b30645f1a0eb5643013d835ce3779a5fc208261dbe10f55",
        strip_prefix = "lazycell-1.3.0",
        build_file = Label("//third_party/cargo/remote:BUILD.lazycell-1.3.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__libc__0_2_126",
        url = "https://crates.io/api/v1/crates/libc/0.2.126/download",
        type = "tar.gz",
        sha256 = "349d5a591cd28b49e1d1037471617a32ddcda5731b99419008085f72d5a53836",
        strip_prefix = "libc-0.2.126",
        build_file = Label("//third_party/cargo/remote:BUILD.libc-0.2.126.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__libloading__0_6_3",
        url = "https://crates.io/api/v1/crates/libloading/0.6.3/download",
        type = "tar.gz",
        sha256 = "2443d8f0478b16759158b2f66d525991a05491138bc05814ef52a250148ef4f9",
        strip_prefix = "libloading-0.6.3",
        build_file = Label("//third_party/cargo/remote:BUILD.libloading-0.6.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__libloading__0_7_3",
        url = "https://crates.io/api/v1/crates/libloading/0.7.3/download",
        type = "tar.gz",
        sha256 = "efbc0f03f9a775e9f6aed295c6a1ba2253c5757a9e03d55c6caa46a681abcddd",
        strip_prefix = "libloading-0.7.3",
        build_file = Label("//third_party/cargo/remote:BUILD.libloading-0.7.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__link_cplusplus__1_0_6",
        url = "https://crates.io/api/v1/crates/link-cplusplus/1.0.6/download",
        type = "tar.gz",
        sha256 = "f8cae2cd7ba2f3f63938b9c724475dfb7b9861b545a90324476324ed21dbc8c8",
        strip_prefix = "link-cplusplus-1.0.6",
        build_file = Label("//third_party/cargo/remote:BUILD.link-cplusplus-1.0.6.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__log__0_4_17",
        url = "https://crates.io/api/v1/crates/log/0.4.17/download",
        type = "tar.gz",
        sha256 = "abb12e687cfb44aa40f41fc3978ef76448f9b6038cad6aef4259d3c095a2382e",
        strip_prefix = "log-0.4.17",
        build_file = Label("//third_party/cargo/remote:BUILD.log-0.4.17.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__memchr__2_5_0",
        url = "https://crates.io/api/v1/crates/memchr/2.5.0/download",
        type = "tar.gz",
        sha256 = "2dffe52ecf27772e601905b7522cb4ef790d2cc203488bbd0e2fe85fcb74566d",
        strip_prefix = "memchr-2.5.0",
        build_file = Label("//third_party/cargo/remote:BUILD.memchr-2.5.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__miette__4_7_1",
        url = "https://crates.io/api/v1/crates/miette/4.7.1/download",
        type = "tar.gz",
        sha256 = "1c90329e44f9208b55f45711f9558cec15d7ef8295cc65ecd6d4188ae8edc58c",
        strip_prefix = "miette-4.7.1",
        build_file = Label("//third_party/cargo/remote:BUILD.miette-4.7.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__miette_derive__4_7_1",
        url = "https://crates.io/api/v1/crates/miette-derive/4.7.1/download",
        type = "tar.gz",
        sha256 = "6b5bc45b761bcf1b5e6e6c4128cd93b84c218721a8d9b894aa0aff4ed180174c",
        strip_prefix = "miette-derive-4.7.1",
        build_file = Label("//third_party/cargo/remote:BUILD.miette-derive-4.7.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__minimal_lexical__0_2_1",
        url = "https://crates.io/api/v1/crates/minimal-lexical/0.2.1/download",
        type = "tar.gz",
        sha256 = "68354c5c6bd36d73ff3feceb05efa59b6acb7626617f4962be322a825e61f79a",
        strip_prefix = "minimal-lexical-0.2.1",
        build_file = Label("//third_party/cargo/remote:BUILD.minimal-lexical-0.2.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__miniz_oxide__0_5_3",
        url = "https://crates.io/api/v1/crates/miniz_oxide/0.5.3/download",
        type = "tar.gz",
        sha256 = "6f5c75688da582b8ffc1f1799e9db273f32133c49e048f614d22ec3256773ccc",
        strip_prefix = "miniz_oxide-0.5.3",
        build_file = Label("//third_party/cargo/remote:BUILD.miniz_oxide-0.5.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__moveit__0_5_0",
        url = "https://crates.io/api/v1/crates/moveit/0.5.0/download",
        type = "tar.gz",
        sha256 = "815d5988a1dd22f08bad572a83ee654563bb422ece5d5bce41f31ec49399dcb5",
        strip_prefix = "moveit-0.5.0",
        build_file = Label("//third_party/cargo/remote:BUILD.moveit-0.5.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__nom__5_1_2",
        url = "https://crates.io/api/v1/crates/nom/5.1.2/download",
        type = "tar.gz",
        sha256 = "ffb4262d26ed83a1c0a33a38fe2bb15797329c85770da05e6b828ddb782627af",
        strip_prefix = "nom-5.1.2",
        build_file = Label("//third_party/cargo/remote:BUILD.nom-5.1.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__nom__7_1_1",
        url = "https://crates.io/api/v1/crates/nom/7.1.1/download",
        type = "tar.gz",
        sha256 = "a8903e5a29a317527874d0402f867152a3d21c908bb0b933e416c65e301d4c36",
        strip_prefix = "nom-7.1.1",
        build_file = Label("//third_party/cargo/remote:BUILD.nom-7.1.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__object__0_29_0",
        url = "https://crates.io/api/v1/crates/object/0.29.0/download",
        type = "tar.gz",
        sha256 = "21158b2c33aa6d4561f1c0a6ea283ca92bc54802a93b263e910746d679a7eb53",
        strip_prefix = "object-0.29.0",
        build_file = Label("//third_party/cargo/remote:BUILD.object-0.29.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__once_cell__1_13_0",
        url = "https://crates.io/api/v1/crates/once_cell/1.13.0/download",
        type = "tar.gz",
        sha256 = "18a6dbe30758c9f83eb00cbea4ac95966305f5a7772f3f42ebfc7fc7eddbd8e1",
        strip_prefix = "once_cell-1.13.0",
        build_file = Label("//third_party/cargo/remote:BUILD.once_cell-1.13.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__os_str_bytes__6_2_0",
        url = "https://crates.io/api/v1/crates/os_str_bytes/6.2.0/download",
        type = "tar.gz",
        sha256 = "648001efe5d5c0102d8cea768e348da85d90af8ba91f0bea908f157951493cd4",
        strip_prefix = "os_str_bytes-6.2.0",
        build_file = Label("//third_party/cargo/remote:BUILD.os_str_bytes-6.2.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__owo_colors__3_4_0",
        url = "https://crates.io/api/v1/crates/owo-colors/3.4.0/download",
        type = "tar.gz",
        sha256 = "decf7381921fea4dcb2549c5667eda59b3ec297ab7e2b5fc33eac69d2e7da87b",
        strip_prefix = "owo-colors-3.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.owo-colors-3.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__pathdiff__0_2_1",
        url = "https://crates.io/api/v1/crates/pathdiff/0.2.1/download",
        type = "tar.gz",
        sha256 = "8835116a5c179084a830efb3adc117ab007512b535bc1a21c991d3b32a6b44dd",
        strip_prefix = "pathdiff-0.2.1",
        build_file = Label("//third_party/cargo/remote:BUILD.pathdiff-0.2.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__peeking_take_while__0_1_2",
        url = "https://crates.io/api/v1/crates/peeking_take_while/0.1.2/download",
        type = "tar.gz",
        sha256 = "19b17cddbe7ec3f8bc800887bab5e717348c95ea2ca0b1bf0837fb964dc67099",
        strip_prefix = "peeking_take_while-0.1.2",
        build_file = Label("//third_party/cargo/remote:BUILD.peeking_take_while-0.1.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__pin_project_lite__0_2_9",
        url = "https://crates.io/api/v1/crates/pin-project-lite/0.2.9/download",
        type = "tar.gz",
        sha256 = "e0a7ae3ac2f1173085d398531c705756c94a4c56843785df85a60c1a0afac116",
        strip_prefix = "pin-project-lite-0.2.9",
        build_file = Label("//third_party/cargo/remote:BUILD.pin-project-lite-0.2.9.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__pin_utils__0_1_0",
        url = "https://crates.io/api/v1/crates/pin-utils/0.1.0/download",
        type = "tar.gz",
        sha256 = "8b870d8c151b6f2fb93e84a13146138f05d02ed11c7e7c54f8826aaaf7c9f184",
        strip_prefix = "pin-utils-0.1.0",
        build_file = Label("//third_party/cargo/remote:BUILD.pin-utils-0.1.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__predicates__2_1_1",
        url = "https://crates.io/api/v1/crates/predicates/2.1.1/download",
        type = "tar.gz",
        sha256 = "a5aab5be6e4732b473071984b3164dbbfb7a3674d30ea5ff44410b6bcd960c3c",
        strip_prefix = "predicates-2.1.1",
        build_file = Label("//third_party/cargo/remote:BUILD.predicates-2.1.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__predicates_core__1_0_3",
        url = "https://crates.io/api/v1/crates/predicates-core/1.0.3/download",
        type = "tar.gz",
        sha256 = "da1c2388b1513e1b605fcec39a95e0a9e8ef088f71443ef37099fa9ae6673fcb",
        strip_prefix = "predicates-core-1.0.3",
        build_file = Label("//third_party/cargo/remote:BUILD.predicates-core-1.0.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__predicates_tree__1_0_5",
        url = "https://crates.io/api/v1/crates/predicates-tree/1.0.5/download",
        type = "tar.gz",
        sha256 = "4d86de6de25020a36c6d3643a86d9a6a9f552107c0559c60ea03551b5e16c032",
        strip_prefix = "predicates-tree-1.0.5",
        build_file = Label("//third_party/cargo/remote:BUILD.predicates-tree-1.0.5.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__proc_macro_error__1_0_4",
        url = "https://crates.io/api/v1/crates/proc-macro-error/1.0.4/download",
        type = "tar.gz",
        sha256 = "da25490ff9892aab3fcf7c36f08cfb902dd3e71ca0f9f9517bea02a73a5ce38c",
        strip_prefix = "proc-macro-error-1.0.4",
        build_file = Label("//third_party/cargo/remote:BUILD.proc-macro-error-1.0.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__proc_macro_error_attr__1_0_4",
        url = "https://crates.io/api/v1/crates/proc-macro-error-attr/1.0.4/download",
        type = "tar.gz",
        sha256 = "a1be40180e52ecc98ad80b184934baf3d0d29f979574e439af5a55274b35f869",
        strip_prefix = "proc-macro-error-attr-1.0.4",
        build_file = Label("//third_party/cargo/remote:BUILD.proc-macro-error-attr-1.0.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__proc_macro2__1_0_40",
        url = "https://crates.io/api/v1/crates/proc-macro2/1.0.40/download",
        type = "tar.gz",
        sha256 = "dd96a1e8ed2596c337f8eae5f24924ec83f5ad5ab21ea8e455d3566c69fbcaf7",
        strip_prefix = "proc-macro2-1.0.40",
        build_file = Label("//third_party/cargo/remote:BUILD.proc-macro2-1.0.40.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__quote__1_0_20",
        url = "https://crates.io/api/v1/crates/quote/1.0.20/download",
        type = "tar.gz",
        sha256 = "3bcdf212e9776fbcb2d23ab029360416bb1706b1aea2d1a5ba002727cbcab804",
        strip_prefix = "quote-1.0.20",
        build_file = Label("//third_party/cargo/remote:BUILD.quote-1.0.20.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__redox_syscall__0_2_13",
        url = "https://crates.io/api/v1/crates/redox_syscall/0.2.13/download",
        type = "tar.gz",
        sha256 = "62f25bc4c7e55e0b0b7a1d43fb893f4fa1361d0abe38b9ce4f323c2adfe6ef42",
        strip_prefix = "redox_syscall-0.2.13",
        build_file = Label("//third_party/cargo/remote:BUILD.redox_syscall-0.2.13.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__regex__1_6_0",
        url = "https://crates.io/api/v1/crates/regex/1.6.0/download",
        type = "tar.gz",
        sha256 = "4c4eb3267174b8c6c2f654116623910a0fef09c4753f8dd83db29c48a0df988b",
        strip_prefix = "regex-1.6.0",
        build_file = Label("//third_party/cargo/remote:BUILD.regex-1.6.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__regex_automata__0_1_10",
        url = "https://crates.io/api/v1/crates/regex-automata/0.1.10/download",
        type = "tar.gz",
        sha256 = "6c230d73fb8d8c1b9c0b3135c5142a8acee3a0558fb8db5cf1cb65f8d7862132",
        strip_prefix = "regex-automata-0.1.10",
        build_file = Label("//third_party/cargo/remote:BUILD.regex-automata-0.1.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__regex_syntax__0_6_27",
        url = "https://crates.io/api/v1/crates/regex-syntax/0.6.27/download",
        type = "tar.gz",
        sha256 = "a3f87b73ce11b1619a3c6332f45341e0047173771e8b8b73f87bfeefb7b56244",
        strip_prefix = "regex-syntax-0.6.27",
        build_file = Label("//third_party/cargo/remote:BUILD.regex-syntax-0.6.27.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__remove_dir_all__0_5_3",
        url = "https://crates.io/api/v1/crates/remove_dir_all/0.5.3/download",
        type = "tar.gz",
        sha256 = "3acd125665422973a33ac9d3dd2df85edad0f4ae9b00dafb1a05e43a9f5ef8e7",
        strip_prefix = "remove_dir_all-0.5.3",
        build_file = Label("//third_party/cargo/remote:BUILD.remove_dir_all-0.5.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rust_info__0_3_2",
        url = "https://crates.io/api/v1/crates/rust_info/0.3.2/download",
        type = "tar.gz",
        sha256 = "821495e93d15e4433347b3a72e97005f1d8a620dc88d46637fecfcb16e98043d",
        strip_prefix = "rust_info-0.3.2",
        build_file = Label("//third_party/cargo/remote:BUILD.rust_info-0.3.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rustc_demangle__0_1_21",
        url = "https://crates.io/api/v1/crates/rustc-demangle/0.1.21/download",
        type = "tar.gz",
        sha256 = "7ef03e0a2b150c7a90d01faf6254c9c48a41e95fb2a8c2ac1c6f0d2b9aefc342",
        strip_prefix = "rustc-demangle-0.1.21",
        build_file = Label("//third_party/cargo/remote:BUILD.rustc-demangle-0.1.21.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rustc_hash__1_1_0",
        url = "https://crates.io/api/v1/crates/rustc-hash/1.1.0/download",
        type = "tar.gz",
        sha256 = "08d43f7aa6b08d49f382cde6a7982047c3426db949b1424bc4b7ec9ae12c6ce2",
        strip_prefix = "rustc-hash-1.1.0",
        build_file = Label("//third_party/cargo/remote:BUILD.rustc-hash-1.1.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rustc_version__0_4_0",
        url = "https://crates.io/api/v1/crates/rustc_version/0.4.0/download",
        type = "tar.gz",
        strip_prefix = "rustc_version-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.rustc_version-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rustversion__1_0_8",
        url = "https://crates.io/api/v1/crates/rustversion/1.0.8/download",
        type = "tar.gz",
        sha256 = "24c8ad4f0c00e1eb5bc7614d236a7f1300e3dbd76b68cac8e06fb00b015ad8d8",
        strip_prefix = "rustversion-1.0.8",
        build_file = Label("//third_party/cargo/remote:BUILD.rustversion-1.0.8.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__ryu__1_0_10",
        url = "https://crates.io/api/v1/crates/ryu/1.0.10/download",
        type = "tar.gz",
        sha256 = "f3f6f92acf49d1b98f7a81226834412ada05458b7364277387724a237f062695",
        strip_prefix = "ryu-1.0.10",
        build_file = Label("//third_party/cargo/remote:BUILD.ryu-1.0.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__semver__1_0_15",
        url = "https://crates.io/api/v1/crates/semver/1.0.15/download",
        type = "tar.gz",
        strip_prefix = "semver-1.0.15",
        build_file = Label("//third_party/cargo/remote:BUILD.semver-1.0.15.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde__1_0_139",
        url = "https://crates.io/api/v1/crates/serde/1.0.139/download",
        type = "tar.gz",
        sha256 = "0171ebb889e45aa68b44aee0859b3eede84c6f5f5c228e6f140c0b2a0a46cad6",
        strip_prefix = "serde-1.0.139",
        build_file = Label("//third_party/cargo/remote:BUILD.serde-1.0.139.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde_derive__1_0_139",
        url = "https://crates.io/api/v1/crates/serde_derive/1.0.139/download",
        type = "tar.gz",
        sha256 = "dc1d3230c1de7932af58ad8ffbe1d784bd55efd5a9d84ac24f69c72d83543dfb",
        strip_prefix = "serde_derive-1.0.139",
        build_file = Label("//third_party/cargo/remote:BUILD.serde_derive-1.0.139.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde_json__1_0_82",
        url = "https://crates.io/api/v1/crates/serde_json/1.0.82/download",
        type = "tar.gz",
        sha256 = "82c2c1fdcd807d1098552c5b9a36e425e42e9fbd7c6a37a8425f390f781f7fa7",
        strip_prefix = "serde_json-1.0.82",
        build_file = Label("//third_party/cargo/remote:BUILD.serde_json-1.0.82.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__shlex__1_1_0",
        url = "https://crates.io/api/v1/crates/shlex/1.1.0/download",
        type = "tar.gz",
        sha256 = "43b2853a4d09f215c24cc5489c992ce46052d359b5109343cbafbf26bc62f8a3",
        strip_prefix = "shlex-1.1.0",
        build_file = Label("//third_party/cargo/remote:BUILD.shlex-1.1.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__slab__0_4_6",
        url = "https://crates.io/api/v1/crates/slab/0.4.6/download",
        type = "tar.gz",
        sha256 = "eb703cfe953bccee95685111adeedb76fabe4e97549a58d16f03ea7b9367bb32",
        strip_prefix = "slab-0.4.6",
        build_file = Label("//third_party/cargo/remote:BUILD.slab-0.4.6.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__smallvec__1_9_0",
        url = "https://crates.io/api/v1/crates/smallvec/1.9.0/download",
        type = "tar.gz",
        sha256 = "2fd0db749597d91ff862fd1d55ea87f7855a744a8425a64695b6fca237d1dad1",
        strip_prefix = "smallvec-1.9.0",
        build_file = Label("//third_party/cargo/remote:BUILD.smallvec-1.9.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__smawk__0_3_1",
        url = "https://crates.io/api/v1/crates/smawk/0.3.1/download",
        type = "tar.gz",
        sha256 = "f67ad224767faa3c7d8b6d91985b78e70a1324408abcb1cfcc2be4c06bc06043",
        strip_prefix = "smawk-0.3.1",
        build_file = Label("//third_party/cargo/remote:BUILD.smawk-0.3.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__strsim__0_10_0",
        url = "https://crates.io/api/v1/crates/strsim/0.10.0/download",
        type = "tar.gz",
        sha256 = "73473c0e59e6d5812c5dfe2a064a6444949f089e20eec9a2e5506596494e4623",
        strip_prefix = "strsim-0.10.0",
        build_file = Label("//third_party/cargo/remote:BUILD.strsim-0.10.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__strsim__0_8_0",
        url = "https://crates.io/api/v1/crates/strsim/0.8.0/download",
        type = "tar.gz",
        sha256 = "8ea5119cdb4c55b55d432abb513a0429384878c15dde60cc77b1c99de1a95a6a",
        strip_prefix = "strsim-0.8.0",
        build_file = Label("//third_party/cargo/remote:BUILD.strsim-0.8.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__strum_macros__0_24_2",
        url = "https://crates.io/api/v1/crates/strum_macros/0.24.2/download",
        type = "tar.gz",
        sha256 = "4faebde00e8ff94316c01800f9054fd2ba77d30d9e922541913051d1d978918b",
        strip_prefix = "strum_macros-0.24.2",
        build_file = Label("//third_party/cargo/remote:BUILD.strum_macros-0.24.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__supports_color__1_3_0",
        url = "https://crates.io/api/v1/crates/supports-color/1.3.0/download",
        type = "tar.gz",
        sha256 = "4872ced36b91d47bae8a214a683fe54e7078875b399dfa251df346c9b547d1f9",
        strip_prefix = "supports-color-1.3.0",
        build_file = Label("//third_party/cargo/remote:BUILD.supports-color-1.3.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__supports_hyperlinks__1_2_0",
        url = "https://crates.io/api/v1/crates/supports-hyperlinks/1.2.0/download",
        type = "tar.gz",
        sha256 = "590b34f7c5f01ecc9d78dba4b3f445f31df750a67621cf31626f3b7441ce6406",
        strip_prefix = "supports-hyperlinks-1.2.0",
        build_file = Label("//third_party/cargo/remote:BUILD.supports-hyperlinks-1.2.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__supports_unicode__1_0_2",
        url = "https://crates.io/api/v1/crates/supports-unicode/1.0.2/download",
        type = "tar.gz",
        sha256 = "a8b945e45b417b125a8ec51f1b7df2f8df7920367700d1f98aedd21e5735f8b2",
        strip_prefix = "supports-unicode-1.0.2",
        build_file = Label("//third_party/cargo/remote:BUILD.supports-unicode-1.0.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__syn__1_0_98",
        url = "https://crates.io/api/v1/crates/syn/1.0.98/download",
        type = "tar.gz",
        sha256 = "c50aef8a904de4c23c788f104b7dddc7d6f79c647c7c8ce4cc8f73eb0ca773dd",
        strip_prefix = "syn-1.0.98",
        build_file = Label("//third_party/cargo/remote:BUILD.syn-1.0.98.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__tempfile__3_3_0",
        url = "https://crates.io/api/v1/crates/tempfile/3.3.0/download",
        type = "tar.gz",
        sha256 = "5cdb1ef4eaeeaddc8fbd371e5017057064af0911902ef36b39801f67cc6d79e4",
        strip_prefix = "tempfile-3.3.0",
        build_file = Label("//third_party/cargo/remote:BUILD.tempfile-3.3.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__termcolor__1_1_3",
        url = "https://crates.io/api/v1/crates/termcolor/1.1.3/download",
        type = "tar.gz",
        sha256 = "bab24d30b911b2376f3a13cc2cd443142f0c81dda04c118693e35b3835757755",
        strip_prefix = "termcolor-1.1.3",
        build_file = Label("//third_party/cargo/remote:BUILD.termcolor-1.1.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__terminal_size__0_1_17",
        url = "https://crates.io/api/v1/crates/terminal_size/0.1.17/download",
        type = "tar.gz",
        sha256 = "633c1a546cee861a1a6d0dc69ebeca693bf4296661ba7852b9d21d159e0506df",
        strip_prefix = "terminal_size-0.1.17",
        build_file = Label("//third_party/cargo/remote:BUILD.terminal_size-0.1.17.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__termtree__0_2_4",
        url = "https://crates.io/api/v1/crates/termtree/0.2.4/download",
        type = "tar.gz",
        sha256 = "507e9898683b6c43a9aa55b64259b721b52ba226e0f3779137e50ad114a4c90b",
        strip_prefix = "termtree-0.2.4",
        build_file = Label("//third_party/cargo/remote:BUILD.termtree-0.2.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__test_log__0_2_10",
        url = "https://crates.io/api/v1/crates/test-log/0.2.10/download",
        type = "tar.gz",
        sha256 = "4235dbf7ea878b3ef12dea20a59c134b405a66aafc4fc2c7b9935916e289e735",
        strip_prefix = "test-log-0.2.10",
        build_file = Label("//third_party/cargo/remote:BUILD.test-log-0.2.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__textwrap__0_11_0",
        url = "https://crates.io/api/v1/crates/textwrap/0.11.0/download",
        type = "tar.gz",
        sha256 = "d326610f408c7a4eb6f51c37c330e496b08506c9457c9d34287ecc38809fb060",
        strip_prefix = "textwrap-0.11.0",
        build_file = Label("//third_party/cargo/remote:BUILD.textwrap-0.11.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__textwrap__0_15_0",
        url = "https://crates.io/api/v1/crates/textwrap/0.15.0/download",
        type = "tar.gz",
        sha256 = "b1141d4d61095b28419e22cb0bbf02755f5e54e0526f97f1e3d1d160e60885fb",
        strip_prefix = "textwrap-0.15.0",
        build_file = Label("//third_party/cargo/remote:BUILD.textwrap-0.15.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__thiserror__1_0_31",
        url = "https://crates.io/api/v1/crates/thiserror/1.0.31/download",
        type = "tar.gz",
        sha256 = "bd829fe32373d27f76265620b5309d0340cb8550f523c1dda251d6298069069a",
        strip_prefix = "thiserror-1.0.31",
        build_file = Label("//third_party/cargo/remote:BUILD.thiserror-1.0.31.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__thiserror_impl__1_0_31",
        url = "https://crates.io/api/v1/crates/thiserror-impl/1.0.31/download",
        type = "tar.gz",
        sha256 = "0396bc89e626244658bef819e22d0cc459e795a5ebe878e6ec336d1674a8d79a",
        strip_prefix = "thiserror-impl-1.0.31",
        build_file = Label("//third_party/cargo/remote:BUILD.thiserror-impl-1.0.31.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__toml__0_5_9",
        url = "https://crates.io/api/v1/crates/toml/0.5.9/download",
        type = "tar.gz",
        sha256 = "8d82e1a7758622a465f8cee077614c73484dac5b836c02ff6a40d5d1010324d7",
        strip_prefix = "toml-0.5.9",
        build_file = Label("//third_party/cargo/remote:BUILD.toml-0.5.9.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__trybuild__1_0_63",
        url = "https://crates.io/api/v1/crates/trybuild/1.0.63/download",
        type = "tar.gz",
        sha256 = "764b9e244b482a9b81bde596aa37aa6f1347bf8007adab25e59f901b32b4e0a0",
        strip_prefix = "trybuild-1.0.63",
        build_file = Label("//third_party/cargo/remote:BUILD.trybuild-1.0.63.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_ident__1_0_2",
        url = "https://crates.io/api/v1/crates/unicode-ident/1.0.2/download",
        type = "tar.gz",
        sha256 = "15c61ba63f9235225a22310255a29b806b907c9b8c964bcbd0a2c70f3f2deea7",
        strip_prefix = "unicode-ident-1.0.2",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-ident-1.0.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_linebreak__0_1_2",
        url = "https://crates.io/api/v1/crates/unicode-linebreak/0.1.2/download",
        type = "tar.gz",
        sha256 = "3a52dcaab0c48d931f7cc8ef826fa51690a08e1ea55117ef26f89864f532383f",
        strip_prefix = "unicode-linebreak-0.1.2",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-linebreak-0.1.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_width__0_1_9",
        url = "https://crates.io/api/v1/crates/unicode-width/0.1.9/download",
        type = "tar.gz",
        sha256 = "3ed742d4ea2bd1176e236172c8429aaf54486e7ac098db29ffe6529e0ce50973",
        strip_prefix = "unicode-width-0.1.9",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-width-0.1.9.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__uuid__1_1_2",
        url = "https://crates.io/api/v1/crates/uuid/1.1.2/download",
        type = "tar.gz",
        sha256 = "dd6469f4314d5f1ffec476e05f17cc9a78bc7a27a6a857842170bdf8d6f98d2f",
        strip_prefix = "uuid-1.1.2",
        build_file = Label("//third_party/cargo/remote:BUILD.uuid-1.1.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__vec_map__0_8_2",
        url = "https://crates.io/api/v1/crates/vec_map/0.8.2/download",
        type = "tar.gz",
        sha256 = "f1bddf1187be692e79c5ffeab891132dfb0f236ed36a43c7ed39f1165ee20191",
        strip_prefix = "vec_map-0.8.2",
        build_file = Label("//third_party/cargo/remote:BUILD.vec_map-0.8.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__version_check__0_9_4",
        url = "https://crates.io/api/v1/crates/version_check/0.9.4/download",
        type = "tar.gz",
        sha256 = "49874b5167b65d7193b8aba1567f5c7d93d001cafc34600cee003eda787e483f",
        strip_prefix = "version_check-0.9.4",
        build_file = Label("//third_party/cargo/remote:BUILD.version_check-0.9.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__wait_timeout__0_2_0",
        url = "https://crates.io/api/v1/crates/wait-timeout/0.2.0/download",
        type = "tar.gz",
        sha256 = "9f200f5b12eb75f8c1ed65abd4b2db8a6e1b138a20de009dacee265a2498f3f6",
        strip_prefix = "wait-timeout-0.2.0",
        build_file = Label("//third_party/cargo/remote:BUILD.wait-timeout-0.2.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__which__3_1_1",
        url = "https://crates.io/api/v1/crates/which/3.1.1/download",
        type = "tar.gz",
        sha256 = "d011071ae14a2f6671d0b74080ae0cd8ebf3a6f8c9589a2cd45f23126fe29724",
        strip_prefix = "which-3.1.1",
        build_file = Label("//third_party/cargo/remote:BUILD.which-3.1.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__which__4_2_5",
        url = "https://crates.io/api/v1/crates/which/4.2.5/download",
        type = "tar.gz",
        sha256 = "5c4fb54e6113b6a8772ee41c3404fb0301ac79604489467e0a9ce1f3e97c24ae",
        strip_prefix = "which-4.2.5",
        build_file = Label("//third_party/cargo/remote:BUILD.which-4.2.5.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__winapi__0_3_9",
        url = "https://crates.io/api/v1/crates/winapi/0.3.9/download",
        type = "tar.gz",
        sha256 = "5c839a674fcd7a98952e593242ea400abe93992746761e38641405d28b00f419",
        strip_prefix = "winapi-0.3.9",
        build_file = Label("//third_party/cargo/remote:BUILD.winapi-0.3.9.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__winapi_i686_pc_windows_gnu__0_4_0",
        url = "https://crates.io/api/v1/crates/winapi-i686-pc-windows-gnu/0.4.0/download",
        type = "tar.gz",
        sha256 = "ac3b87c63620426dd9b991e5ce0329eff545bccbbb34f3be09ff6fb6ab51b7b6",
        strip_prefix = "winapi-i686-pc-windows-gnu-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.winapi-i686-pc-windows-gnu-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__winapi_util__0_1_5",
        url = "https://crates.io/api/v1/crates/winapi-util/0.1.5/download",
        type = "tar.gz",
        sha256 = "70ec6ce85bb158151cae5e5c87f95a8e97d2c0c4b001223f33a334e3ce5de178",
        strip_prefix = "winapi-util-0.1.5",
        build_file = Label("//third_party/cargo/remote:BUILD.winapi-util-0.1.5.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__winapi_x86_64_pc_windows_gnu__0_4_0",
        url = "https://crates.io/api/v1/crates/winapi-x86_64-pc-windows-gnu/0.4.0/download",
        type = "tar.gz",
        sha256 = "712e227841d057c1ee1cd2fb22fa7e5a5461ae8e48fa2ca79ec42cfc1931183f",
        strip_prefix = "winapi-x86_64-pc-windows-gnu-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.winapi-x86_64-pc-windows-gnu-0.4.0.bazel"),
    )
