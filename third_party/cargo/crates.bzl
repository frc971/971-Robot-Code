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
        name = "raze__addr2line__0_19_0",
        url = "https://crates.io/api/v1/crates/addr2line/0.19.0/download",
        type = "tar.gz",
        sha256 = "a76fd60b23679b7d19bd066031410fb7e458ccc5e958eb5c325888ce4baedc97",
        strip_prefix = "addr2line-0.19.0",
        build_file = Label("//third_party/cargo/remote:BUILD.addr2line-0.19.0.bazel"),
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
        name = "raze__ahash__0_7_6",
        url = "https://crates.io/api/v1/crates/ahash/0.7.6/download",
        type = "tar.gz",
        sha256 = "fcb51a0695d8f838b1ee009b3fbf66bda078cd64590202a864a8f3e8c4315c47",
        strip_prefix = "ahash-0.7.6",
        build_file = Label("//third_party/cargo/remote:BUILD.ahash-0.7.6.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__aho_corasick__0_7_20",
        url = "https://crates.io/api/v1/crates/aho-corasick/0.7.20/download",
        type = "tar.gz",
        sha256 = "cc936419f96fa211c1b9166887b38e5e40b19958e5b895be7c1f93adec7071ac",
        strip_prefix = "aho-corasick-0.7.20",
        build_file = Label("//third_party/cargo/remote:BUILD.aho-corasick-0.7.20.bazel"),
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
        name = "raze__anyhow__1_0_68",
        url = "https://crates.io/api/v1/crates/anyhow/1.0.68/download",
        type = "tar.gz",
        sha256 = "2cb2f989d18dd141ab8ae82f64d1a8cdd37e0840f73a406896cf5e99502fab61",
        strip_prefix = "anyhow-1.0.68",
        build_file = Label("//third_party/cargo/remote:BUILD.anyhow-1.0.68.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__aquamarine__0_1_12",
        url = "https://crates.io/api/v1/crates/aquamarine/0.1.12/download",
        type = "tar.gz",
        sha256 = "a941c39708478e8eea39243b5983f1c42d2717b3620ee91f4a52115fd02ac43f",
        strip_prefix = "aquamarine-0.1.12",
        build_file = Label("//third_party/cargo/remote:BUILD.aquamarine-0.1.12.bazel"),
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
        name = "raze__backtrace__0_3_67",
        url = "https://crates.io/api/v1/crates/backtrace/0.3.67/download",
        type = "tar.gz",
        sha256 = "233d376d6d185f2a3093e58f283f60f880315b6c60075b01f36b3b85154564ca",
        strip_prefix = "backtrace-0.3.67",
        build_file = Label("//third_party/cargo/remote:BUILD.backtrace-0.3.67.bazel"),
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
        name = "raze__cc__1_0_78",
        url = "https://crates.io/api/v1/crates/cc/1.0.78/download",
        type = "tar.gz",
        sha256 = "a20104e2335ce8a659d6dd92a51a767a0c062599c73b343fd152cb401e828c3d",
        strip_prefix = "cc-1.0.78",
        build_file = Label("//third_party/cargo/remote:BUILD.cc-1.0.78.bazel"),
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
        name = "raze__clang_sys__1_4_0",
        url = "https://crates.io/api/v1/crates/clang-sys/1.4.0/download",
        type = "tar.gz",
        sha256 = "fa2e27ae6ab525c3d369ded447057bca5438d86dc3a68f6faafb8269ba82ebf3",
        strip_prefix = "clang-sys-1.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.clang-sys-1.4.0.bazel"),
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
        name = "raze__clap__3_2_23",
        url = "https://crates.io/api/v1/crates/clap/3.2.23/download",
        type = "tar.gz",
        sha256 = "71655c45cb9845d3270c9d6df84ebe72b4dad3c2ba3f7023ad47c144e4e473a5",
        strip_prefix = "clap-3.2.23",
        build_file = Label("//third_party/cargo/remote:BUILD.clap-3.2.23.bazel"),
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
        name = "raze__cxx__1_0_85",
        url = "https://crates.io/api/v1/crates/cxx/1.0.85/download",
        type = "tar.gz",
        sha256 = "5add3fc1717409d029b20c5b6903fc0c0b02fa6741d820054f4a2efa5e5816fd",
        strip_prefix = "cxx-1.0.85",
        build_file = Label("//third_party/cargo/remote:BUILD.cxx-1.0.85.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxx_gen__0_7_85",
        url = "https://crates.io/api/v1/crates/cxx-gen/0.7.85/download",
        type = "tar.gz",
        sha256 = "ccca653bd8a21c5cfe696cd5347729d43f651298459b22e57c60fbae1cd49fec",
        strip_prefix = "cxx-gen-0.7.85",
        build_file = Label("//third_party/cargo/remote:BUILD.cxx-gen-0.7.85.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_cmd__1_0_77",
        url = "https://crates.io/api/v1/crates/cxxbridge-cmd/1.0.77/download",
        type = "tar.gz",
        sha256 = "de38cfccf77ed5c524784e2bcbb8863287acd00af2f5d522370f27257ef86307",
        strip_prefix = "cxxbridge-cmd-1.0.77",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-cmd-1.0.77.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_flags__1_0_85",
        url = "https://crates.io/api/v1/crates/cxxbridge-flags/1.0.85/download",
        type = "tar.gz",
        sha256 = "69a3e162fde4e594ed2b07d0f83c6c67b745e7f28ce58c6df5e6b6bef99dfb59",
        strip_prefix = "cxxbridge-flags-1.0.85",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-flags-1.0.85.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__cxxbridge_macro__1_0_85",
        url = "https://crates.io/api/v1/crates/cxxbridge-macro/1.0.85/download",
        type = "tar.gz",
        sha256 = "3e7e2adeb6a0d4a282e581096b06e1791532b7d576dcde5ccd9382acf55db8e6",
        strip_prefix = "cxxbridge-macro-1.0.85",
        build_file = Label("//third_party/cargo/remote:BUILD.cxxbridge-macro-1.0.85.bazel"),
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
        name = "raze__either__1_8_0",
        url = "https://crates.io/api/v1/crates/either/1.8.0/download",
        type = "tar.gz",
        sha256 = "90e5c1c8368803113bf0c9584fc495a58b86dc8a29edbf8fe877d21d9507e797",
        strip_prefix = "either-1.8.0",
        build_file = Label("//third_party/cargo/remote:BUILD.either-1.8.0.bazel"),
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
        name = "raze__env_logger__0_9_3",
        url = "https://crates.io/api/v1/crates/env_logger/0.9.3/download",
        type = "tar.gz",
        sha256 = "a12e6657c4c97ebab115a42dcee77225f7f482cdd841cf7088c657a42e9e00e7",
        strip_prefix = "env_logger-0.9.3",
        build_file = Label("//third_party/cargo/remote:BUILD.env_logger-0.9.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__fastrand__1_8_0",
        url = "https://crates.io/api/v1/crates/fastrand/1.8.0/download",
        type = "tar.gz",
        sha256 = "a7a407cfaa3385c4ae6b23e84623d48c2798d06e3e6a1878f7f59f17b3f86499",
        strip_prefix = "fastrand-1.8.0",
        build_file = Label("//third_party/cargo/remote:BUILD.fastrand-1.8.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures__0_3_25",
        url = "https://crates.io/api/v1/crates/futures/0.3.25/download",
        type = "tar.gz",
        sha256 = "38390104763dc37a5145a53c29c63c1290b5d316d6086ec32c293f6736051bb0",
        strip_prefix = "futures-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_channel__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-channel/0.3.25/download",
        type = "tar.gz",
        sha256 = "52ba265a92256105f45b719605a571ffe2d1f0fea3807304b522c1d778f79eed",
        strip_prefix = "futures-channel-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-channel-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_core__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-core/0.3.25/download",
        type = "tar.gz",
        sha256 = "04909a7a7e4633ae6c4a9ab280aeb86da1236243a77b694a49eacd659a4bd3ac",
        strip_prefix = "futures-core-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-core-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_executor__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-executor/0.3.25/download",
        type = "tar.gz",
        sha256 = "7acc85df6714c176ab5edf386123fafe217be88c0840ec11f199441134a074e2",
        strip_prefix = "futures-executor-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-executor-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_io__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-io/0.3.25/download",
        type = "tar.gz",
        sha256 = "00f5fb52a06bdcadeb54e8d3671f8888a39697dcb0b81b23b55174030427f4eb",
        strip_prefix = "futures-io-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-io-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_macro__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-macro/0.3.25/download",
        type = "tar.gz",
        sha256 = "bdfb8ce053d86b91919aad980c220b1fb8401a9394410e1c289ed7e66b61835d",
        strip_prefix = "futures-macro-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-macro-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_sink__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-sink/0.3.25/download",
        type = "tar.gz",
        sha256 = "39c15cf1a4aa79df40f1bb462fb39676d0ad9e366c2a33b590d7c66f4f81fcf9",
        strip_prefix = "futures-sink-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-sink-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_task__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-task/0.3.25/download",
        type = "tar.gz",
        sha256 = "2ffb393ac5d9a6eaa9d3fdf37ae2776656b706e200c8e16b1bdb227f5198e6ea",
        strip_prefix = "futures-task-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-task-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__futures_util__0_3_25",
        url = "https://crates.io/api/v1/crates/futures-util/0.3.25/download",
        type = "tar.gz",
        sha256 = "197676987abd2f9cadff84926f410af1c183608d36641465df73ae8211dc65d6",
        strip_prefix = "futures-util-0.3.25",
        build_file = Label("//third_party/cargo/remote:BUILD.futures-util-0.3.25.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__getrandom__0_2_8",
        url = "https://crates.io/api/v1/crates/getrandom/0.2.8/download",
        type = "tar.gz",
        sha256 = "c05aeb6a22b8f62540c194aac980f2115af067bfe15a0734d7277a768d396b31",
        strip_prefix = "getrandom-0.2.8",
        build_file = Label("//third_party/cargo/remote:BUILD.getrandom-0.2.8.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__gimli__0_27_0",
        url = "https://crates.io/api/v1/crates/gimli/0.27.0/download",
        type = "tar.gz",
        sha256 = "dec7af912d60cdbd3677c1af9352ebae6fb8394d165568a2234df0fa00f87793",
        strip_prefix = "gimli-0.27.0",
        build_file = Label("//third_party/cargo/remote:BUILD.gimli-0.27.0.bazel"),
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
        name = "raze__hashbrown__0_12_3",
        url = "https://crates.io/api/v1/crates/hashbrown/0.12.3/download",
        type = "tar.gz",
        sha256 = "8a9ee70c43aaf417c914396645a0fa852624801b24ebb7ae78fe8272889ac888",
        strip_prefix = "hashbrown-0.12.3",
        build_file = Label("//third_party/cargo/remote:BUILD.hashbrown-0.12.3.bazel"),
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
        name = "raze__indexmap__1_9_2",
        url = "https://crates.io/api/v1/crates/indexmap/1.9.2/download",
        type = "tar.gz",
        sha256 = "1885e79c1fc4b10f0e172c475f458b7f7b93061064d98c3293e98c5ba0c8b399",
        strip_prefix = "indexmap-1.9.2",
        build_file = Label("//third_party/cargo/remote:BUILD.indexmap-1.9.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__indoc__1_0_8",
        url = "https://crates.io/api/v1/crates/indoc/1.0.8/download",
        type = "tar.gz",
        sha256 = "da2d6f23ffea9d7e76c53eee25dfb67bcd8fde7f1198b0855350698c9f07c780",
        strip_prefix = "indoc-1.0.8",
        build_file = Label("//third_party/cargo/remote:BUILD.indoc-1.0.8.bazel"),
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
        name = "raze__itertools__0_10_5",
        url = "https://crates.io/api/v1/crates/itertools/0.10.5/download",
        type = "tar.gz",
        sha256 = "b0fd2260e829bddf4cb6ea802289de2f86d6a7a690192fbe91b3f46e0f2c8473",
        strip_prefix = "itertools-0.10.5",
        build_file = Label("//third_party/cargo/remote:BUILD.itertools-0.10.5.bazel"),
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
        name = "raze__itoa__1_0_5",
        url = "https://crates.io/api/v1/crates/itoa/1.0.5/download",
        type = "tar.gz",
        sha256 = "fad582f4b9e86b6caa621cabeb0963332d92eea04729ab12892c2533951e6440",
        strip_prefix = "itoa-1.0.5",
        build_file = Label("//third_party/cargo/remote:BUILD.itoa-1.0.5.bazel"),
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
        name = "raze__libc__0_2_138",
        url = "https://crates.io/api/v1/crates/libc/0.2.138/download",
        type = "tar.gz",
        sha256 = "db6d7e329c562c5dfab7a46a2afabc8b987ab9a4834c9d1ca04dc54c1546cef8",
        strip_prefix = "libc-0.2.138",
        build_file = Label("//third_party/cargo/remote:BUILD.libc-0.2.138.bazel"),
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
        name = "raze__libloading__0_7_4",
        url = "https://crates.io/api/v1/crates/libloading/0.7.4/download",
        type = "tar.gz",
        sha256 = "b67380fd3b2fbe7527a606e18729d21c6f3951633d0500574c4dc22d2d638b9f",
        strip_prefix = "libloading-0.7.4",
        build_file = Label("//third_party/cargo/remote:BUILD.libloading-0.7.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__link_cplusplus__1_0_8",
        url = "https://crates.io/api/v1/crates/link-cplusplus/1.0.8/download",
        type = "tar.gz",
        sha256 = "ecd207c9c713c34f95a097a5b029ac2ce6010530c7b49d7fea24d977dede04f5",
        strip_prefix = "link-cplusplus-1.0.8",
        build_file = Label("//third_party/cargo/remote:BUILD.link-cplusplus-1.0.8.bazel"),
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
        name = "raze__miniz_oxide__0_6_2",
        url = "https://crates.io/api/v1/crates/miniz_oxide/0.6.2/download",
        type = "tar.gz",
        sha256 = "b275950c28b37e794e8c55d88aeb5e139d0ce23fdbbeda68f8d7174abdf9e8fa",
        strip_prefix = "miniz_oxide-0.6.2",
        build_file = Label("//third_party/cargo/remote:BUILD.miniz_oxide-0.6.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__moveit__0_5_1",
        url = "https://crates.io/api/v1/crates/moveit/0.5.1/download",
        type = "tar.gz",
        sha256 = "d7d756ffe4e38013507d35bf726a93fcdae2cae043ab5ce477f13857a335030d",
        strip_prefix = "moveit-0.5.1",
        build_file = Label("//third_party/cargo/remote:BUILD.moveit-0.5.1.bazel"),
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
        name = "raze__object__0_30_0",
        url = "https://crates.io/api/v1/crates/object/0.30.0/download",
        type = "tar.gz",
        sha256 = "239da7f290cfa979f43f85a8efeee9a8a76d0827c356d37f9d3d7254d6b537fb",
        strip_prefix = "object-0.30.0",
        build_file = Label("//third_party/cargo/remote:BUILD.object-0.30.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__once_cell__1_16_0",
        url = "https://crates.io/api/v1/crates/once_cell/1.16.0/download",
        type = "tar.gz",
        sha256 = "86f0b0d4bf799edbc74508c1e8bf170ff5f41238e5f8225603ca7caaae2b7860",
        strip_prefix = "once_cell-1.16.0",
        build_file = Label("//third_party/cargo/remote:BUILD.once_cell-1.16.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__os_str_bytes__6_4_1",
        url = "https://crates.io/api/v1/crates/os_str_bytes/6.4.1/download",
        type = "tar.gz",
        sha256 = "9b7820b9daea5457c9f21c69448905d723fbd21136ccf521748f23fd49e723ee",
        strip_prefix = "os_str_bytes-6.4.1",
        build_file = Label("//third_party/cargo/remote:BUILD.os_str_bytes-6.4.1.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__owo_colors__3_5_0",
        url = "https://crates.io/api/v1/crates/owo-colors/3.5.0/download",
        type = "tar.gz",
        sha256 = "c1b04fb49957986fdce4d6ee7a65027d55d4b6d2265e5848bbb507b58ccfdb6f",
        strip_prefix = "owo-colors-3.5.0",
        build_file = Label("//third_party/cargo/remote:BUILD.owo-colors-3.5.0.bazel"),
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
        name = "raze__predicates_core__1_0_5",
        url = "https://crates.io/api/v1/crates/predicates-core/1.0.5/download",
        type = "tar.gz",
        sha256 = "72f883590242d3c6fc5bf50299011695fa6590c2c70eac95ee1bdb9a733ad1a2",
        strip_prefix = "predicates-core-1.0.5",
        build_file = Label("//third_party/cargo/remote:BUILD.predicates-core-1.0.5.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__predicates_tree__1_0_7",
        url = "https://crates.io/api/v1/crates/predicates-tree/1.0.7/download",
        type = "tar.gz",
        sha256 = "54ff541861505aabf6ea722d2131ee980b8276e10a1297b94e896dd8b621850d",
        strip_prefix = "predicates-tree-1.0.7",
        build_file = Label("//third_party/cargo/remote:BUILD.predicates-tree-1.0.7.bazel"),
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
        name = "raze__proc_macro2__1_0_49",
        url = "https://crates.io/api/v1/crates/proc-macro2/1.0.49/download",
        type = "tar.gz",
        sha256 = "57a8eca9f9c4ffde41714334dee777596264c7825420f521abc92b5b5deb63a5",
        strip_prefix = "proc-macro2-1.0.49",
        build_file = Label("//third_party/cargo/remote:BUILD.proc-macro2-1.0.49.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__quote__1_0_23",
        url = "https://crates.io/api/v1/crates/quote/1.0.23/download",
        type = "tar.gz",
        sha256 = "8856d8364d252a14d474036ea1358d63c9e6965c8e5c1885c18f73d70bff9c7b",
        strip_prefix = "quote-1.0.23",
        build_file = Label("//third_party/cargo/remote:BUILD.quote-1.0.23.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__redox_syscall__0_2_16",
        url = "https://crates.io/api/v1/crates/redox_syscall/0.2.16/download",
        type = "tar.gz",
        sha256 = "fb5a58c1855b4b6819d59012155603f0b22ad30cad752600aadfcb695265519a",
        strip_prefix = "redox_syscall-0.2.16",
        build_file = Label("//third_party/cargo/remote:BUILD.redox_syscall-0.2.16.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__regex__1_7_0",
        url = "https://crates.io/api/v1/crates/regex/1.7.0/download",
        type = "tar.gz",
        sha256 = "e076559ef8e241f2ae3479e36f97bd5741c0330689e217ad51ce2c76808b868a",
        strip_prefix = "regex-1.7.0",
        build_file = Label("//third_party/cargo/remote:BUILD.regex-1.7.0.bazel"),
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
        name = "raze__regex_syntax__0_6_28",
        url = "https://crates.io/api/v1/crates/regex-syntax/0.6.28/download",
        type = "tar.gz",
        sha256 = "456c603be3e8d448b072f410900c09faf164fbce2d480456f50eea6e25f9c848",
        strip_prefix = "regex-syntax-0.6.28",
        build_file = Label("//third_party/cargo/remote:BUILD.regex-syntax-0.6.28.bazel"),
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
        sha256 = "bfa0f585226d2e68097d4f95d113b15b83a82e819ab25717ec0590d9584ef366",
        strip_prefix = "rustc_version-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.rustc_version-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__rustversion__1_0_11",
        url = "https://crates.io/api/v1/crates/rustversion/1.0.11/download",
        type = "tar.gz",
        sha256 = "5583e89e108996506031660fe09baa5011b9dd0341b89029313006d1fb508d70",
        strip_prefix = "rustversion-1.0.11",
        build_file = Label("//third_party/cargo/remote:BUILD.rustversion-1.0.11.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__ryu__1_0_12",
        url = "https://crates.io/api/v1/crates/ryu/1.0.12/download",
        type = "tar.gz",
        sha256 = "7b4b9743ed687d4b4bcedf9ff5eaa7398495ae14e61cba0a295704edbc7decde",
        strip_prefix = "ryu-1.0.12",
        build_file = Label("//third_party/cargo/remote:BUILD.ryu-1.0.12.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__semver__1_0_16",
        url = "https://crates.io/api/v1/crates/semver/1.0.16/download",
        type = "tar.gz",
        sha256 = "58bc9567378fc7690d6b2addae4e60ac2eeea07becb2c64b9f218b53865cba2a",
        strip_prefix = "semver-1.0.16",
        build_file = Label("//third_party/cargo/remote:BUILD.semver-1.0.16.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde__1_0_151",
        url = "https://crates.io/api/v1/crates/serde/1.0.151/download",
        type = "tar.gz",
        sha256 = "97fed41fc1a24994d044e6db6935e69511a1153b52c15eb42493b26fa87feba0",
        strip_prefix = "serde-1.0.151",
        build_file = Label("//third_party/cargo/remote:BUILD.serde-1.0.151.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde_derive__1_0_151",
        url = "https://crates.io/api/v1/crates/serde_derive/1.0.151/download",
        type = "tar.gz",
        sha256 = "255abe9a125a985c05190d687b320c12f9b1f0b99445e608c21ba0782c719ad8",
        strip_prefix = "serde_derive-1.0.151",
        build_file = Label("//third_party/cargo/remote:BUILD.serde_derive-1.0.151.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__serde_json__1_0_91",
        url = "https://crates.io/api/v1/crates/serde_json/1.0.91/download",
        type = "tar.gz",
        sha256 = "877c235533714907a8c2464236f5c4b2a17262ef1bd71f38f35ea592c8da6883",
        strip_prefix = "serde_json-1.0.91",
        build_file = Label("//third_party/cargo/remote:BUILD.serde_json-1.0.91.bazel"),
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
        name = "raze__slab__0_4_7",
        url = "https://crates.io/api/v1/crates/slab/0.4.7/download",
        type = "tar.gz",
        sha256 = "4614a76b2a8be0058caa9dbbaf66d988527d86d003c11a94fbd335d7661edcef",
        strip_prefix = "slab-0.4.7",
        build_file = Label("//third_party/cargo/remote:BUILD.slab-0.4.7.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__smallvec__1_10_0",
        url = "https://crates.io/api/v1/crates/smallvec/1.10.0/download",
        type = "tar.gz",
        sha256 = "a507befe795404456341dfab10cef66ead4c041f62b8b11bbb92bffe5d0953e0",
        strip_prefix = "smallvec-1.10.0",
        build_file = Label("//third_party/cargo/remote:BUILD.smallvec-1.10.0.bazel"),
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
        name = "raze__strum_macros__0_24_3",
        url = "https://crates.io/api/v1/crates/strum_macros/0.24.3/download",
        type = "tar.gz",
        sha256 = "1e385be0d24f186b4ce2f9982191e7101bb737312ad61c1f2f984f34bcf85d59",
        strip_prefix = "strum_macros-0.24.3",
        build_file = Label("//third_party/cargo/remote:BUILD.strum_macros-0.24.3.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__supports_color__1_3_1",
        url = "https://crates.io/api/v1/crates/supports-color/1.3.1/download",
        type = "tar.gz",
        sha256 = "8ba6faf2ca7ee42fdd458f4347ae0a9bd6bcc445ad7cb57ad82b383f18870d6f",
        strip_prefix = "supports-color-1.3.1",
        build_file = Label("//third_party/cargo/remote:BUILD.supports-color-1.3.1.bazel"),
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
        name = "raze__syn__1_0_107",
        url = "https://crates.io/api/v1/crates/syn/1.0.107/download",
        type = "tar.gz",
        sha256 = "1f4064b5b16e03ae50984a5a8ed5d4f8803e6bc1fd170a3cda91a1be4b18e3f5",
        strip_prefix = "syn-1.0.107",
        build_file = Label("//third_party/cargo/remote:BUILD.syn-1.0.107.bazel"),
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
        name = "raze__termtree__0_4_0",
        url = "https://crates.io/api/v1/crates/termtree/0.4.0/download",
        type = "tar.gz",
        sha256 = "95059e91184749cb66be6dc994f67f182b6d897cb3df74a5bf66b5e709295fd8",
        strip_prefix = "termtree-0.4.0",
        build_file = Label("//third_party/cargo/remote:BUILD.termtree-0.4.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__test_log__0_2_11",
        url = "https://crates.io/api/v1/crates/test-log/0.2.11/download",
        type = "tar.gz",
        sha256 = "38f0c854faeb68a048f0f2dc410c5ddae3bf83854ef0e4977d58306a5edef50e",
        strip_prefix = "test-log-0.2.11",
        build_file = Label("//third_party/cargo/remote:BUILD.test-log-0.2.11.bazel"),
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
        name = "raze__textwrap__0_15_2",
        url = "https://crates.io/api/v1/crates/textwrap/0.15.2/download",
        type = "tar.gz",
        sha256 = "b7b3e525a49ec206798b40326a44121291b530c963cfb01018f63e135bac543d",
        strip_prefix = "textwrap-0.15.2",
        build_file = Label("//third_party/cargo/remote:BUILD.textwrap-0.15.2.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__textwrap__0_16_0",
        url = "https://crates.io/api/v1/crates/textwrap/0.16.0/download",
        type = "tar.gz",
        sha256 = "222a222a5bfe1bba4a77b45ec488a741b3cb8872e5e499451fd7d0129c9c7c3d",
        strip_prefix = "textwrap-0.16.0",
        build_file = Label("//third_party/cargo/remote:BUILD.textwrap-0.16.0.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__thiserror__1_0_38",
        url = "https://crates.io/api/v1/crates/thiserror/1.0.38/download",
        type = "tar.gz",
        sha256 = "6a9cd18aa97d5c45c6603caea1da6628790b37f7a34b6ca89522331c5180fed0",
        strip_prefix = "thiserror-1.0.38",
        build_file = Label("//third_party/cargo/remote:BUILD.thiserror-1.0.38.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__thiserror_impl__1_0_38",
        url = "https://crates.io/api/v1/crates/thiserror-impl/1.0.38/download",
        type = "tar.gz",
        sha256 = "1fb327af4685e4d03fa8cbcf1716380da910eeb2bb8be417e7f9fd3fb164f36f",
        strip_prefix = "thiserror-impl-1.0.38",
        build_file = Label("//third_party/cargo/remote:BUILD.thiserror-impl-1.0.38.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__toml__0_5_10",
        url = "https://crates.io/api/v1/crates/toml/0.5.10/download",
        type = "tar.gz",
        sha256 = "1333c76748e868a4d9d1017b5ab53171dfd095f70c712fdb4653a406547f598f",
        strip_prefix = "toml-0.5.10",
        build_file = Label("//third_party/cargo/remote:BUILD.toml-0.5.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__trybuild__1_0_73",
        url = "https://crates.io/api/v1/crates/trybuild/1.0.73/download",
        type = "tar.gz",
        sha256 = "ed01de3de062db82c0920b5cabe804f88d599a3f217932292597c678c903754d",
        strip_prefix = "trybuild-1.0.73",
        build_file = Label("//third_party/cargo/remote:BUILD.trybuild-1.0.73.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_ident__1_0_6",
        url = "https://crates.io/api/v1/crates/unicode-ident/1.0.6/download",
        type = "tar.gz",
        sha256 = "84a22b9f218b40614adcb3f4ff08b703773ad44fa9423e4e0d346d5db86e4ebc",
        strip_prefix = "unicode-ident-1.0.6",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-ident-1.0.6.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_linebreak__0_1_4",
        url = "https://crates.io/api/v1/crates/unicode-linebreak/0.1.4/download",
        type = "tar.gz",
        sha256 = "c5faade31a542b8b35855fff6e8def199853b2da8da256da52f52f1316ee3137",
        strip_prefix = "unicode-linebreak-0.1.4",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-linebreak-0.1.4.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__unicode_width__0_1_10",
        url = "https://crates.io/api/v1/crates/unicode-width/0.1.10/download",
        type = "tar.gz",
        sha256 = "c0edd1e5b14653f783770bce4a4dabb4a5108a5370a5f5d8cfe8710c361f6c8b",
        strip_prefix = "unicode-width-0.1.10",
        build_file = Label("//third_party/cargo/remote:BUILD.unicode-width-0.1.10.bazel"),
    )

    maybe(
        http_archive,
        name = "raze__uuid__1_2_2",
        url = "https://crates.io/api/v1/crates/uuid/1.2.2/download",
        type = "tar.gz",
        sha256 = "422ee0de9031b5b948b97a8fc04e3aa35230001a722ddd27943e0be31564ce4c",
        strip_prefix = "uuid-1.2.2",
        build_file = Label("//third_party/cargo/remote:BUILD.uuid-1.2.2.bazel"),
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
        name = "raze__wasi__0_11_0_wasi_snapshot_preview1",
        url = "https://crates.io/api/v1/crates/wasi/0.11.0+wasi-snapshot-preview1/download",
        type = "tar.gz",
        sha256 = "9c8d87e72b64a3b4db28d11ce29237c246188f4f51057d65a7eab63b7987e423",
        strip_prefix = "wasi-0.11.0+wasi-snapshot-preview1",
        build_file = Label("//third_party/cargo/remote:BUILD.wasi-0.11.0+wasi-snapshot-preview1.bazel"),
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
        name = "raze__which__4_3_0",
        url = "https://crates.io/api/v1/crates/which/4.3.0/download",
        type = "tar.gz",
        sha256 = "1c831fbbee9e129a8cf93e7747a82da9d95ba8e16621cae60ec2cdc849bacb7b",
        strip_prefix = "which-4.3.0",
        build_file = Label("//third_party/cargo/remote:BUILD.which-4.3.0.bazel"),
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
