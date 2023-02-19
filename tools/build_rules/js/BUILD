load("@npm//:@angular/compiler-cli/package_json.bzl", angular_compiler_cli = "bin")
load(":ts.bzl", "ts_project")

# Define the @angular/compiler-cli ngc bin binary as a target
angular_compiler_cli.ngc_binary(
    name = "ngc",
    visibility = ["//visibility:public"],
)

# ESBuild plugin to run the Angular linker
ts_project(
    name = "ngc.esbuild",
    srcs = ["ngc.esbuild.ts"],
    tsconfig = "//:tsconfig.node",
    visibility = ["//visibility:public"],
    deps = [
        "//:node_modules/@angular/compiler-cli",
        "//:node_modules/@babel/core",
        #"//:node_modules/@types/babel__core",
        "//:node_modules/@types/node",
    ],
)
