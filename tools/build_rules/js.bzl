load("@aspect_rules_js//js:providers.bzl", "JsInfo")
load("@bazel_skylib//rules:write_file.bzl", "write_file")
load("@aspect_rules_js//js:defs.bzl", "js_library")
load("@aspect_rules_js//npm:defs.bzl", "npm_package")
load("@aspect_bazel_lib//lib:copy_to_directory.bzl", "copy_to_directory")
load("@aspect_bazel_lib//lib:copy_file.bzl", "copy_file")
load("@aspect_rules_esbuild//esbuild:defs.bzl", "esbuild")

#load("@npm//:history-server/package_json.bzl", history_server_bin = "bin")
load("@npm//:html-insert-assets/package_json.bzl", html_insert_assets_bin = "bin")
load("//tools/build_rules/js:ng.bzl", "ng_esbuild", "ng_project")
load("//tools/build_rules/js:ts.bzl", _ts_project = "ts_project")
load("@aspect_rules_rollup//rollup:defs.bzl", upstream_rollup_bundle = "rollup_bundle")
load("@aspect_rules_terser//terser:defs.bzl", "terser_minified")
load("@aspect_rules_cypress//cypress:defs.bzl", "cypress_module_test")

ts_project = _ts_project

# Common dependencies of Angular applications
APPLICATION_DEPS = [
    "//:node_modules/@angular/common",
    "//:node_modules/@angular/core",
    #"//:node_modules/@angular/router",
    "//:node_modules/@angular/platform-browser",
    "//:node_modules/@types/node",
    "//:node_modules/rxjs",
    #"//:node_modules/tslib",
]

APPLICATION_HTML_ASSETS = ["styles.css", "favicon.ico"]

# Common dependencies of Angular packages
PACKAGE_DEPS = [
    "//:node_modules/@angular/common",
    "//:node_modules/@angular/core",
    #"//:node_modules/@angular/router",
    "//:node_modules/@types/node",
    "//:node_modules/rxjs",
    #"//:node_modules/tslib",
]

TEST_DEPS = APPLICATION_DEPS + [
    "//:node_modules/@angular/compiler",
    "//:node_modules/@types/jasmine",
    "//:node_modules/jasmine-core",
    "//:node_modules/@angular/platform-browser-dynamic",
]

NG_DEV_DEFINE = {
    "process.env.NODE_ENV": "'development'",
    "ngJitMode": "false",
}
NG_PROD_DEFINE = {
    "process.env.NODE_ENV": "'production'",
    "ngDevMode": "false",
    "ngJitMode": "false",
}

def ng_application(
        name,
        deps = [],
        test_deps = [],
        extra_srcs = [],
        assets = None,
        html_assets = APPLICATION_HTML_ASSETS,
        visibility = ["//visibility:public"],
        **kwargs):
    """
    Bazel macro for compiling an Angular application. Creates {name}, test, serve targets.

    Projects structure:
      main.ts
      index.html
      polyfills.ts
      styles.css, favicon.ico (defaults, can be overriden)
      app/
        **/*.{ts,css,html}

    Tests:
      app/
        **/*.spec.ts

    Args:
      name: the rule name
      deps: direct dependencies of the application
      test_deps: additional dependencies for tests
      html_assets: assets to insert into the index.html, [styles.css, favicon.ico] by default
      assets: assets to include in the file bundle
      visibility: visibility of the primary targets ({name}, 'test', 'serve')
      **kwargs: extra args passed to main Angular CLI rules
    """
    assets = assets if assets else native.glob(["assets/**/*"])
    html_assets = html_assets if html_assets else []

    test_spec_srcs = native.glob(["app/**/*.spec.ts"])

    srcs = native.glob(
        ["main.ts", "app/**/*", "package.json"],
        exclude = test_spec_srcs,
    ) + extra_srcs

    # Primary app source
    ng_project(
        name = "_app",
        srcs = srcs,
        deps = deps + APPLICATION_DEPS,
        visibility = ["//visibility:private"],
    )

    # App polyfills source + bundle.
    ng_project(
        name = "_polyfills",
        srcs = ["polyfills.ts"],
        deps = ["//:node_modules/zone.js"],
        visibility = ["//visibility:private"],
    )
    esbuild(
        name = "polyfills-bundle",
        entry_point = "polyfills.js",
        srcs = [":_polyfills"],
        define = {"process.env.NODE_ENV": "'production'"},
        config = {
            "resolveExtensions": [".mjs", ".js"],
        },
        metafile = False,
        format = "esm",
        minify = True,
        visibility = ["//visibility:private"],
    )

    _pkg_web(
        name = "prod",
        entry_point = "main.js",
        entry_deps = [":_app"],
        html_assets = html_assets,
        assets = assets,
        production = True,
        visibility = ["//visibility:private"],
    )

    _pkg_web(
        name = "dev",
        entry_point = "main.js",
        entry_deps = [":_app"],
        html_assets = html_assets,
        assets = assets,
        production = False,
        visibility = ["//visibility:private"],
    )

    # The default target: the prod package
    native.alias(
        name = name,
        actual = "prod",
        visibility = visibility,
    )

def _pkg_web(name, entry_point, entry_deps, html_assets, assets, production, visibility):
    """ Bundle and create runnable web package.

      For a given application entry_point, assets and defined constants... generate
      a bundle using that entry and constants, an index.html referencing the bundle and
      providated assets, package all content into a resulting directory of the given name.
    """

    bundle = "bundle-%s" % name

    ng_esbuild(
        name = bundle,
        entry_points = [entry_point],
        srcs = entry_deps,
        define = NG_PROD_DEFINE if production else NG_DEV_DEFINE,
        format = "esm",
        output_dir = True,
        splitting = True,
        metafile = False,
        minify = production,
        visibility = ["//visibility:private"],
    )

    html_out = "_%s_html" % name

    html_insert_assets_bin.html_insert_assets(
        name = html_out,
        outs = ["%s/index.html" % html_out],
        args = [
                   # Template HTML file.
                   "--html",
                   "$(location :index.html)",
                   # Output HTML file.
                   "--out",
                   "%s/%s/index.html" % (native.package_name(), html_out),
                   # Root directory prefixes to strip from asset paths.
                   "--roots",
                   native.package_name(),
                   "%s/%s" % (native.package_name(), html_out),
               ] +
               # Generic Assets
               ["--assets"] + ["$(execpath %s)" % s for s in html_assets] +
               ["--scripts", "--module", "polyfills-bundle.js"] +
               # Main bundle to bootstrap the app last
               ["--scripts", "--module", "%s/main.js" % bundle],
        # The input HTML template, all assets for potential access for stamping
        srcs = [":index.html", ":%s" % bundle, ":polyfills-bundle"] + html_assets,
        visibility = ["//visibility:private"],
    )

    copy_to_directory(
        name = name,
        srcs = [":%s" % bundle, ":polyfills-bundle", ":%s" % html_out] + html_assets + assets,
        root_paths = [".", "%s/%s" % (native.package_name(), html_out)],
        visibility = visibility,
    )

    # http server serving the bundle
    # TODO(phil): Get this working.
    #history_server_bin.history_server_binary(
    #    name = "serve" + ("-prod" if production else ""),
    #    args = ["$(location :%s)" % name],
    #    data = [":%s" % name],
    #    visibility = visibility,
    #)

def ng_pkg(name, generate_public_api = True, extra_srcs = [], deps = [], test_deps = [], visibility = ["//visibility:public"], **kwargs):
    """
    Bazel macro for compiling an npm-like Angular package project. Creates '{name}' and 'test' targets.

    Projects structure:
      src/
        public-api.ts
        **/*.{ts,css,html}

    Tests:
      src/
        **/*.spec.ts

    Args:
      name: the rule name
      deps: package dependencies
      test_deps: additional dependencies for tests
      visibility: visibility of the primary targets ('{name}', 'test')
    """

    test_spec_srcs = native.glob(["**/*.spec.ts"])

    srcs = native.glob(
        ["**/*.ts", "**/*.css", "**/*.html"],
        exclude = test_spec_srcs + [
            "public-api.ts",
        ],
    ) + extra_srcs

    # An index file to allow direct imports of the directory similar to a package.json "main"
    write_file(
        name = "_index",
        out = "index.ts",
        content = ["export * from \"./public-api\";"],
        visibility = ["//visibility:private"],
    )

    if generate_public_api:
        write_file(
            name = "_public_api",
            out = "public-api.ts",
            content = [
                "export * from './%s.component';" % name,
                "export * from './%s.module';" % name,
            ],
            visibility = ["//visibility:private"],
        )
        srcs.append(":_public_api")

    ng_project(
        name = "_lib",
        srcs = srcs + [":_index"],
        deps = deps + PACKAGE_DEPS,
        #visibility = ["//visibility:private"],
        visibility = ["//visibility:public"],
        **kwargs
    )

    js_library(
        name = name + "_js",
        srcs = [":_lib"],
        visibility = ["//visibility:public"],
    )

    npm_package(
        name = name,
        srcs = ["package.json", ":_lib"],
        # This is a perf improvement; the default will be flipped to False in rules_js 2.0
        include_runfiles = False,
        visibility = ["//visibility:public"],
    )

def rollup_bundle(name, entry_point, deps = [], visibility = None, **kwargs):
    """Calls the upstream rollup_bundle() and exposes a .min.js file.

    Legacy version of rollup_bundle() used to provide the .min.js file. This
    wrapper provides the same interface by explicitly exposing a .min.js file.
    """
    copy_file(
        name = name + "__rollup_config",
        src = "//:rollup.config.js",
        out = name + "__rollup_config.js",
    )

    upstream_rollup_bundle(
        name = name,
        visibility = visibility,
        deps = deps + [
            "//:node_modules/@rollup/plugin-node-resolve",
        ],
        sourcemap = "false",
        config_file = ":%s__rollup_config.js" % name,
        entry_point = entry_point,
        **kwargs
    )

    terser_minified(
        name = name + "__min",
        srcs = [name + ".js"],
        sourcemap = False,
    )

    # Copy the __min.js file (a declared output inside the rule) so that it's a
    # pre-declared output and publicly visible. I.e. via attr.output() below.
    _expose_file_with_suffix(
        name = name + "__min_exposed",
        src = ":%s__min" % name,
        out = name + ".min.js",
        suffix = "__min.js",
        visibility = visibility,
    )

def _expose_file_with_suffix_impl(ctx):
    """Copies the .min.js file in order to make it publicly accessible."""
    sources = ctx.attr.src[JsInfo].sources.to_list()
    min_js = None
    for src in sources:
        if src.basename.endswith(ctx.attr.suffix):
            min_js = src
            break

    if min_js == None:
        fail("Couldn't find .min.js in " + str(ctx.attr.src))

    ctx.actions.run(
        inputs = [min_js],
        outputs = [ctx.outputs.out],
        executable = "cp",
        arguments = [min_js.path, ctx.outputs.out.path],
    )

_expose_file_with_suffix = rule(
    implementation = _expose_file_with_suffix_impl,
    attrs = {
        "src": attr.label(providers = [JsInfo]),
        "out": attr.output(mandatory = True),
        "suffix": attr.string(mandatory = True),
    },
)

def cypress_test(runner, data = None, **kwargs):
    """Runs a cypress test with the specified runner.

    Args:
        runner: The runner that starts up any necessary servers and then
            invokes Cypress itself. See the Module API documentation for more
            information: https://docs.cypress.io/guides/guides/module-api
        data: The spec files (*.cy.js) and the servers under test. Also any
            other files needed at runtime.
        kwargs: Arguments forwarded to the upstream cypress_module_test().
    """

    # Figure out how many directories deep this package is relative to the
    # workspace root.
    package_depth = len(native.package_name().split("/"))

    # Chrome is located at the runfiles root. So we need to go up one more
    # directory than the workspace root.
    chrome_location = "../" * (package_depth + 1) + "chrome_linux/chrome"
    config_location = "../" * package_depth + "tools/build_rules/js/cypress.config.js"

    data = data or []
    data.append("//tools/build_rules/js:cypress.config.js")
    data.append("@xvfb_amd64//:wrapped_bin/Xvfb")

    cypress_module_test(
        args = [
            "run",
            "--config-file=" + config_location,
            "--browser=" + chrome_location,
        ],
        browsers = ["@chrome_linux//:all"],
        copy_data_to_bin = False,
        cypress = "//:node_modules/cypress",
        data = data,
        runner = runner,
        **kwargs
    )
