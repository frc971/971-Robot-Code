load("//tools/build_rules:pandoc.bzl", "pandoc_html")

articles = [
    "create-a-new-autonomous",
    "create-a-new-robot",
    "create-a-simple-program-for-running-a-motor",
    "download-code-to-the-robot",
    "make-a-drivebase-move",
    "submitting-code-for-a-review",
    "tune-an-autonomous",
    "using-bazel",
]

[pandoc_html(
    name = article_name,
    src = article_name + ".md",
    target_compatible_with = ["@platforms//os:linux"],
) for article_name in articles]
