load("//tools/build_rules:pandoc.bzl", "pandoc_html")

articles = [
    "create-a-new-autonomous",
    "create-a-new-robot",
    "create-a-simple-program-for-running-a-motor",
    "download-code-to-the-robot",
    "make-a-drivebase-move",
    "send-and-receive-messages-on-queues",
    "submitting-code-for-a-review",
    "tune-an-autonomous",
    "generated-queue-code",
    "using-bazel",
]

[pandoc_html(
    name = article_name,
    src = article_name + ".md",
) for article_name in articles]
