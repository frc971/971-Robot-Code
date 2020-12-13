"""Converts a markdown file to an html file.

Given the src "xyz.md", produces "xyz.html".

Attrs:
  src: Markdown file to convert. Only one file can be specified.
"""

def pandoc_html(name, src, target_compatible_with = None):
    output = name + ".html"
    native.genrule(
        name = name,
        srcs = [src],
        outs = [output],
        cmd = "$(location @pandoc//:pandoc_wrapper) -s $< -o $@",
        tools = ["@pandoc//:all_files", "@pandoc//:pandoc_wrapper"],
        executable = True,
        target_compatible_with = target_compatible_with,
    )
