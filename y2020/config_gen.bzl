def generate_pi_config(number):
  native.genrule(
      name = "generate_pi_config_%d" % (number),
      srcs = ["y2020_pi_template.json"],
      outs = ["y2020_pi%d.json" % (number)],
      cmd = " ".join([
          "$(location //y2020:generate_pi_config)",
          "$(location y2020_pi_template.json)",
          "'{\"NUM\": %d}'" % (number),
          "$(OUTS)",
      ]),
      tools = ["//y2020:generate_pi_config"],
  )
