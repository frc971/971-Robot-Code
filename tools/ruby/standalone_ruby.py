#!/usr/bin/python

import zlib, sys, os, stat

with file(sys.argv[2], "wb+") as outf:
  outf.write("""#!/usr/bin/env ruby
require "zlib"

def zip_file_binding()
  binding()
end

module Kernel
  class ZipRequireRecord
    attr_accessor :fname, :val
    def initialize(fname, val)
      @fname = fname
      @val = val
    end
    def do_require()
      return false if !@val
      eval(@val, binding = zip_file_binding(), filename="//@bazel/" + @fname)
      @val = nil
      return true
    end
  end
  BAZEL_START_ENTRY = "/@bazel/"
  ZIP_TEMP_OBJS = {}
  def require_relative(relative_feature)
    file = caller.first.split(/:\d/,2).first
    raise LoadError, "require_relative is called in #{$1}" if /\A\((.*)\)/ =~ file
    require File.expand_path(relative_feature, File.dirname(file))
  end
  def require_zip(moduleName)
    if moduleName.start_with?(BAZEL_START_ENTRY)
      moduleName = moduleName[BAZEL_START_ENTRY.length..-1]
    end
    zip_obj = ZIP_TEMP_OBJS[moduleName]
    if not zip_obj and !moduleName.end_with?('.rb')
      zip_obj = ZIP_TEMP_OBJS[moduleName + '.rb']
    end
    return zip_obj.do_require() if (zip_obj)
    return pre_zip_load_require(moduleName)
  end
  alias :pre_zip_load_require :require
  alias :require :require_zip
  def init_app(main_fname)
    $0 = "//@bazel/" + main_fname
    Kernel.require_zip(main_fname)
  end
  def register_inline_file(fname, data)
    raise if ZIP_TEMP_OBJS.has_key?(fname)
    ZIP_TEMP_OBJS[fname] = Kernel::ZipRequireRecord.new(fname, data.force_encoding("UTF-8"))
  end
end

my_data = Zlib.inflate(DATA.read())
""")

  blob_data = []
  offset = 0
  with file(sys.argv[1], 'r') as infile:
    for line in infile.xreadlines():
      file_outname, file_binname = line.strip().split('|')
      with file(file_binname, 'r') as obj_file:
        obj = obj_file.read()
      blob_data.append(obj)
      new_off = offset + len(obj)
      outf.write("Kernel.register_inline_file(%s, my_data.slice(%s...%s))\n" %
                 (repr(file_outname), offset, new_off))
      offset = new_off

  outf.write("Kernel.init_app(%s)\n" % repr(sys.argv[3]))
  outf.write("__END__\n")
  outf.write(zlib.compress("".join(blob_data)))
