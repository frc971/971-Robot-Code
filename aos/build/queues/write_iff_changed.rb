require 'tempfile'

# Writes a file like normal except doesn't do anything if the new contents are
# the same as what's already there. Useful for helping build tools not rebuild
# if generated files are re-generated but don't change.
# Usable as a standard File, including to redirect to with IO.popen etc.
class WriteIffChanged < Tempfile
  def initialize(filename)
    super('write_iff_changed')
    @filename = filename
  end
  def WriteIffChanged.open(filename)
    out = WriteIffChanged.new(filename)
    yield out
    out.close
  end
  def close
    flush
    contents = File.open(path, 'r') do |f|
      f.readlines(nil)
    end
    if !File.exists?(@filename) ||
        File.new(@filename, 'r').readlines(nil) != contents
      File.open(@filename, 'w') do |out|
        out.write(contents[0])
      end
    end
    super
  end
end
