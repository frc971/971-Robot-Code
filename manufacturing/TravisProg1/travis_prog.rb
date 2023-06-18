$LOAD_PATH.unshift(File.dirname(__FILE__) + "/lib_files")
require "geometry/router/nester.rb"
Router::do_nesting()
