#!/usr/bin/env ruby

SRCDIR = './_gensrc/'

# remove zero delay (#0) timing control for Verilator (causes SEGV)
Dir.glob(SRCDIR+'*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/\/\/ power-on initialization(.*?)^end$/m, "")
  File.write(filename, fixed)
end
