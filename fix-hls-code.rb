#!/usr/bin/env ruby

SRCDIR = './_gensrc/'

Dir.glob(SRCDIR+'*.v') do |filename|
  fixed = File.read(filename)
    .gsub(/\/\/ power-on initialization(.*?)^end$/m, "")
  File.write(filename, fixed)
end
