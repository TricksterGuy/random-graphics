file = File.open(ARGV[0], 'rb')
output = File.open(ARGV[1], 'w+')

output.write("#<- Begin File #{ARGV[0]} ->\n")

name = ARGV[0]

requires = []
opened = [ARGV[0]]
loop do
  until file.eof?
    line = file.readline
    chunks = line.match(/\Arequire\s("|')(\w+)("|')/)
    if chunks.nil?
      output.write(line)
	output.write("\n")
    elsif chunks[2]
      requires << chunks[2] + '.rb' unless opened.include?(chunks[2]) || requires.include?(chunks[2]) || chunks[2] == 'ruby-processing'
    end
  end
  file.close
  output.write("\n#<- End File #{name} ->\n")
  nex = requires.shift
  break if nex.nil?
  opened << nex
  file = File.open(nex, 'rb')
  name = nex
  output.write("\n#<- Begin File #{name} ->\n")
end
