require "data/codecs/data_view.rb"
class SerializeBuffer
	class Writer
		def initialize()
			@build_stack = [[]]
		end
		def write_varint(varint)
			@build_stack[-1].push(SerializeBuffer::WriteVarint.new(varint))
		end
		def code_string(str)
			@build_stack[-1].push(SerializeBuffer::WriteString.new(str))
		end
		def write_f64(val)
			@build_stack[-1].push(SerializeBuffer::WriteF64.new(val))
		end
		def push()
			@build_stack.push([])
		end
		def pop()
			t = @build_stack.pop()
			@build_stack[-1].push(SerializeBuffer::WriteList.new(t))
		end
		def data() @build_stack[0] end
		def depth() @build_stack.length end
	end
	class WriteVarint
		attr_accessor :varint
		def initialize(varint)
			@varint = varint
		end
		def inspect() @varint.inspect end
		def pretty_print(i)
			puts "#{"  " * i}#{@varint.inspect}"
		end
		def get_concat_len()
			return OutputView.varint_size(@varint)
		end
		def concat_output(output)
			#puts "varint: #{output.i}"
			output.varint(@varint)
		end
	end
	class WriteF64
		attr_accessor :val
		def initialize(val)
			@val = val
		end
		def inspect() @val.inspect end
		def pretty_print(i)
			puts "#{"  " * i}#{@val.inspect}"
		end
		def get_concat_len()
			return 8
		end
		def concat_output(output)
			#puts "varint: #{output.i}"
			output.f64(@val)
		end
	end
	class WriteString
		attr_accessor :str
		def initialize(str)
			@str = str
		end
		def inspect() @str.inspect end
		def pretty_print(i)
			puts "#{"  " * i}#{@str.inspect}"
		end
		def get_concat_len()
			len = @str.bytes.length
			return len + OutputView.varint_size(len)
		end
		def concat_output(output)
			#puts "varstring: #{output.i} #{@str.inspect}"
			output.varint(@str.bytes.length)
			#puts "varstrdata: #{output.i}"
			output.add_bytes(@str.bytes)
		end
	end
	class WriteList
		attr_accessor :items
		def initialize(items)
			@items = items
		end
		def inspect()
			"#{@items.inspect}"
		end
		def pretty_print(i)
			@items.each do |item|
				if (item.class == self.class)
					#puts "#{"  " * i}["
					@items.each do |v|
						v.pretty_print(i + 1)
					end
					#puts "#{"  " * i}]"
					return
				end
			end
			#puts "#{"  " * i}#{@items.inspect}"
		end
		def get_concat_len()
			@len = SerializeBuffer.get_concat_length(@items)
			return @len + OutputView.varint_size(@len)
		end
		def concat_output(output)
			#puts "write_list: #{output.i} #{@len}"
			output.varint(@len)
			#				puts "start_list_list: #{output.i}"
			SerializeBuffer.concat_output(output, @items)
		end
		def get_ipos(ids, idoff)
			@len || get_concat_len()
			return OutputView.varint_size(@len) +
				SerializeBuffer.get_ipos(ids, @items, idoff)
		end
	end
	def self.concat_output(output, arr)
		arr.each do |obj|
			obj.concat_output(output)
		end
	end
	def self.get_concat_length(arr)
		len = 0
		arr.each do |obj|
			len += obj.get_concat_len()
		end
		return len
	end
	def self.get_ipos(ids, arr, idoff = 0)
		ipos = 0
		arr.each.with_index do |obj, i|
			if (i == ids[idoff])
				idoff += 1
				if (idoff < ids.length)
					return ipos + obj.get_ipos(ids, idoff) 
				else
					return ipos 
				end
			else
				ipos += obj.get_concat_len()
			end
		end
		return ipos
	end
	def self.pretty_print(arr)
		arr.each do |v|
			v.pretty_print(0)
		end
	end
	# example:
	# buf = SerializeBuffer.code { |writer| ... }
	# File.write(fname, buf)
	def self.code(&blk)
		ser = SerializeBuffer::Writer.new()
		blk.call(ser)
		data = ser.data()
		len = SerializeBuffer.get_concat_length(data)
		buffer = OutputView.new(len)
		SerializeBuffer.concat_output(buffer, data)
		buffer.val
	end
end
class StreamReader
	def initialize(data)
		@data = DataView.new(data)
		@endis = []
	end
	def ipos()
		@data.i
	end
	def push()
		len = @data.varint()
		@endis.push(@data.endi)
		newendi = @data.i + len
		raise "problem!" if (newendi > @data.endi)
		@data.endi = @data.i + len
	end
	def parse_f64() @data.f64() end
	def parse_varint() @data.varint() end
	def parse_string() @data.get_utf8_string(@data.varint()) end
	def not_done() @data.i < @data.endi end
	def pop()
		raise "problem!" if @data.endi != @data.i
		@data.endi = @endis.pop()
	end
end
