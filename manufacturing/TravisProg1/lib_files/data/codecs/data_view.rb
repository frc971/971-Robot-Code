class DataView
	attr_accessor :i, :endi
	def initialize(val)
		@val = val
		@i = 0
		@endi = @val.length
	end
	def unpack_single(i, l, code)
		raise "overran buffer" if (i + l > @val.bytesize)
		v = @val.byteslice(i, l).unpack(code)
		@i = i + l
		return v[0]
	end
	def unpack_multiple(i, l, n, code)
		raise "overran buffer" if (i + l * n > @val.bytesize)
		v = @val.byteslice(i, l * n).unpack("#{code}#{n}")
		@i = i + l * n
		return v
	end
	def f64(i = @i)
		unpack_single(i, 8, "D")
	end
	def nf64(n, i = @i)
		unpack_multiple(i, 8, n, "D")
	end
	def i8(i = @i)
		unpack_single(i, 1, "c")
	end
	def i16(i = @i)
		unpack_single(i, 2, "s")
	end
	def i32(i = @i)
		unpack_single(i, 4, "i")
	end
	def u8(i = @i)
		unpack_single(i, 1, "C")
	end
	def u16(i = @i)
		unpack_single(i, 2, "S")
	end
	def u32(i = @i)
		unpack_single(i, 4, "I")
	end
	def varint(i = @i)
		@i = i
		v = 0
		p = 0
		while true
			k = @val.getbyte(@i)
			v += (k & 0x7f) << p
			p += 7
			@i += 1
			if (k & 0x80 == 0)
				return v
			end
		end
	end
	def get_utf8_string(len, i = @i)
		val = @val.byteslice(i, len)
		@i = i + len
		return val.force_encoding("UTF-8")
	end
	def null_utf16(i = @i)
		i_n = i
		i_n += 2 while @val.byteslice(i_n, 2) != "\0\0"
		val = @val.byteslice(i, i_n - i).force_encoding("UTF-16LE")
		@i = i_n + 2
		val
	end
	def null_ascii8(i = @i)
		i_n = i
		i_n += 1 while @val.byteslice(i_n, 1) != "\0"
		val = @val.byteslice(i, i_n - i)
		@i = i_n + 1
		val
	end
end
BaseNullString = "\0".force_encoding("ASCII-8BIT")
class OutputView
	attr_accessor :i, :val
	def initialize(len)
		@val = BaseNullString * len
		@i = 0
	end
	def pack_single(v, i, l, code)
		raise "overran buffer" if (i + l > @val.bytesize)
		add_bytes([v].pack(code).bytes, i)
		return self
	end
	def f64(v, i = @i)
		pack_single(v, i, 8, "D")
	end
	def i8(v, i = @i)
		pack_single(v, i, 1, "c")
	end
	def i16(v, i = @i)
		pack_single(v, i, 2, "s")
	end
	def i32(v, i = @i)
		pack_single(v, i, 4, "i")
	end
	def u8(v, i = @i)
		pack_single(v, i, 1, "C")
	end
	def u16(v, i = @i)
		pack_single(v, i, 2, "S")
	end
	def u32(v, i = @i)
		pack_single(v, i, 4, "I")
	end
	def add_bytes(v, i = @i)
		v.each do |j|
			@val.setbyte(i, j)
			i += 1
		end
		@i = i
	end
	def varint(v, i = @i)
		@i = i
		v &= 0xFFFFFFFF
		while (v >> 7) != 0
			@val.setbyte(@i, (v & 0x7f) | 0x80)
			@i += 1
			v >>= 7
		end
		@val.setbyte(@i, (v & 0x7f))
		@i += 1
	end
	def self.varint_size(v)
		i = 1
		v &= 0xFFFFFFFF
		i += 1 while (v >>= 7) != 0
		return i
	end
end
