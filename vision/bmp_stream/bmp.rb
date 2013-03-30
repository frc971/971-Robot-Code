class BMPHeader
	def initialize(width,height)
		@width = width
		@height = height
	end
	def text()
		size = @width * @height * 3
		return ["BM",size + 54,"\0\0\0\0",54,40,@width,
			@height,1,24,0,size,2835,2835,0,0].pack("a2Ia4IIIIssIIIIII")
	end
end
