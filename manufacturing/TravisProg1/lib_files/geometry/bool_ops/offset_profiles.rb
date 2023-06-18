class XMonotonicSubdivisionArc
	attr_accessor :st, :ed
	def initialize(st, ed, d)
		@st = st
		@ed = ed
		@d = d

		@d_denom = 1

		#d = d.abs
		#a = (ed - st).rot90()
		#mag = a.mag_sqr() 

		#df = d.to_f
		#@r_real = ((df) ** 2 + mag / 4) / (2 * df)

#		@guard

	end
	def draw_subseq(cr, scale, st, ed, d, d_denom, d_limit, l)
		sign = d <=> 0
		d = d.abs
		a = (ed - st).rot90()
		mag = a.mag_sqr() 

		magv = mag * d_denom * d_denom
		df = d.to_f / d_denom
		restimate = ((df) ** 2 + mag / 4) / (2 * df)
		if (d < d_denom * d_limit)# * 4)
			(@k &1 == 0 ? Color::Red : Color::Green).set(cr)
			@k += 1
			cr.move_to(st.x * scale, -st.y * scale)
			cr.line_to(ed.x * scale, -ed.y * scale)
			cr.stroke()
		else
			vec = (a * (sign * d * 2)) / integer_sqrt(magv)
			mp = (st + ed + vec) / 2 + a.round_up_fudge

			puts "restimate: #{l} -> #{restimate}"

#			l1 = (st - mp).mag_sqr()
#			l2 = (ed - mp).mag_sqr()

			#psqr = ((restimate - df / 2)) ** 2 + mag / (4**2)
			psqr = ((((df) ** 2 + mag / 4) / (2 * df) - df / 2)) ** 2 + mag / (4**2)
			dnxt = restimate - Math.sqrt(psqr) #integer_sqrt(psqr)
			d_denom = d_denom * 16 #* 16

			#puts "d: #{df / dnxt}"
			#puts "d: #{df} -> #{dnxt}"
			dnxt = sign * (dnxt * d_denom).floor.to_i

			draw_subseq(cr, scale, st, mp, dnxt, d_denom, d_limit, l + 1)
			draw_subseq(cr, scale, mp, ed, dnxt, d_denom, d_limit, l + 1)


			#arbn = ((4 * d * d_denom)) 
			#arbd = (mag * d_denom * d_denom - d * d * 4)

			#arbd2 = (restimate * d_denom - d) * 4 * 2 * d * d_denom
			#puts "restimate: #{restimate}  arb: #{arbn} / #{arbd} arbd: #{arbd / arbd2}"
=begin
			s = 2 * Math.sqrt(mag) / ((restimate - df) * 4)


			m = (a * s).to_i_vec + a.round_up_fudge * 2
			#puts m.inspect
			k = (m * sign + st + ed) / 2
			
			Color::Pink.set(cr)
			cr.move_to(st.x * scale, -st.y * scale)
			cr.line_to(k.x * scale, -k.y * scale)
			cr.line_to(ed.x * scale, -ed.y * scale)
			cr.stroke()
=end
		end
	end
	def draw_to(cr, scale)
#		derr = (@st - @ed)
#		p = derr.mag_sqr()

#		num = 

#		c = ((derr.rot90() * (2 * num)) / denom + (@st + @ed)) / 2
		
		@k = 0
		st = @st
#		cr.move_to(@st.x * scale, -@st.y * scale)
		d_denom_limit = [1, (1 / scale)].max
		#d_denom_limit = 1
		draw_subseq(cr, scale, @st, @ed, @d * 1, @d_denom, d_denom_limit, 0)
		cr.stroke()
=begin
		c = to_offsetable().c
		Color::Pink.set(cr)
		cr.line_to(@ed.x * scale, -@ed.y * scale)
		cr.line_to(c .x * scale, -c.y * scale)
		cr.line_to(@st.x * scale, -@st.y * scale)
		cr.stroke()
=end
		#cr.arc(@c.x * scale, -@c.y * scale, @r * scale, @sa, @ea)
	end
	def append_subseq_to(segs, st, ed, d, d_denom, d_limit, l)
		sign = d <=> 0
		d = d.abs
		a = (ed - st).rot90()
		mag = a.mag_sqr() 

		magv = mag * d_denom * d_denom
		df = d.to_f / d_denom
		restimate = ((df) ** 2 + mag / 4) / (2 * df)
		if (d < d_denom * d_limit)# * 4)
			segs << Shape::Segment.new(st, ed)
		else
			vec = (a * (sign * d * 2)) / integer_sqrt(magv)
			mp = (st + ed + vec) / 2 + a.round_up_fudge

			psqr = ((((df) ** 2 + mag / 4) / (2 * df) - df / 2)) ** 2 + mag / (4**2)
			dnxt = restimate - Math.sqrt(psqr) #integer_sqrt(psqr)
			d_denom = d_denom * 16 #* 16

			dnxt = sign * (dnxt * d_denom).floor.to_i

			append_subseq_to(segs, st, mp, dnxt, d_denom, d_limit, l + 1)
			append_subseq_to(segs, mp, ed, dnxt, d_denom, d_limit, l + 1)
		end
	end
	def to_offsetable()
		sign = -(@d <=> 0)
		d = @d.abs
		a = (@st - @ed).rot90()
		mag = a.mag_sqr() 

		magv = mag * @d_denom * @d_denom
		df = d.to_f / @d_denom
		restimate = ((df) ** 2 + mag / 4) / (2 * df)
		vec = (a * (sign * (d - (restimate * @d_denom).to_i) * 2)) / integer_sqrt(magv)

		c = (@st + @ed + vec) / 2
		OffsettableArc.new(@st, @ed, c, restimate.to_i)
	end
	def append_to(segs, scale)
		append_subseq_to(segs, @st, @ed, @d * 1, @d_denom, scale, 0)
	end
	def flip()
		@st, @ed = @ed, @st
		@d = -@d
	end
end

class OffsettableArc
	attr_accessor :c
	def initialize(st, ed, c, r)
		@st, @ed, @c, @r = st, ed, c, r
	end
	def offset_pt(pt, dist)
		v = (pt - @c)
		magsqr = v.mag_sqr()
		l = integer_sqrt(magsqr * 2**20)
		v = (v * dist * 2**10) / l
		return pt + v, pt - v
	end
	def offset(dist)
		sign = (@ed - @c).cross(@st - @c) <=> 0
		#lns = []
		sto, sti = offset_pt(@st, dist)
		edo, edi = offset_pt(@ed, dist)
		#lns << OffsettableArc.new(sto, edo, @c, @r + dist)
		#lns << OffsettableArc.new(sti, edi, @c, @r - dist)
		#lns << OffsettableArc.new(sti, sto, @st, dist)
		#lns << OffsettableArc.new(edo, edi, @ed, dist)
		rlns = []
		#lns.each { |ln| ln.to_xmonotone(rlns, sign) }
		OffsettableArc.new(edo, sto, @c, @r + dist).to_xmonotone(rlns, -sign)
		OffsettableArc.new(sto, sti, @st, dist).to_xmonotone(rlns, -sign)
		OffsettableArc.new(sti, edi, @c, @r - dist).to_xmonotone(rlns, sign)
		OffsettableArc.new(edi, edo, @ed, dist).to_xmonotone(rlns, -sign)
		if (sign < 0)
			rlns.each do |rln|
				rln.flip()
			end
			rlns.reverse!
		end
		return rlns
	end
	def to_xmonotone(lns, sign)
		magsqr = ((@st + @ed) - @c * 2).mag_sqr()
		lns << XMonotonicSubdivisionArc.new(@st, @ed, sign * (@r - integer_sqrt(magsqr) / 2))
	end
end
class TransferLine
	def initialize(st, ed) @st, @ed = st, ed end
	def append_to(segs, scale)
		segs << Shape::Segment.new(@st, @ed)
	end
end
class OffsetableLine
	def initialize(st, ed)
		@st, @ed = st, ed
	end
	def offset_pt(pt, dist)
		v = (@ed - @st).rot90()
		magsqr = v.mag_sqr()
		l = integer_sqrt(magsqr * 2**20)
		v = (v * dist * 2**10) / l
		return pt + v, pt - v
	end
	def offset(dist)
		sto, sti = offset_pt(@st, dist)
		edo, edi = offset_pt(@ed, dist)
		rlns = []
		rlns << TransferLine.new(edo, sto)
		OffsettableArc.new(sto, sti, @st, dist).to_xmonotone(rlns, -1)
		rlns << TransferLine.new(sti, edi)
		OffsettableArc.new(edi, edo, @ed, dist).to_xmonotone(rlns, -1)
	end
end
