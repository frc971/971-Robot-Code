require_relative "entities.rb"

class SegmentRef
	class ReversedSegRef
		attr_accessor :twin, :seg
		def initialize(seg)
			@seg = seg
		end
		def tangent_at(a) -@seg.tangent_at(a) end
		def st() @seg.ed end
		def ed() @seg.st end
		def sta() @seg.eda end 
		def eda() @seg.sta end 
		def is_twin() true end
	end
	attr_accessor :twin, :seg
	def initialize(seg)
		@seg = seg
	end
	def tangent_at(a) @seg.tangent_at(a) end
	def sta() @seg.sta end 
	def eda() @seg.eda end 
	def st() @seg.st end
	def ed() @seg.ed end
	def is_twin() false end
	def self.make_pair(seg)
		ref = SegmentRef.new(seg)
		tref = ReversedSegRef.new(seg)
		ref.twin = tref
		tref.twin = ref
		return ref
	end
end

class Oriented
	def self.import_2d(items2d)
		all_items = []
		items2d.each do |item|
			item.to_oriented(all_items)
		end
		return all_items.collect { |item| SegmentRef.make_pair(item) }
	end
end

class QuadrentArc
	attr_accessor :c, :r, :sta, :eda
	def self.make(c, r, sta, eda)
		q = QuadrentArc.new()
		q.c, q.r, q.sta, q.eda = c, r, sta, eda
		return q
	end
	def pt_for(a) @c + Point2d.new(@r * Math.cos(a), @r * Math.sin(a)) end
	def tangent_at(a)
		return Point2d.new(-Math.sin(a), Math.cos(a))
	end
	def midpoint_and_tangent()
		a = (@sta + @eda) / 2
		return [pt_for(a), tangent_at(a)]
	end
	def st() pt_for(@sta) end
	def ed() pt_for(@eda) end
end

class Line2d
	def midpoint_and_tangent()
		return (@st + @ed).scale(0.5), (@ed - @st).normalize()
	end
	def sta() return 0.0 end
	def eda() return 1.0 end
	def tangent_at(a)
		return (@ed - @st).normalize()
	end
end

Q0 = 0
Q1 = Math::PI / 2
Q2 = 2 * Q1
Q3 = 3 * Q1
Q4 = 4 * Q1

class Arc2d
	def emit_arcs(st, ed, output)
#		puts "emitting arcs: #{st} -> #{ed}"
		return if st == ed

=begin
		4.times do |i|
			iSt = i * Q1
			iEd = (i + 1) * Q1
			if (st < iEd && ed >= iSt)
				ast = st < iSt ? iSt : st
				aed = ed < iEd ? ed : iEd
				if (aed > ast)
					output.push QuadrentArc.make(@c, @r, ast, aed)
				end
			end
		end
=end
		
	end
	def to_oriented(output)
		ed, st = @eda, @sta
		ed += Math::PI * 2 if (ed < st)
		dt = ((ed - st) / Q1).ceil().to_i
		dx = (ed - st) / dt
		ast = st
		dt.times do
			aed = ast + dx
			output.push QuadrentArc.make(@c, @r, ast, aed)
			ast = aed
		end
		return
		if (@sta > @eda)
#			puts "wrapped arcs: #{@sta} -> #{@eda}"
			emit_arcs(Q0, @eda, output)
			emit_arcs(@sta, Q4, output)
		else
			emit_arcs(@sta, @eda, output)
		end
	end
end

class CircleProfile2d
	def to_oriented(output)
		output.push QuadrentArc.make(@c, @r, Q0, Q1)
		output.push QuadrentArc.make(@c, @r, Q1, Q2)
		output.push QuadrentArc.make(@c, @r, Q2, Q3)
		output.push QuadrentArc.make(@c, @r, Q3, Q4)
	end
end

class Line2d
	def to_oriented(output)
		output.push self
	end
end
