module SweepLine
	def self.order_st_ed(items)
		items.collect { |item|
			if (item.st().comp_h(item.ed()) > 0)
				item.twin()
			else
				item
			end
		}
	end
	def self.stitch_curves(items, tolerance = 0.00001)
		items_out = []
		pts = []
		items.each.with_index { |item, k| PtRef.add_twin_pair(item, k, pts) }

		clusters = point_ref_cluster_and_validate(pts, tolerance)

		visited = []
		clusters.each do |a, b|
			ptref = a
			if (!visited[ptref.key])
				ptref = clock_wise_cluster(a, b)
				sub_profile_a = []
				sub_profile_b = []
				while !visited[ptref.key]
					visited[ptref.key] = true
					sub_profile_a << ptref.obj
					sub_profile_b << ptref.obj.twin
					ptref = ptref.pair.refpair
				end
				sub_profile_a.reverse!
				items_out << TwinableProfile.make_pair(sub_profile_a, sub_profile_b)
				#profiles << LoopProfile2d.new(sub_profile)
			end
		end
		return items_out
	end
end

class TwinableProfile
	def self.make_pair(a, b)
		a, b = self.new(a), self.new(b)
		a.twin = b
		b.twin = a
		return a
	end
	attr_accessor :twin, :items
	def initialize(items)
		@items = items
	end
end

module ProfileOffset
	def self.add_if_is_tangent(a, b)
		tana = a.tangent_at(a.eda)
		tanb = b.tangent_at(b.sta)
		return nil if (tana.area(tanb) > -1e-6 && tana.dot(tanb) > 0.0)
		puts "#{tana.inspect} #{tanb.inspect} = #{tana.area(tanb)}"

		sta = Math.atan2(-tana.x, tana.y)
		eda = Math.atan2(-tanb.x, tanb.y)
		join_arcs = []
		sta += Q4 if (sta < 0)
		eda += Q4 if (eda < 0)
		Arc2d.new(a.ed, 0, sta, eda).to_oriented(join_arcs)
		return join_arcs.collect { |a| SegmentRef.make_pair(a) }
	end
	def self.offset(profile, dist)
		sub_profile_a = []
		sub_profile_b = []
		items = profile.items
		items.each.with_index do |item, i|
			if (join_arcs = add_if_is_tangent(items[i - 1], items[i]))
				join_arcs.each do |join_arc|
					join_arc = join_arc.offset(dist)
					#dist -= 0.025
					sub_profile_a << join_arc
					sub_profile_b << join_arc.twin
				end
			end
			item_off = item.offset(dist)
			dist -= 0.025
			sub_profile_a << item_off
			sub_profile_b << item_off.twin
		end
		sub_profile_b.reverse!
		return TwinableProfile.make_pair(sub_profile_a, sub_profile_b)
	end
	def self.prepare_for_offset(profile)
		sub_profile_a = []
		sub_profile_b = []
		items = profile.items
		items.each.with_index do |item, i|
			if (join_arcs = add_if_is_tangent(items[i - 1], items[i]))
				join_arcs.each do |join_arc|
					sub_profile_a << join_arc
					sub_profile_b << join_arc.twin
				end
			end
			sub_profile_a << item
			sub_profile_b << item.twin
		end
		sub_profile_b.reverse!
		return TwinableProfile.make_pair(sub_profile_a, sub_profile_b)
	end
	def self.just_offset(profile, dist)
		sub_profile_a = []
		sub_profile_b = []
		items = profile.items
		items.each.with_index do |item, i|
			item_off = item.offset(dist)
			sub_profile_a << item_off
			sub_profile_b << item_off.twin
		end
		sub_profile_b.reverse!
		return TwinableProfile.make_pair(sub_profile_a, sub_profile_b)
	end
	def self.neighbor_curves(profile)
		#profile = prepare_for_offset(profile)
		#profile = just_offset(profile, 0.05)
		a, dbg = VoronoiDiagramConstructor.neighbor_curves(profile)
		return dbg
	end
end

class SegmentRef
	class ReversedSegRef
		def offset(dist)
			return @seg.offset(-dist).twin
		end
	end
	def offset(dist)
		return @seg.offset(dist)
	end
end

class QuadrentArc
	def offset(dist)
		q = QuadrentArc.new()
		q.c, q.r, q.sta, q.eda = @c, @r + dist, @sta, @eda

		if q.r < 0
			if (q.sta < Q2)
				q.sta += Q2
				q.eda += Q2
			else
				q.sta -= Q2
				q.eda -= Q2
			end
			q.r = -q.r
			puts "I was flipped"
		end
		return SegmentRef.make_pair(q)
	end
end

class Line2d
	def offset(dist)
		ptoff = (@st - @ed).rot90.normalize().scale(dist)
		return SegmentRef.make_pair(Line2d.new(@st + ptoff, @ed + ptoff))
	end
end

class OffsetGenerator
	def initialize()

	end
end
