require_relative "predicate.rb"

def lex_order_pt(a, b)
	return a, b if x_lex_order(a, b) < 0
	return b, a
end

def intersect_parallel_filter(a, b)
	ast, aed = lex_order_pt(a.st, a.ed)
	bst, bed = lex_order_pt(b.st, b.ed)
	return nil if x_lex_order(aed, bst) < 0
	return nil if x_lex_order(bed, ast) < 0
	return intersect(a, b)
end

def find_intersect_overlap_dup(ln1, ln2, inters, i, j)
	ast, aed = lex_order_pt(ln1.st, ln1.ed)
	bst, bed = lex_order_pt(ln2.st, ln2.ed)
	return if x_lex_order(aed, bst) < 0
	return if x_lex_order(bed, ast) < 0
	return if !test_inter_or_parrallel(ln1, ln2)
	return if !test_inter_or_parrallel(ln2, ln1)

	a1, b1, c1 = get_sys_eqn(ln1)
	a2, b2, c2 = get_sys_eqn(ln2)
	d = a1 * b2 - b1 * a2
	if d == 0
		if (x_lex_order(ast, bst) > 0) #ast, aed
			inters << [bed, i, j]
			if (x_lex_order(ast, bed) != 0)
				inters << [ast, i, j]
			end
		else # aed, bst
			inters << [bst, i, j]
			if (x_lex_order(aed, bst) != 0)
				inters << [aed, i, j]
			end
		end
		return
	end
	xn = c1 * b2 - c2 * b1
	yn = c2 * a1 - c1 * a2
	
	inters << [Point2di.new(Rational(xn, d), Rational(yn, d)), i, j]
end

class AllPairsInter
	def self.inter(shapes)
		all_lns = []
		shapes.each do |shape|
			all_lns += shape.segs
		end
		inters = []
		0.upto(all_lns.length - 1) do |i|
			(i + 1).upto(all_lns.length - 1) do |j|
				find_intersect_overlap_dup(all_lns[i], all_lns[j], inters, i, j)
			end
		end
		inters.sort! { |a, b| x_lex_order(a[0], b[0]) }
		prevs = [] 
		prev_pt = nil
		inters_lst = []
		inters.each do |a, b, c|
			if (!prev_pt)
				prevs = []
				prev_pt = a
			elsif (x_lex_order(prev_pt, a) != 0)
				inters_lst << [prev_pt, prevs.uniq()]
				prevs = []
				prev_pt = a
			end
			prevs << b << c
		end
		inters_lst << [prev_pt, prevs.uniq()] if prev_pt

		line_last_inter = []
		pt_infos = inters_lst.collect do |pt, is|
			pt_i = PointInfo.new(pt)
			#puts is.inspect
			is.each do |i|
				if line_last_inter[i]
					line_last_inter[i].connect_to(pt_i, -1)
				end
				line_last_inter[i] = pt_i
			end
			pt_i
		end
		pt_infos.each do |pt_info|
			pt_info.do_dedup()
		end
		
		labl = LabeledLineList.new()
		lno = ln = pt_infos[0].lex_min_line().twin
		i = 0
		while (labl.add_line(ln) ; ln = ln.next(); ln != lno) 
			i += 1
		end

		return labl.to_shape() 
	end
end
