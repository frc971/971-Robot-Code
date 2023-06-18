def x_lex_order(pta, ptb)
	return pta.x <=> ptb.x if pta.x != ptb.x
	return pta.y <=> ptb.y
end

# test that line1 intersects line2
def test_inter(ln1, ln2)
	st = ln1.st
	tst = ln1.ed - st
	as = tst.cross(ln2.st - st) <=> 0
	bs = tst.cross(ln2.ed - st) <=> 0
	
	raise "colinear" if as == 0 && bs == 0
	return true if as == 0 || bs == 0
	return true if as != bs
	return false
end

# test that line1 intersects line2
def test_inter_or_parrallel(ln1, ln2)
	st = ln1.st
	tst = ln1.ed - st
	as = tst.cross(ln2.st - st) <=> 0
	bs = tst.cross(ln2.ed - st) <=> 0
	
	return true if as == 0 && bs == 0
	return true if as == 0 || bs == 0
	return true if as != bs
	return false
end

def get_sys_eqn(ln)
	a = ln.ed.y - ln.st.y
	b = ln.st.x - ln.ed.x
	c = ln.st.x * a + ln.st.y * b
	return a, b, c
end

def intersect(ln1, ln2)
	# puts "intersecting: #{ln1} #{ln2}"
	return nil if !test_inter(ln1, ln2)
	return nil if !test_inter(ln2, ln1)

	a1, b1, c1 = get_sys_eqn(ln1)
	a2, b2, c2 = get_sys_eqn(ln2)
	d = a1 * b2 - b1 * a2
	raise "problem!" if d == 0
	xn = c1 * b2 - c2 * b1
	yn = c2 * a1 - c1 * a2
	return Point2di.new(Rational(xn, d), Rational(yn, d))
end
