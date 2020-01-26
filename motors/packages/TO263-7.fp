	# pad 1,2,3 width (1/100 mil)
	# pad 1,2,3 length (1/100 mil)
	# x value for pads 1,3 (1/100 mil)
	# y value for pads 1,2,3 (1/100 mil)
	# tab pad width (1/100 mil)
	# ideally we would be able to have a polygon
	# pad because the recommendation is a T shape where
	# the width is 328 mil on the thinner part and 425 mil on the
	# thicker part
	# tab pad length (1/100 mil)
	#define(`PADL2',  `27500')
	# x value for the tab pad (1/100 mil)
	# y value for the tab pad (1/100 mil)
	#define(`PADY2', `-21025')
	# package width (1/100 mil)
	# package height (1/100 mil)
	# y values for drawing the pad.  Recall we draw the pad with an aperture
	# we need a line segment of length PADL1 - PADW1 so we have end points:
	# PADY1 +/- 0.5*(PADL1 - PADW1)
	# width of soldermask relief (5 mil on each side)
	# silkscreen width (1/100 mils)
	# clearance to polygons (1/100 mils)
	# how much space to leave around the part before the
	# silk screen (1/100 mils)
	# X values for silk on sides and top
	# X values for silk on sides and bottom
# Element [SFlags "Desc" "Name" "Value" MX MY TX TY TDir TScale TSFlags]
Element[ "" "Transistor" "" "TO263" 0 0 0 0 0 100 ""]
(
# Pad [rX1 rY1 rX2 rY2 Thickness Clearance Mask "Name" "Number" SFlags]
# the signal pads

Pad[ -15000 10725 -15000 5825 3600 2000 4400 "1" "1" "square"]
Pad[ -10000 10725 -10000 5825 3600 2000 4400 "2" "2" "square"]
Pad[ -5000 10725 -5000  5825 3600 2000 4400 "3" "3" "square"]
Pad[ 0 10725 0 5825 3600 2000 4400 "4" "4" "square"]
Pad[ 5000 10725 5000 5825 3600 2000 4400 "5" "5" "square"]
Pad[ 10000 10725 10000 5825 3600 2000 4400 "6" "6" "square"]
Pad[ 15000 10725 15000 5825 3600 2000 4400 "7" "7" "square"]
Pad[ -750 -24525 750 -24525 41000 2000 41800 "8" "8" "square"]

#ElementLine[ x1 y1 x2 y2 width]
ElementLine[ -22750 14025 22750 14025 1000 ]
ElementLine[ -22750 14025 -22750 -46525 1000 ]
ElementLine[ 22750 14025 22750 -46525 1000 ]
ElementLine[ -22750 -46525 22750 -46525 1000 ]


)
