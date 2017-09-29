# This is the footprint recommended in
# http://www.coilcraft.com/pdfs/xal50xx.pdf.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-6500 6950 -6500 -6950 4600 2000 5500 "1" "1" "square"]
	Pad[6500 6950 6500 -6950 4600 2000 5500 "2" "2" "square"]
	ElementLine[-10400 10800 -10400 -10800 800]
	ElementLine[-10400 -10800 10400 -10800 800]
	ElementLine[10400 10800 10400 -10800 800]
	ElementLine[-10400 10800 10400 10800 800]
)
