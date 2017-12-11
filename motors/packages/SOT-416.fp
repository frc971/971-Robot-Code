# This is the footprint recommended in
# http://www.onsemi.com/pub/Collateral/ESDR0502B-D.PDF.
# Aka SC-75.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-0.5mm -0.5715mm -0.5mm -0.7235mm 0.356mm 2000 0.56mm "2" "2" "square"]
	Pad[0.5mm -0.5715mm 0.5mm -0.7235mm 0.356mm 2000 0.56mm "1" "1" "square"]
	Pad[0 0.5715mm 0 0.7235mm 0.356mm 2000 0.56mm "3" "3" "square"]

	ElementLine[0.9mm 0.55mm 0.9mm -0.55mm 800]
	ElementLine[-0.9mm 0.55mm -0.9mm -0.55mm 800]
	ElementLine[0.9mm 0.55mm 0.45mm 0.55mm 800]
	ElementLine[-0.9mm 0.55mm -0.45mm 0.55mm 800]
)
