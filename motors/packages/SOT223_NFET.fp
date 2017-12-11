Element(0x00 "SMT transistor, 4 pins" "" "SOT223" 305 0 3 100 0x00)
(
	ElementLine(0 0 0 414 10)
	ElementLine(0 414 285 414 10)
	ElementLine(285 414 285 0 10)
	ElementLine(285 0 0 0 10)
	# 1st pin on pin side
	Pad(52 296
	    52 362
			   56
			      "G" "G" 0x100)
	Pad(142 296
	       142 362
			   56
			      "D" "D" 0x100)
	# last pin on pin side
	Pad(233 296
	    233 362
			   56
			      "S" "S" 0x100)
	# extra wide pin on opposite side
	Pad(187 85
	    97 85
			   122 "D" "D" 0x100)
	Mark(52 329)
)
