# This is the footprint recommended in
# http://www.ti.com/lit/ds/symlink/lmr16006.pdf.
# AKA SC-74 and SOT457 and 6-TSOP.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-0.95mm -1.1mm -0.95mm -1.6mm 0.6mm 2000 0.74mm "6" "6" "square"]
	Pad[0mm -1.1mm 0mm -1.6mm 0.6mm 2000 0.74mm "5" "5" "square"]
	Pad[0.95mm -1.1mm 0.95mm -1.6mm 0.6mm 2000 0.74mm "4" "4" "square"]
	Pad[0.95mm 1.1mm 0.95mm 1.6mm 0.6mm 2000 0.74mm "3" "3" "square"]
	Pad[0mm 1.1mm 0mm 1.6mm 0.6mm 2000 0.74mm "2" "2" "square"]
	Pad[-0.95mm 1.1mm -0.95mm 1.6mm 0.6mm 2000 0.74mm "1" "1" "square"]

	ElementLine[-1.6mm 0.9mm -1.6mm 0.9mm 2000]
	ElementLine[1.525mm 0.875mm 1.525mm -0.875mm 800]
	ElementLine[-1.525mm 0.875mm -1.525mm -0.875mm 800]
)
