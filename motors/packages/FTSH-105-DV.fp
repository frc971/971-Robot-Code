# This is the footprint for 10-pin SMT 0.05" connectors recommended in
# http://suddendocs.samtec.com/prints/ftsh-1xx-xx-xxx-dv-xxx-footprint.pdf.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-0.1in 0.1205in -0.1in 0.0395in 0.029in 2000 0.043in "1" "1" "square"]
	Pad[-0.05in 0.1205in -0.05in 0.0395in 0.029in 2000 0.043in "3" "3" "square"]
	Pad[0 0.1205in 0 0.0395in 0.029in 2000 0.043in "5" "5" "square"]
	Pad[0.05in 0.1205in 0.05in 0.0395in 0.029in 2000 0.043in "7" "7" "square"]
	Pad[0.1in 0.1205in 0.1in 0.0395in 0.029in 2000 0.043in "9" "9" "square"]

	Pad[-0.1in -0.1205in -0.1in -0.0395in 0.029in 2000 0.043in "2" "2" "square"]
	Pad[-0.05in -0.1205in -0.05in -0.0395in 0.029in 2000 0.043in "4" "4" "square"]
	Pad[0 -0.1205in 0 -0.0395in 0.029in 2000 0.043in "6" "6" "square"]
	Pad[0.05in -0.1205in 0.05in -0.0395in 0.029in 2000 0.043in "8" "8" "square"]
	Pad[0.1in -0.1205in 0.1in -0.0395in 0.029in 2000 0.043in "10" "10" "square"]

	ElementLine[-0.1in -0.012in 0.1in -0.012in 800]
	ElementLine[-0.1in 0.012in -0.0425in 0.012in 800]
	ElementLine[0.1in 0.012in 0.0425in 0.012in 800]
	ElementArc[-0.1in 0.007in 0.005in 0.005in 0 360 800]
)
