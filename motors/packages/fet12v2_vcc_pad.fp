Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	# Note that a 0-width mask opening works fine. 0-thickness pads just fail
	# to change apertures in the gerber exporter, which ends poorly, but 0-width
	# mask gets filtered out somewhere before making it to the gerber-drawing.

	Pad[120mil 190mil 415mil 190mil 120mil 2000 0 "1" "1" "square,nopaste"]
	Pad[-120mil 190mil -415mil 190mil 120mil 2000 0 "1" "1" "square,nopaste"]
	Pad[-50mil 190mil 50mil 190mil 20mil 2000 0 "1" "1" "square,nopaste"]
	Pad[0 140mil 0 240mil 20mil 2000 0 "1" "1" "square,nopaste"]
	Pad[-457.5mil 267.5mil 457.5mil 267.5mil 35mil 2000 0 "1" "1" "square,nopaste"]
	Pad[-365mil 20mil 365mil 20mil 220mil 2000 0 "1" "1" "square,nopaste"]

	Pad[120mil 190mil 415mil 190mil 120mil 2000 135mil "1" "1" "square,onsolder,nopaste"]
	Pad[-120mil 190mil -525mil 190mil 120mil 2000 135mil "1" "1" "square,onsolder,nopaste"]
	Pad[-50mil 190mil 50mil 190mil 20mil 2000 0 "1" "1" "square,onsolder,nopaste"]
	Pad[0 140mil 0 240mil 20mil 2000 0 "1" "1" "square,onsolder,nopaste"]
	Pad[-565mil 270mil 455mil 270mil 40mil 2000 55mil "1" "1" "square,onsolder,nopaste"]
	Pad[-475mil 20mil 365mil 20mil 220mil 2000 235mil "1" "1" "square,onsolder,nopaste"]
)
