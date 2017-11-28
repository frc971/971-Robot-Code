# This is the footprint at
# https://www.molex.com/pdm_docs/sd/5023520300_sd.pdf for a 5-pin header.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[-4mm -3mm -4mm -5mm 1.2mm 2000 1.5mm "1" "1" "square"]
	Pad[-2mm -3mm -2mm -5mm 1.2mm 2000 1.5mm "2" "2" "square"]
	Pad[0 -3mm 0 -5mm 1.2mm 2000 1.5mm "3" "3" "square"]
	Pad[2mm -3mm 2mm -5mm 1.2mm 2000 1.5mm "4" "4" "square"]
	Pad[4mm -3mm 4mm -5mm 1.2mm 2000 1.5mm "5" "5" "square"]

	Pad[6.55mm 3.28mm 6.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]
	Pad[-6.55mm 3.28mm -6.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]

	ElementLine[5mm 4.13mm -5mm 4.13mm 1200]
	ElementLine[5mm -5.6mm 5.7mm -5.6mm 1200]
	ElementLine[5.7mm -5.6mm 7.4mm -2.4mm 1200]
	ElementLine[-5mm -5.6mm -5.7mm -5.6mm 1200]
	ElementLine[-5.7mm -5.6mm -7.4mm -2.4mm 1200]
)
