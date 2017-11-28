# This is the footprint at
# https://www.molex.com/pdm_docs/sd/5023520300_sd.pdf for a 3-pin header.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[-2mm -3mm -2mm -5mm 1.2mm 2000 1.5mm "1" "1" "square"]
	Pad[0 -3mm 0 -5mm 1.2mm 2000 1.5mm "2" "2" "square"]
	Pad[2mm -3mm 2mm -5mm 1.2mm 2000 1.5mm "3" "3" "square"]

	Pad[4.55mm 3.28mm 4.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]
	Pad[-4.55mm 3.28mm -4.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]

	ElementLine[3mm 4.13mm -3mm 4.13mm 1200]
	ElementLine[3mm -5.6mm 3.7mm -5.6mm 1200]
	ElementLine[3.7mm -5.6mm 5.4mm -2.4mm 1200]
	ElementLine[-3mm -5.6mm -3.7mm -5.6mm 1200]
	ElementLine[-3.7mm -5.6mm -5.4mm -2.4mm 1200]
)
