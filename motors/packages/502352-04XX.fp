# This is the footprint at
# https://www.molex.com/pdm_docs/sd/5023520300_sd.pdf for a 4-pin header.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[-3mm -3mm -3mm -5mm 1.2mm 2000 1.5mm "1" "1" "square"]
	Pad[-1mm -3mm -1mm -5mm 1.2mm 2000 1.5mm "2" "2" "square"]
	Pad[1mm -3mm 1mm -5mm 1.2mm 2000 1.5mm "3" "3" "square"]
	Pad[3mm -3mm 3mm -5mm 1.2mm 2000 1.5mm "4" "4" "square"]

	Pad[5.55mm 3.28mm 5.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]
	Pad[-5.55mm 3.28mm -5.55mm -0.32mm 1.7mm 2000 2mm "" "" "square"]

	ElementLine[4mm 4.13mm -4mm 4.13mm 1200]
	ElementLine[4mm -5.6mm 4.7mm -5.6mm 1200]
	ElementLine[4.7mm -5.6mm 6.4mm -2.4mm 1200]
	ElementLine[-4mm -5.6mm -4.7mm -5.6mm 1200]
	ElementLine[-4.7mm -5.6mm -6.4mm -2.4mm 1200]
)
