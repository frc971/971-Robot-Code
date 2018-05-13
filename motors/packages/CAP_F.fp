# This is the footprint at
# https://web.archive.org/web/20131102032848/http://industrial.panasonic.com/www-data/pdf/ABA0000/ABA0000PE251.pdf
# for a 8x10.2 capacitor (size E or F).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 2.55mm 0 4.55mm 2mm 2000 2.3mm "1" "1" "square"]
	Pad[0 -2.55mm 0 -4.55mm 2mm 2000 2.3mm "2" "2" "square"]

	ElementLine[-4mm -5mm -1.75mm -5mm 0.5mm]
	ElementLine[4mm -5mm 1.75mm -5mm 0.5mm]

	#ElementLine[-4.25mm 4.25mm -4.25mm -4.25mm 800]
	#ElementLine[4.25mm 4.25mm 4.25mm -4.25mm 800]
	#ElementLine[4.25mm 4.25mm 2.75mm 5.75mm 800]
	#ElementLine[-4.25mm 4.25mm -2.75mm 5.75mm 800]
	ElementLine[-4.75mm 4.325mm -4.75mm -4mm 0.25mm]
	ElementLine[4.75mm 4.325mm 4.75mm -4mm 0.25mm]
	ElementLine[4.75mm 4.325mm 3.95mm 5.125mm 0.25mm]
	ElementLine[-4.75mm 4.325mm -3.95mm 5.125mm 0.25mm]
	ElementLine[3.95mm 5.125mm 1.8mm 5.125mm 0.25mm]
	ElementLine[-3.95mm 5.125mm -1.8mm 5.125mm 0.25mm]
)
