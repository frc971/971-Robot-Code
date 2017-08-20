# This is the footprint at
# https://web.archive.org/web/20131102032848/http://industrial.panasonic.com/www-data/pdf/ABA0000/ABA0000PE251.pdf
# for a 8x10.2 capacitor (size F).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 2.55mm 0 4.55mm 2mm 2000 9874 "1" "1" "square"]
	Pad[0 -2.55mm 0 -4.55mm 2mm 2000 9874 "2" "2" "square"]

    ElementLine[-10000 -24000 10000 -24000 1200]

	ElementLine[-4.25mm 4.25mm -4.25mm -4.25mm 800]
	ElementLine[4.25mm 4.25mm 4.25mm -4.25mm 800]
	ElementLine[4.25mm 4.25mm 2.75mm 5.75mm 800]
	ElementLine[-4.25mm 4.25mm -2.75mm 5.75mm 800]
)
