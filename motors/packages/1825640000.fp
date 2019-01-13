# This is the footprint at
# https://media.digikey.com/pdf/Data%20Sheets/Weidmuller%20PDFs/1825640000.pdf
# for a 2-position big Weidmuller push in spring terminal.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pin[-1.75mm 0 1.6mm 2000 1.85mm 1.1mm "1" "1" ""]
	Pin[1.75mm 0 1.6mm 2000 1.85mm 1.1mm "2" "2" ""]
	Pin[-1.75mm 5.5mm 1.6mm 2000 1.85mm 1.1mm "1" "1" ""]
	Pin[1.75mm 5.5mm 1.6mm 2000 1.85mm 1.1mm "2" "2" ""]

	ElementLine[-3.35mm -1.4mm 3.35mm -1.4mm 800]
	ElementLine[-3.35mm 6.9mm 3.35mm 6.9mm 800]
	ElementLine[-3.35mm -1.4mm -3.35mm 6.9mm 800]
	ElementLine[3.35mm -1.4mm 3.35mm 6.9mm 800]

	ElementArc[-1.75mm 1.8mm 0.7mm 0.7mm 0 360 800]
	ElementLine[-2.45mm 4.3mm -1.05mm 4.3mm 800]
	ElementLine[-1.05mm 4.3mm -1.05mm 2.9mm 800]
	ElementLine[-1.05mm 2.9mm -2.45mm 2.9mm 800]
	ElementLine[-2.45mm 2.9mm -2.45mm 4.3mm 800]
	ElementArc[1.75mm 1.8mm 0.7mm 0.7mm 0 360 800]
	ElementLine[2.45mm 4.3mm 1.05mm 4.3mm 800]
	ElementLine[1.05mm 4.3mm 1.05mm 2.9mm 800]
	ElementLine[1.05mm 2.9mm 2.45mm 2.9mm 800]
	ElementLine[2.45mm 2.9mm 2.45mm 4.3mm 800]
)
