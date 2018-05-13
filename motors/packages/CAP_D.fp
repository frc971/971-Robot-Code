# This is the footprint at
# https://web.archive.org/web/20131102032848/http://industrial.panasonic.com/www-data/pdf/ABA0000/ABA0000PE251.pdf
# for a 6.3mm diameter capacitor (size D).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 1.7mm 0 3.3mm 1.6mm 2000 1.9mm "1" "1" "square"]
	Pad[0 -1.7mm 0 -3.3mm 1.6mm 2000 1.9mm "2" "2" "square"]

	ElementLine[-3.7mm -3.55mm -1.5mm -3.55mm 0.5mm]
	ElementLine[3.7mm -3.55mm 1.5mm -3.55mm 0.5mm]

	ElementLine[-3.825mm 2.475mm -3.825mm -3mm 0.25mm]
	ElementLine[3.825mm 2.475mm 3.825mm -3mm 0.25mm]
	ElementLine[3.825mm 2.475mm 2.625mm 3.675mm 0.25mm]
	ElementLine[-3.825mm 2.475mm -2.625mm 3.675mm 0.25mm]
	ElementLine[2.625mm 3.675mm 1.5mm 3.675mm 0.25mm]
	ElementLine[-2.625mm 3.675mm -1.5mm 3.675mm 0.25mm]
)
