# This is the footprint at
# https://web.archive.org/web/20131102032848/http://industrial.panasonic.com/www-data/pdf/ABA0000/ABA0000PE251.pdf
# for a 10mm diameter capacitor (size G).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 3.3mm 0 5.4mm 2.0mm 2000 2.3mm "1" "1" "square"]
	Pad[0 -3.3mm 0 -5.4mm 2.0mm 2000 2.3mm "2" "2" "square"]

	ElementLine[-5mm -5.85mm -2mm -5.85mm 0.5mm]
	ElementLine[5mm -5.85mm 2mm -5.85mm 0.5mm]

	ElementLine[-5.75mm 4.275mm -5.75mm -5mm 0.25mm]
	ElementLine[5.75mm 4.275mm 5.75mm -5mm 0.25mm]
	ElementLine[5.75mm 4.275mm 4.05mm 5.975mm 0.25mm]
	ElementLine[-5.75mm 4.275mm -4.05mm 5.975mm 0.25mm]
	ElementLine[4.05mm 5.975mm 2mm 5.975mm 0.25mm]
	ElementLine[-4.05mm 5.975mm -2mm 5.975mm 0.25mm]
)
