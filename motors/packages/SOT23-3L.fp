# This is the footprint recommended in
# https://media.digikey.com/pdf/Data%20Sheets/ST%20Microelectronics%20PDFS/ESDCAN(01,24)-2BLY,.pdf.
# This package seems to be equivalent to SOT23-3 too.
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-0.97mm -0.735mm -0.97mm -1.205mm 0.48mm 2000 0.68mm "2" "2" "square"]
	Pad[0.97mm -0.735mm 0.97mm -1.205mm 0.48mm 2000 0.68mm "1" "1" "square"]
	Pad[0 0.735mm 0 1.205mm 0.48mm 2000 0.68mm "3" "3" "square"]

	ElementLine[0.6mm 0.845mm 1.62mm 0.845mm 800]
	ElementLine[1.62mm 0.845mm 1.62mm -0.845mm 800]
	ElementLine[-0.6mm 0.845mm -1.62mm 0.845mm 800]
	ElementLine[-1.62mm 0.845mm -1.62mm -0.845mm 800]
	ElementLine[0.5mm -0.845mm -0.5mm -0.845mm 800]
)
