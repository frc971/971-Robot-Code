# This is the footprint at
# https://www.diodes.com/assets/Package-Files/SOD323F.pdf
# for a SOD323F diode (slightly different than SOD323).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 1.1485mm 0 0.8415mm 0.403mm 2000 0.61mm "1" "1" "square"]
	Pad[0 -1.1485mm 0 -0.8415mm 0.403mm 2000 0.61mm "2" "2" "square"]

	ElementLine[0.65mm -1.05mm 0.47mm -1.05mm 1200]
	ElementLine[-0.65mm -1.05mm -0.47mm -1.05mm 1200]

	ElementLine[0.7mm 1.1mm 0.7mm -1.1mm 800]
	ElementLine[-0.7mm 1.1mm -0.7mm -1.1mm 800]
)
