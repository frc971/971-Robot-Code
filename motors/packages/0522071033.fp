# This is the footprint at
# http://www.molex.com/pdm_docs/sd/152670201_sd.pdf.
# Also works for 0522711079 (bottom contact version).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[-4.5mm 0.65mm -4.5mm -0.65mm 0.6mm 2000 0.75mm "1" "1" "square"]
	Pad[-3.5mm 0.65mm -3.5mm -0.65mm 0.6mm 2000 0.75mm "2" "2" "square"]
	Pad[-2.5mm 0.65mm -2.5mm -0.65mm 0.6mm 2000 0.75mm "3" "3" "square"]
	Pad[-1.5mm 0.65mm -1.5mm -0.65mm 0.6mm 2000 0.75mm "4" "4" "square"]
	Pad[-0.5mm 0.65mm -0.5mm -0.65mm 0.6mm 2000 0.75mm "5" "5" "square"]
	Pad[0.5mm 0.65mm 0.5mm -0.65mm 0.6mm 2000 0.75mm "6" "6" "square"]
	Pad[1.5mm 0.65mm 1.5mm -0.65mm 0.6mm 2000 0.75mm "7" "7" "square"]
	Pad[2.5mm 0.65mm 2.5mm -0.65mm 0.6mm 2000 0.75mm "8" "8" "square"]
	Pad[3.5mm 0.65mm 3.5mm -0.65mm 0.6mm 2000 0.75mm "9" "9" "square"]
	Pad[4.5mm 0.65mm 4.5mm -0.65mm 0.6mm 2000 0.75mm "10" "10" "square"]

	Pad[7.65mm 2mm 7.65mm 2.7mm 2.1mm 2000 2.25mm "" "" "square"]
	Pad[-7.65mm 2mm -7.65mm 2.7mm 2.1mm 2000 2.25mm "" "" "square"]

	ElementLine[6.35mm 1.35mm 4.8mm 1.35mm 800]
	ElementLine[-6.35mm 1.35mm -4.8mm 1.35mm 800]

	ElementLine[4.95mm 3.15mm 4.95mm 1.55mm 800]
	ElementLine[-4.95mm 3.15mm -4.95mm 1.55mm 800]
	ElementLine[4.95mm 3.15mm 6.35mm 3.15mm 800]
	ElementLine[-4.95mm 3.15mm -6.35mm 3.15mm 800]

	ElementLine[5mm 12.25mm -5mm 12.25mm 800]
	ElementLine[5mm 12.25mm 5mm 7.35mm 800]
	ElementLine[-5mm 12.25mm -5mm 7.35mm 800]
)
