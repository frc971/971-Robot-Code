# A footprint for "female" ZH-compatible connectors.
# This is based on information in http://www.jst-mfg.com/product/pdf/eng/eZR.pdf
# and http://www.jst-mfg.com/product/pdf/eng/eZH.pdf.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	# The main pins.
	Pin[-2.25mm 0 1.1mm 2000 1.3mm 0.7mm "4" "4" ""]
	Pin[-0.75mm 0 1.1mm 2000 1.3mm 0.7mm "3" "3" ""]
	Pin[0.75mm 0 1.1mm 2000 1.3mm 0.7mm "2" "2" ""]
	Pin[2.25mm 0 1.1mm 2000 1.3mm 0.7mm "1" "1" ""]

	# Lines along the edges of the ZH connector.
	ElementLine[-3.75mm 2mm 3.75mm 2mm 800]
	ElementLine[-3.75mm -1.5mm 3.75mm -1.5mm 800]
)
