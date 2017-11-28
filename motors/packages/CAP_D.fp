# This is the footprint at
# http://www.nichicon.co.jp/english/products/pdfs/e-ch_ref.pdf
# for a 6.3mm diameter capacitor (size D).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 1.75mm 0 3.65mm 1.6mm 2000 1.85mm "1" "1" "square"]
	Pad[0 -1.75mm 0 -3.65mm 1.6mm 2000 1.85mm "2" "2" "square"]

    ElementLine[-3.55mm -3.9mm -1.5mm -3.9mm 2000]
    ElementLine[3.55mm -3.9mm 1.5mm -3.9mm 2000]

	ElementLine[-3.8mm 2.95mm -3.8mm -3.2mm 800]
	ElementLine[3.8mm 2.95mm 3.8mm -3.2mm 800]
	ElementLine[3.8mm 2.95mm 2.6mm 4.15mm 800]
	ElementLine[-3.8mm 2.95mm -2.6mm 4.15mm 800]
)
