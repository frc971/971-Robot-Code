# This is the footprint at
# http://www.nichicon.co.jp/english/products/pdfs/e-ch_ref.pdf
# for a 10mm diameter capacitor (size G).
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pad[0 5.25mm 0 6.75mm 2.5mm 2000 10743 "1" "1" "square"]
	Pad[0 -5.25mm 0 -6.75mm 2.5mm 2000 10743 "2" "2" "square"]

    ElementLine[-5mm -7.5mm -2mm -7.5mm 2000]
    ElementLine[5mm -7.5mm 2mm -7.5mm 2000]

	ElementLine[-5.25mm 5.25mm -5.25mm -5.25mm 800]
	ElementLine[5.25mm 5.25mm 5.25mm -5.25mm 800]
	ElementLine[5.25mm 5.25mm 2.75mm 7.75mm 800]
	ElementLine[-5.25mm 5.25mm -2.75mm 7.75mm 800]
)
