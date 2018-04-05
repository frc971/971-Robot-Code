# This is the footprint for the FG18 through-hole capacitor family from
# https://product.tdk.com/info/en/catalog/datasheets/leadmlcc_halogenfree_fg_en.pdf.
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pin[-1.25mm 0 54mil 30mil 60mil 35mil "1" "1" ""]
	Pin[1.25mm 0 54mil 30mil 60mil 35mil "2" "2" ""]

	ElementLine[-2.4mm 1.25mm 2.4mm 1.25mm 1200]
	ElementLine[2.4mm 1.25mm 2.4mm -1.25mm 1200]
	ElementLine[2.4mm -1.25mm -2.4mm -1.25mm 1200]
	ElementLine[-2.4mm -1.25mm -2.4mm 1.25mm 1200]
)
