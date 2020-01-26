# This is the footprint recommended by TDK in:
# https://product.tdk.com/info/en/catalog/datasheets/inductor_automotive_power_clf7045ni-d_en.pdf
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-2.75mm 0mm -2.95mm 0mm 2.2mm 2000 2.4mm "1" "1" "square"]
	Pad[2.75mm 0mm 2.95mm 0mm 2.2mm 2000 2.4mm "2" "2" "square"]

	ElementLine[4.5mm 3.7mm -4.5mm 3.7mm 1000]
	ElementLine[4.5mm -3.7mm -4.5mm -3.7mm 1000]
	ElementLine[4.5mm -3.7mm 4.5mm 3.7mm 1000]
       	ElementLine[-4.5mm -3.7mm -4.5mm 3.7mm 1000] 
)
