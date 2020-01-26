# This is the footprint recommended by TDK in:
# https://product.tdk.com/info/en/catalog/datasheets/inductor_commercial_power_spm12565xt_en.pdf
#
Element["" "" "" "" 0 0 0 0 0 100 ""]
(
	Pad[-3.65mm 0mm -6.05mm 0mm 2.8mm 2000 3mm "1" "1" "square"]
	Pad[3.65mm 0mm 6.05mm 0mm 2.8mm 2000 3mm "2" "2" "square"]

	ElementLine[-7.85mm 6.6mm -7.85mm -4.6mm 1000]
	ElementLine[7.85mm 6.6mm 7.85mm -6.6mm 1000]
	ElementLine[7.85mm 6.6mm -7.85mm 6.6mm 1000]
       	ElementLine[7.85mm -6.6mm -5.85mm -6.6mm 1000] 
        ElementLine[-7.85mm -4.6mm -5.85mm -6.6mm 1000]
)
