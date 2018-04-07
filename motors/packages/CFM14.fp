# This is the footprint for the CFM14 through-hole resistor family.
# Body length: 3.3mm
# Lead diameter: 0.4
Element[0x0 "" "" "" 0 0 0 0 0 100 0x0]
(
	Pin[-2.8mm 0 1.1mm 30mil 52mil 0.68mm "1" "1" ""]
	Pin[2.8mm 0 1.1mm 30mil 52mil 0.68mm "2" "2" ""]

	ElementLine[-1.65mm 0.85mm 1.65mm 0.85mm 1200]
	ElementLine[1.65mm 0.85mm 1.65mm -0.85mm 1200]
	ElementLine[1.65mm -0.85mm -1.65mm -0.85mm 1200]
	ElementLine[-1.65mm -0.85mm -1.65mm 0.85mm 1200]
)
