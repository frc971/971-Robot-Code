# This is the footprint for a 4mmx4mm 10-lead 0.8mm pitch flat no-lead package.
# It is the recommended land pattern in
# https://www.fairchildsemi.com/datasheets/FA/FAN8811T.pdf.
Element["" "" "" "" 0 0 0 0 0 100 0x0]
(
	# The exposed pad.
	Pad[0.245mm 0 -0.245mm 0 2.66mm 2000 2.6mm "11" "11" "square,nopaste"]
	Pad[0.725mm 0.725mm 0.725mm 0.725mm 1.15mm 0 1.15mm "11" "11" "square"]
	Pad[-0.725mm 0.725mm -0.725mm 0.725mm 1.15mm 0 1.15mm "11" "11" "square"]
	Pad[0.725mm -0.725mm 0.725mm -0.725mm 1.15mm 0 1.15mm "11" "11" "square"]
	Pad[-0.725mm -0.725mm -0.725mm -0.725mm 1.15mm 0 1.15mm "11" "11" "square"]
	Pin[0 0 0.75mm 2000 100 0.3mm "11" "11" ""]
	Pin[0.75mm 0.75mm 0.75mm 2000 100 0.3mm "11" "11" ""]
	Pin[-0.75mm 0.75mm 0.75mm 2000 100 0.3mm "11" "11" ""]
	Pin[0.75mm -0.75mm 0.75mm 2000 100 0.3mm "11" "11" ""]
	Pin[-0.75mm -0.75mm 0.75mm 2000 100 0.3mm "11" "11" ""]

	# The top row of pads.
	Pad[-1.6mm -1.725mm -1.6mm -2.225mm 0.35mm 0.18mm 0.53mm "10" "10" ""]
	Pad[-0.8mm -1.725mm -0.8mm -2.225mm 0.35mm 0.18mm 0.53mm "9" "9" ""]
	Pad[0mm -1.725mm 0mm -2.225mm 0.35mm 0.18mm 0.53mm "8" "8" ""]
	Pad[0.8mm -1.725mm 0.8mm -2.225mm 0.35mm 0.18mm 0.53mm "7" "7" ""]
	Pad[1.6mm -1.725mm 1.6mm -2.225mm 0.35mm 0.18mm 0.53mm "6" "6" ""]

	# The bottom row of pads.
	Pad[-1.6mm 1.725mm -1.6mm 2.225mm 0.35mm 0.18mm 0.53mm "1" "1" ""]
	Pad[-0.8mm 1.725mm -0.8mm 2.225mm 0.35mm 0.18mm 0.53mm "2" "2" ""]
	Pad[0mm 1.725mm 0mm 2.225mm 0.35mm 0.18mm 0.53mm "3" "3" ""]
	Pad[0.8mm 1.725mm 0.8mm 2.225mm 0.35mm 0.18mm 0.53mm "4" "4" ""]
	Pad[1.6mm 1.725mm 1.6mm 2.225mm 0.35mm 0.18mm 0.53mm "5" "5" ""]

	ElementLine[-2.05mm -2.05mm -2.05mm 2.05mm 800]
	ElementLine[2.05mm -2.05mm 2.05mm 2.05mm 800]
	ElementArc[-2.05mm 1.9mm 0.2mm 0.2mm 270 180 800]
)
