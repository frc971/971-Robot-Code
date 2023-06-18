require_relative "parts_library.rb"
require_relative "metal_sheet.rb"
require_relative "view.rb"

module Router
	def self.do_nesting()
		parts_lib = PartsLibrary.new(Pathname.new("./parts_library"))
		sheets = SheetList.new(Pathname.new("./sheets"))
		puts sheets.inspect
		view = RouterLayoutView.new(parts_lib, sheets)
		view = ClickTypeSplitView.new(view)
		EvWindow.run(view, 1000, 640)
	end
end

# Folder organized list of parts files (folders binding DXF to GCODE).
#
# Stories:
# 
# Add Part to Delta Cut File:
#  - select part
#  - click to place
#
# From Any View:
#  - Save Stuff. [Breadcrums: Go to view selection step; Bac]
#
# Views:
# 
# List of Sheets:
#  - Click sheet name -> sheet-edit
#  
# Edit Sheet:
#  - Add New Delta Sheet Cut. Add new part. Delete selected part. Generate Gcode.
#  - Edit notes.
#  - Select particular part.
#  - Switch to different Delta Sheet Cut. (list of delta-sheet cuts).
#
# TODO(parker):
#  - List + delete items
#  - Serialize active cut choice.
#  - Scrolling on lists that might grow too big.
#  - Visual save indicator.
#  - Error mechanism.
# DONE
#  - Select + emit non-zero origin.
#  - Select active cut. List cuts.
