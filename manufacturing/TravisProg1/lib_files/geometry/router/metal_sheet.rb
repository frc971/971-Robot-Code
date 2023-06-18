require "json"
module Router
	class SheetCut
		attr_accessor :layout, :fname
		def initialize(fname)
			@fname = fname
			@layout = SheetLayout.new()
		end
		def save_changes()
			ser = SerializeBuffer::Writer.new()
			serialize(ser)
			data = ser.data
			len = SerializeBuffer.get_concat_length(data)
			buffer = OutputView.new(len)
			SerializeBuffer.concat_output(buffer, data)
			File.open(@fname, "wb+") { |f| f.write(buffer.val) }
		end
		def inspect()
			lns = @layout.items.collect { |a|
				a.item.filename.name()
			}
			return "<SheetCut: #{@fname}, #{lns.inspect}>"
		end
		def layout() @layout end
	end
	class MetalSheet
		def initialize(sheet_info, sheet_folder)
			@sheet_folder = sheet_folder
			@cuts = []
			@sheet_info = sheet_info
			@active_cut = nil
		end
		def sheet_info() @sheet_info end
		def load_cut(fname, pm_loader)
			puts "loading cut: #{fname}"
			cut = SheetCut.new(fname)
			SheetCut.load(cut, pm_loader)
			@cuts << cut
		end
		def cuts() @cuts end
		def inspect() "<MetalSheet: #{@sheet_folder}, #{@sheet_info}, #{@cuts.inspect}>" end
		def add_cut(name)
			@cuts << cut = SheetCut.new(@sheet_folder + "#{name}.cut")
			cut
		end
		def save_changes()
			@cuts.each do |cut|
				cut.save_changes()
			end
		end
		def active_cut_change(&blk)
			@active_cut_change = blk
			@active_cut_change.call(@active_cut) if @active_cut
		end
		def sheet_name() @sheet_folder.basename.to_path end
		def active_cut() @active_cut end
		def set_active(cut)
			@active_cut_change.call(cut) if @active_cut_change
			@active_cut = cut
		end
		def empty?() @cuts.empty?() end
	end
	class MetalSheetLoader
		def initialize(sheet, cuts)
			@sheet_name = sheet
			@sheet = SheetInfo.parse(sheet)
			@cuts = cuts
		end
		def sheet_name()
			@sheet_name.dirname.basename.to_path
		end
		def self.make(folder)
			c = folder.children.select(&:file?)
			sheets = [] 
			cuts = []
			c.each do |child|
				sheets << child if (child.extname == ".sheet")
				cuts << child if (child.extname == ".cut")
			end
			return nil if (sheets.length != 1)
			return MetalSheetLoader.new(sheets[0], cuts)
		end
		def open(pm_loader)
			ms = MetalSheet.new(@sheet, @sheet_name.dirname)
			@cuts.each do |cut|
				ms.load_cut(cut, pm_loader)
			end
			if (active_fname = @sheet.active())
				ms.cuts.each do |cut|
					if (cut.fname.to_path == active_fname)
						ms.set_active(cut)
					end
				end
			end
			return ms
		end
	end
	class SheetDeltaLayout
	end

	class SheetList < Array
		def initialize(folder)
			folder.children.collect do |child|
				c = MetalSheetLoader.make(child)
				self << c if c
			end
			@open_sheets = {}
		end
		def open_sheet(i, parts_lib)
			sheet_load = self[i]
			name = sheet_load.sheet_name
			return @open_sheets[name] ||= sheet_load.open(parts_lib)
		end
	end
	class SheetInfo
		def initialize(fname, json, w, h)
			@fname = fname
			@json = json
			@w, @h = w, h
		end
		def self.parse(fname)
			json = JSON.parse(File.read(fname))
			w = json["w"]
			h = json["h"]
			self.new(fname, json, w.to_f, h.to_f)
		end
		def set_active(cut)
			@json["active"] = cut.fname.to_path
			File.open(@fname, "w+") { |f| f.write(JSON.dump(@json)) }
		end
		def active()
			@json["active"]
		end
		def sheet_dxf()
			fname = @json["sheet_dxf"]
			return nil if !fname
			@sheet_dxf ||= (
			items = get_entities(fname)
			items2d = items.collect { |item| item.to_2d() }

			items2d_orient = Oriented.import_2d(items2d)
#			items2d_orient = SweepLine.stitch_curves(items2d_orient)
			items2d_orient
			)
		end
		def hole_file()
			hf = @json["hole_file"]
			raise "sheet: #{@fname} missing hole filename" if !hf
			return hf
		end
	end
end

require_relative "file_formats.rb"
