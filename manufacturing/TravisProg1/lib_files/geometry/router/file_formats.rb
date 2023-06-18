module Router
	class GcodeSpec
		def serialize(ser)
			ser.write_varint(0)
			ser.code_string(full_path())
		end
	end
	class HoleSpec
		def self.serialize(ser)
			ser.write_varint(1)
		end
	end
	class SheetCut
		def serialize(ser)
			ser.push()
			@layout.items.each { |a|
				ser.write_f64(a.c.x)
				ser.write_f64(a.c.y)
				ser.write_f64(a.rot)
				a.item.filename.serialize(ser)
			}
			ser.pop()
		end
		def self.load(skel, pm_loader)
			reader = StreamReader.new(File.open(skel.fname, "rb").read())
			reader.push()
			while (reader.not_done())
				pt = Point2d.load_from(reader)
				ang = reader.parse_f64()
				t = reader.parse_varint()
				pm = nil
				error_spec = nil
				if (t == 0) # GcodeSpec
					pm_fname = reader.parse_string()
					pm = pm_loader.load_pm_string(Pathname.new(pm_fname))
					error_spec = "Could not load #{pm_fname}" if !pm
				elsif (t == 1) # HoleSpec
					pm = pm_loader.load_pm(HoleSpec)
					error_spec = "Could not load #{HoleSpec}" if !pm
				else
					raise "error loading: #{skel.fname}"
				end
				# Comment the following line out to enable skipping and deleting of unknown parts.
				raise error_spec if error_spec
				if (pm)
					skel.layout.add_item(pm, pt, ang)
				end
			end
			reader.pop()
		end
	end
end
