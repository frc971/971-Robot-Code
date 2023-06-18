require_relative "dxf_parse.rb"
require_relative "entities.rb"
require_relative "profile_utils.rb"
require_relative "entity_draw.rb"
require_relative "make_loops.rb"
require_relative "2dpoint.rb"


def full_load_dxf(fname)
	items = get_entities(fname)
	items2d = items.collect { |item| item.to_2d() }

	items2d_orient = Oriented.import_2d(items2d)
	items2d_orient = SweepLine.stitch_curves(items2d_orient)
	return items2d_orient
end
