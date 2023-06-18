ENV["FONTCONFIG_FILE"] = "/tmp/fontcfg"
require "gtk3"
$layout_creator = Pango::Context.new()
$layout_creator.font_map = Pango::CairoFontMap.default
Desc = Pango::FontDescription.new(
	RUBY_PLATFORM =~ /linux/ ? "Fixed Medium Semi-Condensed 10"
#: "Lucida Console Normal 9")
: "Lucida Sans Unicode Normal 12")

def get_default_pango()
	return Pango::Layout.new($layout_creator)
end
def update_pango_context(cr)
	cr.update_pango_context($layout_creator)
end

def get_desc_pango()
	p = get_default_pango()
	p.set_font_description(Desc)
	return p
end

def get_text_desc_pango(txt)
	p = get_default_pango()
	p.set_font_description(Desc)
	p.set_text(txt)
	return p
end
