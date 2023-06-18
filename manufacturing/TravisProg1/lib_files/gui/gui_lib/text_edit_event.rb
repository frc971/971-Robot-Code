class TextEditEvent
	def self.key_press(obj, txt, ev, cursor)
		k = ev.keyval
    if (k >= 32 && k <= 126) 
      txt.insert(cursor[0], k.chr)
			obj.update_txt()
			cursor[0] += 1
    elsif k == Gdk::Keyval::KEY_Right && cursor[0] < txt.length
     	cursor[0] += 1
    elsif k == Gdk::Keyval::KEY_BackSpace && cursor[0] > 0
      cursor[0] -= 1
			txt.slice!(cursor[0], 1)
			obj.update_txt()
    elsif k == Gdk::Keyval::KEY_Delete && cursor[0] < txt.length
			txt.slice!(cursor[0], 1)
			obj.update_txt()
    elsif k == Gdk::Keyval::KEY_Left && cursor[0] > 0
      cursor[0] -= 1
		elsif k == Gdk::Keyval::KEY_Home
			cursor[0] = 0
		elsif k == Gdk::Keyval::KEY_End
			cursor[0] = txt.length
		else
			return false
		end
		return true
	end
end

