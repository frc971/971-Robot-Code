class Color:

    def __init__(self, r, g, b, a=1.0):
        self.r = r
        self.g = g
        self.b = b
        self.a = a


palette = {
    "RED": Color(1, 0, 0),
    "GREEN": Color(0, 1, 0),
    "BLUE": Color(0, 0, 1),
    "YELLOW": Color(1, 1, 0),
    "MAGENTA": Color(1, 0, 1),
    "CYAN": Color(0, 1, 1),
    "BLACK": Color(0, 0, 0),
    "WHITE": Color(1, 1, 1),
    "GREY": Color(0.5, 0.5, 0.5),
    "LIGHT_GREY": Color(0.75, 0.75, 0.75),
    "DARK_GREY": Color(0.25, 0.25, 0.25),
    "ORANGE": Color(1, 0.65, 0)
}
