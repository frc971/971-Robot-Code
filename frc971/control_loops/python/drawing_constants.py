import cairo
from color import Color, palette
import numpy as np


def set_color(cr, color, a=1):
    if color.a == 1.0:
        cr.set_source_rgba(color.r, color.g, color.b, a)
    else:
        cr.set_source_rgba(color.r, color.g, color.b, color.a)


def draw_px_cross(cr, x, y, length_px, color=palette["RED"]):
    """Draws a cross with fixed dimensions in pixel space."""
    set_color(cr, color)
    cr.move_to(x, y - length_px)
    cr.line_to(x, y + length_px)
    cr.stroke()

    cr.move_to(x - length_px, y)
    cr.line_to(x + length_px, y)
    cr.stroke()
    set_color(cr, palette["WHITE"])


def draw_px_x(cr, x, y, length_px1, color=palette["BLACK"]):
    """Draws a x with fixed dimensions in pixel space."""
    length_px = length_px1 / np.sqrt(2)
    set_color(cr, color)
    cr.move_to(x - length_px, y - length_px)
    cr.line_to(x + length_px, y + length_px)
    cr.stroke()

    cr.move_to(x - length_px, y + length_px)
    cr.line_to(x + length_px, y - length_px)
    cr.stroke()
    set_color(cr, palette["WHITE"])


def draw_circle(cr, x, y, radius, color=palette["RED"]):
    set_color(cr, color)
    cr.arc(x, y, radius, 0, 2 * np.pi)
    cr.fill()
    cr.stroke()


def draw_control_points_cross(cr,
                              points,
                              width=10,
                              radius=4,
                              color=palette["BLUE"]):
    for i in range(0, len(points)):
        draw_px_x(cr, points[i][0], points[i][1], width, color)
        set_color(cr, color)
        cr.arc(points[i][0], points[i][1], radius, 0, 2.0 * np.pi)
        cr.fill()
        set_color(cr, palette["WHITE"])


def display_text(cr, text, widtha, heighta, widthb, heightb):
    cr.scale(widtha, -heighta)
    cr.show_text(text)
    cr.scale(widthb, -heightb)


def draw_points(cr, p, size):
    for i in range(0, len(p)):
        draw_px_cross(cr, p[i][0], p[i][1], size,
                      Color(0, np.sqrt(0.2 * i), 0))
