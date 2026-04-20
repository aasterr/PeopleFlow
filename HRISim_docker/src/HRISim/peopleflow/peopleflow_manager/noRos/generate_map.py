#!/usr/bin/env python3
from PIL import Image
import numpy as np

RESOLUTION = 0.05
ORIGIN_X   = -9.1
ORIGIN_Y   = -11.1

WIDTH  = int((6.1  - ORIGIN_X) / RESOLUTION)
HEIGHT = int((2.4  - ORIGIN_Y) / RESOLUTION)

FREE     = 205
OCCUPIED = 0
UNKNOWN  = 127

def world_to_pixel(wx, wy):
    col = int((wx - ORIGIN_X) / RESOLUTION)
    row = HEIGHT - 1 - int((wy - ORIGIN_Y) / RESOLUTION)
    return col, row

def fill_rect(x_min, x_max, y_min, y_max, val):
    c0, r1 = world_to_pixel(x_min, y_min)
    c1, r0 = world_to_pixel(x_max, y_max)
    c0, c1 = sorted([c0, c1])
    r0, r1 = sorted([r0, r1])
    img[r0:r1+1, c0:c1+1] = val

img = np.full((HEIGHT, WIDTH), UNKNOWN, dtype=np.uint8)

# Spazio libero
fill_rect(-8.0, +5.0,  -1.35, +1.35, FREE)   # braccio orizzontale
fill_rect(-1.35, +1.35, -10.0, -1.35, FREE)  # braccio verticale

# Muri
W = 0.15
fill_rect(-8.0,    +5.0,    +1.35-W, +1.35+W, OCCUPIED)  # top
fill_rect(-8.0,   -1.35,   -1.35-W, -1.35+W, OCCUPIED)  # bottom left
fill_rect(+1.35,  +5.0,    -1.35-W, -1.35+W, OCCUPIED)  # bottom right
fill_rect(-8.0-W, -8.0+W,  -1.35,   +1.35,   OCCUPIED)  # end left
fill_rect(+5.0-W, +5.0+W,  -1.35,   +1.35,   OCCUPIED)  # end right
fill_rect(-1.35-W,-1.35+W, -10.0,  -1.35,    OCCUPIED)  # vert left
fill_rect(+1.35-W,+1.35+W, -10.0,  -1.35,    OCCUPIED)  # vert right
fill_rect(-1.35,  +1.35,   -10.0-W,-10.0+W,  OCCUPIED)  # vert bottom

Image.fromarray(img, mode='L').save("map.pgm")
print(f"Generata map.pgm: {WIDTH}x{HEIGHT} px")
