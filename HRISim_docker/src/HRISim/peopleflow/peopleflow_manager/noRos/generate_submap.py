#!/usr/bin/env python3
from PIL import Image

img = Image.open("map.pgm")
submap = img.resize((2052, 2052), Image.NEAREST)
submap.save("submap_0.pgm")
print(f"submap_0.pgm generata: {submap.size}")
