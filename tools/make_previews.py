"""Optional developer helper: gallery of actual OpenGL captures (requires Pillow)."""
from pathlib import Path
import subprocess

from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parents[1]
subprocess.run([str(root / "build/TrafficSim.exe"), "--model-preview"], check=True)
output = root / "previews"
output.mkdir(exist_ok=True)
gallery = Image.new("RGB", (1320, 1010), "#e9edeb")
draw = ImageDraw.Draw(gallery)
title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 25)
caption = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 16)
for col, (model, name, detail) in enumerate([
    ("wave", "Honda Wave Alpha 110", "Xe số · Vành 17 inch"),
    ("vision", "Honda Vision", "Tay ga tiêu chuẩn · Vành 14 inch"),
]):
    x = 12 + col * 654
    draw.text((x + 6, 15), name, font=title, fill="#20323a")
    draw.text((x + 6, 53), detail, font=caption, fill="#415a64")
    for row, angle in enumerate(["front", "side"]):
        with Image.open(root / "build" / f"{model}-{angle}.ppm") as shot:
            w, h = shot.size
            frame = shot.crop((w // 2 - 500, h // 2 - 378, w // 2 + 500, h // 2 + 338))
            gallery.paste(frame.resize((642, 460), Image.Resampling.LANCZOS), (x, 82 + row * 462))
    with Image.open(root / "build" / f"{model}-dimensions.ppm") as shot:
        shot.save(output / f"{model}-dimensions.png")
gallery.save(output / "Wave-Vision.png")
print(f"Saved previews to {output}")
