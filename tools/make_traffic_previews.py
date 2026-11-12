"""Optional Pillow helper: actual OpenGL traffic frames; GIF plays at 2x simulation speed."""
from pathlib import Path
import subprocess

from PIL import Image

root = Path(__file__).resolve().parents[1]
subprocess.run([str(root / "build/TrafficSim.exe"), "--traffic-preview"], check=True)
output = root / "previews"
output.mkdir(exist_ok=True)
generated = []
for phase in range(6):
    path = root / "build" / f"traffic-{phase}.ppm"
    generated.append(path)
    with Image.open(path) as im:
        im.save(output / f"Traffic-phase-{phase}.png")
frames = []
for index in range(48):
    path = root / "build" / f"flow-{index}.ppm"
    generated.append(path)
    with Image.open(path) as im:
        frames.append(im.resize((960, 600), Image.Resampling.LANCZOS))
palette_source = Image.new("RGB", (960, 450))
for index, im in enumerate(frames):
    palette_source.paste(im.resize((120, 75)), ((index % 8) * 120, (index // 8) * 75))
palette = palette_source.quantize(colors=256)
frames = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in frames]
frames[0].save(output / "Traffic-flow.gif", save_all=True, append_images=frames[1:],
               duration=250, loop=0, optimize=False, disposal=2)
for path in generated:
    path.unlink()
print(f"Saved six signal phases and 2x animation to {output}")
