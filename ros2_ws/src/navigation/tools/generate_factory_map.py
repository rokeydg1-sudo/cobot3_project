#!/usr/bin/env python3
"""Generate the Nav2 occupancy map used by standalone_amr_world_nav2.py."""

from pathlib import Path


# Keep these values synchronized with standalone_amr_world_nav2.py.
WORLD_SIZE_X = 20.0
WORLD_SIZE_Y = 12.0
WALL_THICKNESS = 0.1
RESOLUTION = 0.05

FREE = 254
OCCUPIED = 0


def generate_map(output_directory: Path) -> None:
    width = round(WORLD_SIZE_X / RESOLUTION)
    height = round(WORLD_SIZE_Y / RESOLUTION)
    wall_pixels = max(1, round(WALL_THICKNESS / RESOLUTION))

    pixels = [[FREE] * width for _ in range(height)]

    for y in range(height):
        for x in range(width):
            if (
                x < wall_pixels
                or x >= width - wall_pixels
                or y < wall_pixels
                or y >= height - wall_pixels
            ):
                pixels[y][x] = OCCUPIED

    output_directory.mkdir(parents=True, exist_ok=True)
    pgm_path = output_directory / "factory_map.pgm"
    yaml_path = output_directory / "factory_map.yaml"

    # P5 is the conventional compact PGM representation used by Nav2 maps.
    with pgm_path.open("wb") as pgm_file:
        pgm_file.write(b"P5\n")
        pgm_file.write(b"# Generated from standalone_amr_world_nav2.py\n")
        pgm_file.write(f"{width} {height}\n255\n".encode("ascii"))
        for row in reversed(pixels):
            pgm_file.write(bytes(row))

    yaml_path.write_text(
        "image: factory_map.pgm\n"
        "mode: trinary\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{-WORLD_SIZE_X / 2}, {-WORLD_SIZE_Y / 2}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n",
        encoding="utf-8",
    )

    print(f"Generated {pgm_path} ({width}x{height}, {RESOLUTION} m/pixel)")
    print(f"Generated {yaml_path}")


if __name__ == "__main__":
    generate_map(Path(__file__).resolve().parents[1] / "maps")
