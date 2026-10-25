#!/usr/bin/env python3
import csv
import html
import sys
from collections import defaultdict
from pathlib import Path


def render(csv_path, svg_path):
    cases = defaultdict(lambda: defaultdict(list))
    with Path(csv_path).open(newline="", encoding="utf-8") as stream:
        for record in csv.DictReader(stream):
            cases[record["case"]][record["kind"]].append(
                (float(record["x"]), float(record["y"]))
            )
    panels = []
    for panel_index, (name, groups) in enumerate(cases.items()):
        points = groups["input"] + groups["output"] + groups["obstacle"]
        min_x = min(point[0] for point in points) - 0.08
        min_y = min(point[1] for point in points) - 0.08
        span = max(
            max(point[0] for point in points) - min_x + 0.08,
            max(point[1] for point in points) - min_y + 0.08,
        )
        scale = 250 / span

        def project(point):
            return 35 + (point[0] - min_x) * scale, 310 - (point[1] - min_y) * scale

        panels.append(f'<g transform="translate({panel_index * 340},0)">')
        label = f'{name}: {len(groups["input"])} -&gt; {len(groups["output"])} points'
        panels.append(f'<text x="20" y="35">{html.escape(name)}: '
                      f'{len(groups["input"])} -&gt; {len(groups["output"])} points</text>')
        for obstacle in groups["obstacle"]:
            center_x, center_y = project(obstacle)
            size = 0.05 * scale
            panels.append(f'<rect x="{center_x - size / 2}" y="{center_y - size / 2}" '
                          f'width="{size}" height="{size}" fill="#ef7676"/>')
        for kind, color, dashed in (("input", "#888888", 'stroke-dasharray="5 4"'),
                                    ("output", "#1864ab", "")):
            coordinates = " ".join(f"{coord_x:.3f},{coord_y:.3f}"
                                   for coord_x, coord_y in map(project, groups[kind]))
            panels.append(f'<polyline points="{coordinates}" fill="none" '
                          f'stroke="{color}" stroke-width="2" {dashed}/>')
            for coord_x, coord_y in map(project, groups[kind]):
                panels.append(f'<circle cx="{coord_x}" cy="{coord_y}" r="3" fill="{color}"/>')
        panels.append('</g>')
        print(html.unescape(label))
    width = 340 * len(cases)
    Path(svg_path).write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="400" '
        f'viewBox="0 0 {width} 400"><rect width="100%" height="100%" fill="white"/>'
        '<g font-family="sans-serif" font-size="16">' + "".join(panels) +
        '<text x="20" y="365">Gray dashed: input; blue: simplified; red: occupied cell (0.05m).</text>'
        '<text x="20" y="390">Simplifier only: not final spline or vehicle safety validation.</text></g></svg>',
        encoding="utf-8",
    )
    print(f"EXPORTED {svg_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: render_t02_observations.py INPUT.csv OUTPUT.svg")
    render(sys.argv[1], sys.argv[2])
