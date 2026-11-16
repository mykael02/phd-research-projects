"""Small, standard-library-only report helpers shared by the two starters."""

import csv
import html
import json
from pathlib import Path


def write_results(output, records):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    (output / "results.json").write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    summaries = [{k: v for k, v in row.items() if k not in ("path", "inspection_order")} for row in records]
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)


def draw_grid(output, grid, path, labels, title, radiation=None):
    """Render ground truth for evaluation only; never fed into a policy."""
    size, top = 32, 54
    width, height = max(640, grid.width * size), grid.height * size + top + 28
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
             '<rect width="100%" height="100%" fill="#f8fafc"/>',
             f'<text x="12" y="22" font-family="sans-serif" font-size="16">{html.escape(title)}</text>',
             '<text x="12" y="42" font-family="sans-serif" font-size="12">Ground-truth overview; blue = route, dark = wall, orange = higher dose rate</text>']
    peak = max(radiation.values(), default=1) if radiation else 1
    for r in range(grid.height):
        for c in range(grid.width):
            color = "#1e293b" if not grid.is_free((r, c)) else "#ffffff"
            if radiation and grid.is_free((r, c)):
                fraction = radiation[(r, c)] / max(peak, 1e-12)
                color = f'rgb(255,{int(255-130*fraction)},{int(255-210*fraction)})'
            parts.append(f'<rect x="{c*size}" y="{r*size+top}" width="{size}" height="{size}" fill="{color}" stroke="#cbd5e1"/>')
    points = " ".join(f"{(c+.5)*size},{(r+.5)*size+top}" for r, c in path)
    parts.append(f'<polyline points="{points}" fill="none" stroke="#2563eb" stroke-width="3" opacity="0.8"/>')
    for (r, c), label in labels.items():
        parts.append(f'<text x="{(c+.5)*size}" y="{(r+.5)*size+top+5}" text-anchor="middle" font-family="sans-serif" font-weight="bold" fill="#111827">{html.escape(label)}</text>')
    parts.append('</svg>')
    Path(output).write_text("\n".join(parts), encoding="utf-8")
