#!/usr/bin/env python3
"""Generate SVG research figures and Markdown summary tables from experiment CSVs."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
import statistics


METHOD_LABELS = {
    "static_astar": "static A*",
    "reactive_replanning": "reactive replanning",
    "expansion_temporal": "expansion temporal",
    "sampled_temporal_online": "sampled online",
    "sampled_temporal_bin_cache": "sampled bin cache",
    "sampled_temporal_actea": "sampled ACTEA",
}


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key)
    if value in (None, "", "None"):
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _bool(row: dict[str, str], key: str) -> bool:
    return str(row.get(key, "")).lower() == "true"


def _label(method: str) -> str:
    return METHOD_LABELS.get(method, method)


def _bar_svg(title: str, values: dict[str, float], path: Path, *, ylabel: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width = 1040
    row_height = 42
    margin_left = 245
    margin_right = 90
    margin_top = 70
    margin_bottom = 50
    height = margin_top + margin_bottom + max(len(values), 1) * row_height
    chart_width = width - margin_left - margin_right
    max_value = max(values.values(), default=1.0) or 1.0
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20">{title}</text>',
        f'<text x="{margin_left}" y="54" text-anchor="start" font-family="Arial" font-size="13">{ylabel}</text>',
        f'<line x1="{margin_left}" y1="{margin_top - 12}" x2="{margin_left}" y2="{height - margin_bottom + 8}" stroke="#333"/>',
    ]
    for index, (label, value) in enumerate(values.items()):
        y = margin_top + index * row_height
        bar_w = (value / max_value) * chart_width
        parts.append(f'<text x="{margin_left - 12}" y="{y + 22}" text-anchor="end" font-family="Arial" font-size="13">{label}</text>')
        parts.append(f'<rect x="{margin_left}" y="{y + 5}" width="{bar_w:.1f}" height="25" fill="#2563eb"/>')
        parts.append(f'<text x="{margin_left + bar_w + 8:.1f}" y="{y + 23}" text-anchor="start" font-family="Arial" font-size="12">{value:.3g}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _grouped_bar_svg(title: str, groups: dict[str, dict[str, float]], path: Path, *, ylabel: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    methods = sorted({method for values in groups.values() for method in values})
    group_names = list(groups)
    width = 1120
    height = 520
    margin_left = 80
    margin_right = 40
    margin_top = 70
    margin_bottom = 130
    chart_w = width - margin_left - margin_right
    chart_h = height - margin_top - margin_bottom
    max_value = max((value for values in groups.values() for value in values.values()), default=1.0) or 1.0
    colors = ["#2563eb", "#16a34a", "#f97316", "#7c3aed", "#64748b"]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20">{title}</text>',
        f'<text x="24" y="{margin_top + chart_h/2}" transform="rotate(-90 24 {margin_top + chart_h/2})" text-anchor="middle" font-family="Arial" font-size="13">{ylabel}</text>',
        f'<line x1="{margin_left}" y1="{margin_top + chart_h}" x2="{width - margin_right}" y2="{margin_top + chart_h}" stroke="#333"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + chart_h}" stroke="#333"/>',
    ]
    group_w = chart_w / max(len(group_names), 1)
    bar_w = group_w / max(len(methods) + 1, 1)
    for gi, group in enumerate(group_names):
        x0 = margin_left + gi * group_w
        parts.append(f'<text x="{x0 + group_w/2:.1f}" y="{height - 82}" text-anchor="middle" font-family="Arial" font-size="12">{group}</text>')
        for mi, method in enumerate(methods):
            value = groups[group].get(method, 0.0)
            h = (value / max_value) * chart_h
            x = x0 + (mi + 0.5) * bar_w
            y = margin_top + chart_h - h
            parts.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w * 0.72:.1f}" height="{h:.1f}" fill="{colors[mi % len(colors)]}"/>')
    legend_x = margin_left
    for mi, method in enumerate(methods):
        y = height - 56 + (mi // 3) * 20
        x = legend_x + (mi % 3) * 300
        parts.append(f'<rect x="{x}" y="{y - 10}" width="12" height="12" fill="{colors[mi % len(colors)]}"/>')
        parts.append(f'<text x="{x + 18}" y="{y}" font-family="Arial" font-size="12">{_label(method)}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _line_svg(title: str, series: dict[str, list[tuple[float, float]]], path: Path, *, xlabel: str, ylabel: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    width = 980
    height = 540
    margin_left = 80
    margin_right = 50
    margin_top = 70
    margin_bottom = 100
    chart_w = width - margin_left - margin_right
    chart_h = height - margin_top - margin_bottom
    xs = [x for points in series.values() for x, _ in points]
    ys = [y for points in series.values() for _, y in points]
    min_x, max_x = min(xs, default=0.0), max(xs, default=1.0)
    min_y, max_y = 0.0, max(ys, default=1.0) or 1.0
    colors = ["#2563eb", "#16a34a", "#f97316", "#7c3aed"]

    def project(x: float, y: float) -> tuple[float, float]:
        px = margin_left + ((x - min_x) / max(max_x - min_x, 1e-9)) * chart_w
        py = margin_top + chart_h - ((y - min_y) / max(max_y - min_y, 1e-9)) * chart_h
        return px, py

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="Arial" font-size="20">{title}</text>',
        f'<line x1="{margin_left}" y1="{margin_top + chart_h}" x2="{width - margin_right}" y2="{margin_top + chart_h}" stroke="#333"/>',
        f'<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + chart_h}" stroke="#333"/>',
        f'<text x="{width/2}" y="{height - 34}" text-anchor="middle" font-family="Arial" font-size="13">{xlabel}</text>',
        f'<text x="24" y="{margin_top + chart_h/2}" transform="rotate(-90 24 {margin_top + chart_h/2})" text-anchor="middle" font-family="Arial" font-size="13">{ylabel}</text>',
    ]
    for index, (method, points) in enumerate(series.items()):
        color = colors[index % len(colors)]
        projected = [project(x, y) for x, y in sorted(points)]
        data = " ".join(f"{x:.1f},{y:.1f}" for x, y in projected)
        parts.append(f'<polyline points="{data}" fill="none" stroke="{color}" stroke-width="3"/>')
        for x, y in projected:
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}"/>')
        legend_y = height - 70 + index * 16
        parts.append(f'<rect x="{margin_left + 620}" y="{legend_y - 10}" width="12" height="12" fill="{color}"/>')
        parts.append(f'<text x="{margin_left + 638}" y="{legend_y}" font-family="Arial" font-size="12">{_label(method)}</text>')
    parts.append("</svg>")
    path.write_text("\n".join(parts) + "\n", encoding="utf-8")


def _summarize_by_method(rows: list[dict[str, str]]) -> dict[str, dict[str, float]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        method = row.get("method_name", "").strip()
        if method:
            grouped[method].append(row)
    summary: dict[str, dict[str, float]] = {}
    for method, items in grouped.items():
        summary[method] = {
            "success_rate": sum(1 for row in items if _bool(row, "success")) / max(len(items), 1),
            "avg_query_time_sec": statistics.mean(_float(row, "query_time_sec") for row in items),
            "avg_total_time_sec": statistics.mean(_float(row, "total_runtime", _float(row, "build_time_sec") + _float(row, "annotation_time_sec") + _float(row, "query_time_sec")) for row in items),
        }
    return summary


def _write_markdown_table(path: Path, title: str, rows: list[list[str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text(f"# {title}\n\nNo rows.\n", encoding="utf-8")
        return
    header = rows[0]
    lines = [f"# {title}", "", "| " + " | ".join(header) + " |", "| " + " | ".join("---" for _ in header) + " |"]
    for row in rows[1:]:
        lines.append("| " + " | ".join(row) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate ACTEA experiment charts and tables.")
    parser.add_argument("--experiments-dir", type=Path, default=Path("outputs/experiments"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    figures = args.experiments_dir / "figures"
    tables = args.experiments_dir / "tables"

    correctness = _read_csv(args.experiments_dir / "actea_correctness" / "summary.csv")
    mismatch_values = {"false blocked": 0.0, "false free": 0.0}
    if correctness:
        mismatch_values["false blocked"] = _float(correctness[0], "false_blocked")
        mismatch_values["false free"] = _float(correctness[0], "false_free")
    _bar_svg("Figure 1: ACTEA Correctness Mismatch Counts", mismatch_values, figures / "figure1_actea_correctness.svg", ylabel="count")

    repeated = _read_csv(args.experiments_dir / "repeated_query" / "results.csv")
    repeated_series: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for row in repeated:
        repeated_series[row.get("method_name", "")].append((_float(row, "query_count"), _float(row, "total_runtime")))
    _line_svg("Figure 2: Repeated-Query Scaling", dict(repeated_series), figures / "figure2_repeated_query_scaling.svg", xlabel="number of queries", ylabel="total wall-clock time (s)")

    hard = _read_csv(args.experiments_dir / "hard_scenes" / "results.csv")
    success_by_scene: dict[str, dict[str, float]] = {}
    time_by_scene: dict[str, dict[str, float]] = {}
    for scene in sorted({row.get("scene_family", "") for row in hard if row.get("scene_family")}):
        scene_rows = [row for row in hard if row.get("scene_family") == scene]
        by_method: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in scene_rows:
            by_method[row.get("method_name", "")].append(row)
        success_by_scene[scene] = {method: sum(1 for row in rows if _bool(row, "success")) / max(len(rows), 1) for method, rows in by_method.items()}
        time_by_scene[scene] = {method: statistics.mean(_float(row, "query_time_sec") for row in rows) for method, rows in by_method.items()}
    _grouped_bar_svg("Figure 3: Success Rate by Hard Scene", success_by_scene, figures / "figure3_success_by_scene.svg", ylabel="success rate")
    _grouped_bar_svg("Figure 4: Mean Planning Time by Hard Scene", time_by_scene, figures / "figure4_time_by_scene.svg", ylabel="seconds")

    ablation = _read_csv(args.experiments_dir / "roadmap_scale_ablation" / "results.csv")
    ablation_groups: dict[str, dict[str, float]] = {}
    for scale in sorted({row.get("roadmap_scale", "") for row in ablation if row.get("roadmap_scale")}):
        scale_rows = [row for row in ablation if row.get("roadmap_scale") == scale]
        by_method: dict[str, list[dict[str, str]]] = defaultdict(list)
        for row in scale_rows:
            by_method[row.get("method_name", "")].append(row)
        ablation_groups[scale] = {method: statistics.mean(_float(row, "total_runtime") for row in rows) for method, rows in by_method.items()}
    _grouped_bar_svg("Figure 5: Roadmap-Scale Ablation", ablation_groups, figures / "figure5_roadmap_scale_ablation.svg", ylabel="total runtime (s)")

    heuristic = _read_csv(args.experiments_dir / "heuristic_ablation" / "results.csv")
    heuristic_groups: dict[str, dict[str, float]] = {}
    for heuristic_mode in sorted({row.get("heuristic_mode", "") for row in heuristic if row.get("heuristic_mode")}):
        mode_rows = [row for row in heuristic if row.get("heuristic_mode") == heuristic_mode]
        heuristic_groups[heuristic_mode] = {
            row.get("method_name", ""): _float(row, "expanded_labels_total")
            for row in mode_rows
        }
    _grouped_bar_svg("Figure 6: Heuristic Ablation", heuristic_groups, figures / "figure6_heuristic_ablation.svg", ylabel="expanded labels")

    all_rows: list[dict[str, str]] = []
    for csv_path in args.experiments_dir.glob("*/results.csv"):
        all_rows.extend(_read_csv(csv_path))
    summary = _summarize_by_method(all_rows)
    _bar_svg("Appendix: Success Rate by Method", {_label(method): item["success_rate"] for method, item in summary.items()}, figures / "appendix_success_rate_by_method.svg", ylabel="success rate")
    _bar_svg("Appendix: Average Query Time by Method", {_label(method): item["avg_query_time_sec"] for method, item in summary.items()}, figures / "appendix_query_time_by_method.svg", ylabel="seconds")

    method_rows = [["method", "reusable roadmap?", "nonholonomic?", "time-aware?", "ACTEA intervals?", "repeated-query optimization?"]]
    method_rows.extend(
        [
            ["static A*", "no", "yes", "no", "no", "no"],
            ["reactive replanning", "no", "yes", "reactive", "no", "no"],
            ["expansion temporal", "limited", "yes", "yes", "no", "no"],
            ["sampled temporal online", "yes", "yes", "yes", "no", "no"],
            ["sampled temporal ACTEA", "yes", "yes", "yes", "yes", "yes"],
        ]
    )
    _write_markdown_table(tables / "table1_method_summary.md", "Table 1 - Method Summary", method_rows)

    repeated_rows = [["method", "offline build time", "offline annotation time", "avg query time", "total time over max N", "success rate", "interval hits", "cache hits"]]
    for method in sorted({row.get("method_name", "") for row in repeated}):
        rows = [row for row in repeated if row.get("method_name") == method]
        if not rows:
            continue
        max_row = max(rows, key=lambda row: _float(row, "query_count"))
        repeated_rows.append(
            [
                _label(method),
                f"{_float(max_row, 'build_time_sec'):.4f}",
                f"{_float(max_row, 'annotation_time_sec'):.4f}",
                f"{_float(max_row, 'cumulative_query_time_sec') / max(_float(max_row, 'query_count'), 1.0):.4f}",
                f"{_float(max_row, 'total_runtime'):.4f}",
                f"{_float(max_row, 'success_rate'):.2f}",
                str(int(_float(max_row, "interval_hits"))),
                str(int(_float(max_row, "cache_hits"))),
            ]
        )
    _write_markdown_table(tables / "table2_repeated_query_runtime.md", "Table 2 - Repeated-Query Runtime", repeated_rows)

    density_rows = [["method", "hard-scene success", "avg traversal time", "avg replans", "avg dynamic rejects"]]
    hard_summary = _summarize_by_method(hard)
    for method, item in sorted(hard_summary.items()):
        rows = [row for row in hard if row.get("method_name") == method]
        density_rows.append(
            [
                _label(method),
                f"{item['success_rate']:.2f}",
                f"{statistics.mean(_float(row, 'traversal_time') for row in rows):.3f}",
                f"{statistics.mean(_float(row, 'replans') for row in rows):.3f}",
                f"{statistics.mean(_float(row, 'rejected_dynamic_edges') for row in rows):.3f}",
            ]
        )
    _write_markdown_table(tables / "table3_hard_dynamic_scenes.md", "Table 3 - Hard Dynamic Scenes", density_rows)

    roadmap_rows = [["scale", "method", "nodes", "edges", "build", "annotation", "avg query", "total runtime", "success rate", "expanded labels"]]
    for row in ablation:
        roadmap_rows.append(
            [
                row.get("roadmap_scale", ""),
                _label(row.get("method_name", "")),
                row.get("roadmap_nodes", ""),
                row.get("roadmap_edges", ""),
                f"{_float(row, 'build_time_sec'):.3f}",
                f"{_float(row, 'annotation_time_sec'):.3f}",
                f"{_float(row, 'avg_query_time_sec'):.3f}",
                f"{_float(row, 'total_runtime'):.3f}",
                f"{_float(row, 'success_rate'):.2f}",
                str(int(_float(row, "expanded_labels_total"))),
            ]
        )
    _write_markdown_table(tables / "table5_roadmap_scale_ablation.md", "Table 5 - Roadmap-Scale Ablation", roadmap_rows)

    heuristic_rows = [["heuristic", "method", "avg query", "total runtime", "success rate", "expanded labels", "dynamic rejects"]]
    for row in heuristic:
        heuristic_rows.append(
            [
                row.get("heuristic_mode", ""),
                _label(row.get("method_name", "")),
                f"{_float(row, 'avg_query_time_sec'):.4f}",
                f"{_float(row, 'total_runtime'):.4f}",
                f"{_float(row, 'success_rate'):.2f}",
                str(int(_float(row, "expanded_labels_total"))),
                str(int(_float(row, "rejected_dynamic_edges_total"))),
            ]
        )
    _write_markdown_table(tables / "table6_heuristic_ablation.md", "Table 6 - Heuristic Ablation", heuristic_rows)

    consistency_rows = [["online exact validation", "ACTEA interval lookup", "agreement rate", "false blocked", "false free"]]
    if correctness:
        row = correctness[0]
        consistency_rows.append(["brute-force online", "ACTEA", row.get("agreement_rate", ""), row.get("false_blocked", ""), row.get("false_free", "")])
    _write_markdown_table(tables / "table4_actea_correctness.md", "Table 4 - ACTEA Correctness", consistency_rows)

    print(f"Wrote figures to {figures}")
    print(f"Wrote tables to {tables}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
