#!/usr/bin/env python3
"""Generate Markdown run report from pipeline outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd


def load_csv(path: Path) -> pd.DataFrame:
    return pd.read_csv(path) if path.exists() else pd.DataFrame()


def format_ci(ci):
    if not isinstance(ci, (list, tuple)) or len(ci) != 2:
        return "N/A"
    return f"[{ci[0]:.3f}, {ci[1]:.3f}]"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, help="results.json from gg_pipeline_cli")
    parser.add_argument("--steady", type=Path, help="steady_points.csv path")
    parser.add_argument("--envelope", type=Path, help="envelope.csv path")
    parser.add_argument("--trajectory", type=Path, help="trajectory.csv path")
    parser.add_argument("--output", type=Path, default=None, help="Output Markdown path (default: results.md next to JSON)")
    args = parser.parse_args()

    data = json.loads(args.results.read_text())
    steady_count = data.get("steady_point_count", 0)
    friction = data.get("validation", {})
    quality = data.get("quality", {})
    mc = data.get("monte_carlo", {})

    steady_df = load_csv(args.steady) if args.steady else pd.DataFrame()
    envelope_df = load_csv(args.envelope) if args.envelope else pd.DataFrame()
    trajectory_df = load_csv(args.trajectory) if args.trajectory else pd.DataFrame()

    lines = []
    lines.append(f"# Run Report\n")
    lines.append(f"- Steady-state samples: **{steady_count}**\n")
    lines.append(f"- Max |Gx|: **{data.get('max_gx', 0.0):.3f} g**, Max |Gy|: **{data.get('max_gy', 0.0):.3f} g**\n")
    lines.append(f"- Envelope area: **{data.get('envelope_area', 0.0):.3f} g²** (alpha={data.get('alpha', 0.0)})\n")

    lines.append("\n## Validation\n")
    lines.append(f"- R² (a_y vs v_x·ω_z): **{friction.get('r_squared', 0.0):.3f}**\n")
    if friction.get('friction_mu', 0.0):
        lines.append(f"- Friction μ: **{friction.get('friction_mu', 0.0):.2f}**, violation ratio: **{friction.get('friction_violation_ratio', 0.0):.3f}**, max magnitude: **{friction.get('friction_max_magnitude', 0.0):.3f} g**\n")
    if 'hausdorff_distance' in friction:
        lines.append(f"- Hausdorff distance to reference: **{friction['hausdorff_distance']:.3f} g**\n")

    lines.append("\n## Sampling Quality\n")
    lines.append(f"- Mean Δt: {quality.get('mean_dt', 0.0):.6f} s ({quality.get('nominal_frequency', 0.0):.2f} Hz)\n")
    lines.append(f"- Jitter σ: {quality.get('jitter', 0.0):.6f} s\n")
    lines.append(f"- Missing rate: {quality.get('missing_rate', 0.0):.4f}\n")
    lines.append(f"- Accel saturation rate: {quality.get('accel_saturation_rate', 0.0):.4f}, Gyro saturation rate: {quality.get('gyro_saturation_rate', 0.0):.4f}\n")

    lines.append("\n## Monte Carlo Summary\n")
    if mc.get('enabled'):
        lines.append(f"- Samples: {mc.get('samples', 0)} (seed {mc.get('seed', 'N/A')})\n")
        lines.append(f"- Mean max |Gx|: {mc.get('mean_gx', 0.0):.3f}, 95% CI: {format_ci(mc.get('gx_ci_95'))}\n")
        lines.append(f"- Mean max |Gy|: {mc.get('mean_gy', 0.0):.3f}, 95% CI: {format_ci(mc.get('gy_ci_95'))}\n")
        lines.append(f"- Mean envelope area: {mc.get('mean_area', 0.0):.3f}, 95% CI: {format_ci(mc.get('area_ci_95'))}\n")
    else:
        lines.append("- Monte Carlo disabled\n")

    if not trajectory_df.empty():
        lines.append("\n## VIO Trajectory (preview)\n")
        last = trajectory_df.tail(1).iloc[0]
        lines.append(f"- Final position: ({last['pos_x']:.2f}, {last['pos_y']:.2f}, {last['pos_z']:.2f}) m\n")
        lines.append(f"- Final speed: ({last['vel_x']:.2f}, {last['vel_y']:.2f}, {last['vel_z']:.2f}) m/s\n")

    if not steady_df.empty():
        lines.append("\n## Steady-State Extremes\n")
        top = steady_df.assign(mag=(steady_df['gx']**2 + steady_df['gy']**2)**0.5).nlargest(5, 'mag')
        lines.append(top.to_markdown(index=False))
        lines.append("\n")

    output_path = args.output if args.output else args.results.with_suffix(".md")
    output_path.write_text("\n".join(lines))
    print(f"Wrote report to {output_path}")


if __name__ == "__main__":
    main()
