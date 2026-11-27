"""Export the firmware's randomize coverage distribution as CSV, PNG and SVG.

Run: python Gravity_Sand/tools/plot-fluid-variation.py
Requires matplotlib, numpy, and a native g++ compiler. A temporary C++ exporter
calls FluidVariation.h directly; Python does not duplicate the probability law.
"""
from __future__ import annotations

import argparse
import csv
import io
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter
import numpy as np


def export_header_data(compiler: str, sketch: Path):
    source = r'''
#include <stdio.h>
#include "@HEADER@"
int main() {
  namespace cfg = sand::config;
  namespace v = sand::variation;
  printf("M,%.9g,%.9g,%.9g,%.9g\n",
         cfg::kMinSpawnCoverage, cfg::kMaxSpawnCoverage,
         cfg::kMinVisualVariation, cfg::kMaxVisualVariation);
  for (int i = 0; i < 10000; ++i) {
    const float unit = (i + 0.5f) / 10000.0f;
    printf("Q,%.9g,%.9g,%.9g\n", unit, v::coverageFromUnit(unit), v::scaleFromUnit(unit));
  }
}
'''.replace("@HEADER@", (sketch / "FluidVariation.h").as_posix())
    environment = os.environ.copy()
    environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="simcircle-variation-") as temporary:
        directory = Path(temporary)
        source_path = directory / "export.cpp"
        executable = directory / ("export.exe" if os.name == "nt" else "export")
        source_path.write_text(source, encoding="utf-8")
        subprocess.run([compiler, "-std=c++11", "-O2", str(source_path), "-o", str(executable)],
                       check=True, env=environment)
        output = subprocess.run([str(executable)], check=True, capture_output=True,
                                text=True, env=environment).stdout
    metadata, quantiles = None, []
    for row in csv.reader(io.StringIO(output)):
        if row[0] == "M":
            metadata = [float(value) for value in row[1:]]
        elif row[0] == "Q":
            quantiles.append([float(value) for value in row[1:]])
    assert metadata is not None and len(quantiles) == 10000
    return metadata, np.asarray(quantiles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", default=shutil.which("g++"))
    arguments = parser.parse_args()
    compiler = arguments.compiler
    if not compiler and Path("C:/msys64/ucrt64/bin/g++.exe").is_file():
        compiler = "C:/msys64/ucrt64/bin/g++.exe"
    if not compiler:
        parser.error("g++ was not found; pass --compiler with its path")
    sketch = Path(__file__).resolve().parents[1]
    output_directory = sketch / "assets"
    output_directory.mkdir(exist_ok=True)
    metadata, quantiles = export_header_data(compiler, sketch)
    fill_low, fill_high, size_low, size_high = metadata
    units, coverage, preferred_sizes = quantiles.T
    with (output_directory / "fluid-size-count.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["quantile_unit", "target_fill_percent",
                         "preferred_size_percent_separate_draw"])
        for unit, fill, size in zip(units, coverage, preferred_sizes):
            writer.writerow([f"{unit:.6f}", f"{fill * 100:.5f}", f"{size * 100:.5f}"])

    navy, blue, aqua, muted = "#142d4a", "#087bb8", "#2cabb8", "#567084"
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "axes.labelcolor": navy, "text.color": navy,
                         "xtick.color": muted, "ytick.color": muted,
                         "svg.fonttype": "none"})
    figure = plt.figure(figsize=(13.6, 7.4), facecolor="#f7fbfd")
    grid = figure.add_gridspec(1, 2, width_ratios=[1.5, 1], left=.075, right=.965,
                             bottom=.23, top=.75, wspace=.31)
    curve = figure.add_subplot(grid[0, 0])
    bars = figure.add_subplot(grid[0, 1])
    figure.text(.075, .925, "More room to move", fontsize=24, fontweight="bold")
    figure.text(.075, .866,
                "Randomize aims for 15-35% fill, favoring the lower end. No crowded bonus rolls.",
                fontsize=13, color=muted)
    figure.text(.075, .814,
                "Coverage counts overlap once and clips shapes to the round screen. "
                "Size and count are approximately fitted only when randomizing.", fontsize=11, color=muted)
    for axis in (curve, bars):
        axis.set_facecolor("#f7fbfd")
        for spine in axis.spines.values():
            spine.set_visible(False)
        axis.grid(axis="y" if axis is curve else "x", color="#d8e4eb", linewidth=.8)
        axis.set_axisbelow(True)
    curve.axhspan(15, 35, color="#dceff1", zorder=0)
    curve.plot(units * 100, coverage * 100, color=blue, linewidth=2.8)
    curve.text(5, 32, "More open layouts are more likely", color=blue, fontsize=11)
    curve.set_xlim(0, 100)
    curve.set_ylim(12, 38)
    curve.set_xticks([0, 20, 40, 60, 80, 100])
    curve.set_yticks([15, 20, 25, 30, 35])
    curve.xaxis.set_major_formatter(PercentFormatter(xmax=100))
    curve.yaxis.set_major_formatter(PercentFormatter(xmax=100))
    curve.set_xlabel("Uniform random roll (percentile)", labelpad=10)
    curve.set_ylabel("Incoming visible fill target", labelpad=10)

    edges = np.array([.15, .20, .25, .30, .35])
    bin_counts, _ = np.histogram(coverage, bins=edges)
    chances = bin_counts / len(coverage) * 100
    positions = np.arange(4)
    bars.barh(positions, chances, height=.56, color=[blue, aqua, "#79c2ce", "#b6dce2"])
    bars.invert_yaxis()
    bars.set_yticks(positions, ["15-20% fill", "20-25% fill", "25-30% fill", "30-35% fill"])
    bars.set_xlim(0, max(chances) * 1.2)
    bars.set_xticks([0, 20, 40, 60])
    bars.xaxis.set_major_formatter(PercentFormatter(xmax=100))
    bars.tick_params(axis="y", length=0, pad=9)
    bars.set_title("Chance per press", loc="left", fontsize=13, fontweight="bold", pad=16)
    for position, chance in zip(positions, chances):
        bars.text(chance + 1, position, f"{chance:.1f}%", va="center", fontweight="bold", fontsize=11)

    figure.text(.075, .13,
                f"Target = {fill_low * 100:.0f} + {(fill_high - fill_low) * 100:.0f} x u squared percent. "
                f"Mean target: {np.mean(coverage) * 100:.1f}%.", fontsize=12, color=navy)
    figure.text(.075, .078,
                f"Targets: {fill_low * 100:.0f}-{fill_high * 100:.0f}% (margin for approximate fitting and antialiasing). "
                f"Final size limits: {size_low * 100:.0f}-{size_high * 100:.0f}%.", fontsize=10, color=muted)
    figure.text(.075, .039,
                "Preferred sizes have equal odds; the fit adjusts them to preserve the selected coverage and all five colors.",
                fontsize=10, color=muted)
    figure.savefig(output_directory / "fluid-size-count.png", dpi=200, facecolor=figure.get_facecolor())
    figure.savefig(output_directory / "fluid-size-count.svg", facecolor=figure.get_facecolor())
    plt.close(figure)
    print(f"Wrote header-derived PNG, SVG, and CSV to {output_directory}")
    print(f"Fill chances {chances}; mean target {np.mean(coverage) * 100:.4f}%")


if __name__ == "__main__":
    main()
