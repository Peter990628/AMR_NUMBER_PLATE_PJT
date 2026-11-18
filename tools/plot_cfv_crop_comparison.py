#!/usr/bin/env python3
"""Compare full-frame and cropped CFV inputs with separate direction diagrams.

Orange arrows show the model's predicted vehicle FRONT direction. Cyan dashed
arrows show the same scene's geometric reference. Diagrams are camera-relative
top views, with the camera at the bottom; they are not image projections.
"""

import argparse
import json
from pathlib import Path

from plot_cfv_orientation import circular_error, front_direction, reference_angle


DEFAULT_SOURCE_IDS = ('view_225', 'view_180', 'view_135', 'view_0')
VIEW_NAMES = {
    'view_225': 'Rear-left view',
    'view_180': 'Rear view',
    'view_135': 'Rear-right view',
    'view_0': 'Front view',
}


def prediction_angle(sample):
    """Read the probe's angle field, with an explicit alternate schema fallback."""
    value = sample.get('angle_deg', sample.get('predicted_angle_deg'))
    if value is None:
        raise ValueError(f"Missing prediction angle: {sample.get('id', '<unknown>')}")
    front_direction(value)  # Reject nonfinite values before plotting.
    return float(value)


def comparison_pairs(samples, primary_condition='crop_p10', source_ids=None):
    """Select exactly one baseline and primary crop per requested source image."""
    if primary_condition == 'full_frame':
        raise ValueError('Primary condition must differ from full_frame')
    source_ids = list(DEFAULT_SOURCE_IDS if source_ids is None else source_ids)
    if not source_ids or len(set(source_ids)) != len(source_ids):
        raise ValueError('Source IDs must be nonempty and unique')
    by_key = {}
    for sample in samples:
        key = (sample.get('crop_source_id'), sample.get('input_condition'))
        if key[0] not in source_ids or key[1] not in ('full_frame', primary_condition):
            continue
        if key in by_key:
            raise ValueError(f'Duplicate source / condition: {key}')
        by_key[key] = sample
    pairs = []
    for source_id in source_ids:
        pair = []
        for condition in ('full_frame', primary_condition):
            key = (source_id, condition)
            if key not in by_key:
                raise ValueError(f'Missing source / condition: {key}')
            sample = by_key[key]
            prediction_angle(sample)
            ref = reference_angle(sample)
            if ref is None or ref[0] != 'Geometric reference':
                raise ValueError(f'Missing geometric reference: {sample.get("id")}')
            front_direction(ref[1])
            pair.append(sample)
        if circular_error(reference_angle(pair[0])[1], reference_angle(pair[1])[1]) > 1e-4:
            raise ValueError(f'Baseline and crop references differ for {source_id}')
        pairs.append((source_id, pair))
    return pairs


def draw_direction_panel(ax, angle_deg, reference_deg):
    """Reserve an entire axis for arrows so no part of the car is obscured."""
    from matplotlib.patches import Circle, FancyArrowPatch

    ax.set_facecolor('#172330')
    ax.set_xlim(-1.15, 1.15)
    ax.set_ylim(-1.55, 1.42)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color('#536476')
    ax.text(0, 1.20, 'TOP VIEW', ha='center', color='white', fontsize=9, weight='bold')
    ax.text(0, 1.02, 'camera-relative', ha='center', color='#cbd5e1', fontsize=7)
    ax.add_patch(Circle((0, 0), .78, fill=False, color='#536476', lw=.8))
    ax.plot([-.8, .8], [0, 0], color='#475569', lw=.7)
    ax.plot([0, 0], [-.8, .8], color='#475569', lw=.7)
    for x, label in ((-.99, 'L'), (.99, 'R')):
        ax.text(x, 0, label, ha='center', va='center', color='#cbd5e1', fontsize=8)
    for value, length, color, width, style, scale in (
            (reference_deg, .80, '#67e8f9', 2, '--', 20),
            (angle_deg, .67, '#ff9d2e', 3, '-', 24)):
        x, y = front_direction(value)
        ax.add_patch(FancyArrowPatch(
            (0, 0), (length * x, length * y), arrowstyle='-|>',
            mutation_scale=scale, lw=width, linestyle=style, color=color))
    ax.scatter([0], [0], color='white', s=14, zorder=5)
    ax.scatter([0], [-1.04], marker='^', s=45, color='#e2e8f0')
    ax.text(0, -1.30, 'CAMERA / VIEWER', ha='center', color='#e2e8f0', fontsize=7)


def condition_label(condition):
    if condition == 'full_frame':
        return 'Full-frame input'
    if condition.startswith('crop_p') and condition[6:].isdigit():
        return f'Vehicle crop + {int(condition[6:])}% padding per side'
    return condition


def plot_comparison(report_path, output_path, primary_condition='crop_p10', source_ids=None):
    """Create a new PNG with one full-frame / crop comparison row per view."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    report_path, output_path = Path(report_path), Path(output_path)
    if output_path.exists():
        raise FileExistsError(f'Refusing to overwrite {output_path}')
    report = json.loads(report_path.read_text())
    pairs = comparison_pairs(report['samples'], primary_condition, source_ids)
    rows = len(pairs)
    figure = plt.figure(figsize=(18, 3.55 * rows + 1.7), facecolor='#101720')
    grid = figure.add_gridspec(rows, 2, left=.025, right=.975,
                              top=1 - .9 / (3.55 * rows + 1.7),
                              bottom=.95 / (3.55 * rows + 1.7),
                              wspace=.09, hspace=.28)
    try:
        for row, (source_id, pair) in enumerate(pairs):
            for column, sample in enumerate(pair):
                inner = grid[row, column].subgridspec(1, 2, width_ratios=(3.7, 1.2), wspace=.04)
                photo_ax = figure.add_subplot(inner[0, 0])
                arrow_ax = figure.add_subplot(inner[0, 1])
                image_path = Path(sample.get('image_absolute') or sample['image'])
                if not image_path.is_absolute():
                    image_path = Path.cwd() / image_path
                pixels = plt.imread(image_path)
                photo_ax.imshow(pixels)
                photo_ax.set_axis_off()
                angle = prediction_angle(sample)
                ref = reference_angle(sample)[1]
                diff = circular_error(angle, ref)
                draw_direction_panel(arrow_ax, angle, ref)
                position = grid[row, column].get_position(figure)
                title = (f"{VIEW_NAMES.get(source_id, source_id)}  |  "
                         f"{condition_label(sample['input_condition'])}\n"
                         f"Prediction {angle % 360:.1f}\N{DEGREE SIGN}   "
                         f"Reference {ref % 360:.1f}\N{DEGREE SIGN}   "
                         f"Difference {diff:.1f}\N{DEGREE SIGN}")
                figure.text(position.x0, position.y1 + .007, title,
                            ha='left', va='bottom', fontsize=10, color='white', linespacing=1.5)
                photo_ax.text(.5, -.035, f'Supplied image: {pixels.shape[1]} x {pixels.shape[0]} px',
                              transform=photo_ax.transAxes, ha='center', va='top',
                              color='#cbd5e1', fontsize=8)
        figure.suptitle('CFV: full frame versus vehicle crop', y=.987,
                       fontsize=20, color='white', weight='bold')
        legend = [
            Line2D([0], [0], color='#ff9d2e', lw=3, label='Predicted vehicle FRONT'),
            Line2D([0], [0], color='#67e8f9', lw=2, linestyle='--', label='Geometric reference'),
        ]
        figure.legend(handles=legend, loc='lower center',
                      bbox_to_anchor=(.5, .40 / (3.55 * rows + 1.7)), ncol=2,
                      frameon=False, labelcolor='white', fontsize=11)
        figure.text(.5, .15 / (3.55 * rows + 1.7),
                    'Same source photos and model. Manual vehicle boxes. '
                    'Arrows are schematic top views, not image projections or motion commands.',
                    ha='center', color='#cbd5e1', fontsize=9)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output_path, dpi=150, facecolor=figure.get_facecolor())
    finally:
        plt.close(figure)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--primary-condition', default='crop_p10')
    parser.add_argument('--source-ids', help='Optional comma-separated source IDs, in display order')
    args = parser.parse_args()
    if args.output.suffix.lower() != '.png':
        parser.error('--output must use the .png extension')
    source_ids = None if args.source_ids is None else [s.strip() for s in args.source_ids.split(',')]
    try:
        result = plot_comparison(args.report, args.output, args.primary_condition, source_ids)
    except (ValueError, KeyError, FileExistsError, FileNotFoundError) as error:
        parser.error(str(error))
    print(result)


if __name__ == '__main__':
    main()
