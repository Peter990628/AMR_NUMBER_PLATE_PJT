#!/usr/bin/env python3
"""Plot CFV image predictions with camera-relative, top-view FRONT arrows.

Each arrow is a schematic direction, not a calibrated projection onto the
photograph.  The viewer/camera is below the car; away from the camera is up.
CFV view angle 0 means the car faces the camera, 180 means it faces away.
The plot does not command motion or supply a vehicle pose in a ROS frame.
"""

import argparse
import json
import math
from pathlib import Path
import textwrap


def front_direction(angle_deg):
    """Unit vehicle-front vector in a top view with camera-right x, away y.

    CFV's cardinal views map to: 0 -> down, 90 -> right, 180 -> up,
    270 -> left.  Matplotlib uses mathematical (up-positive) y coordinates.
    """
    angle = float(angle_deg)
    if not math.isfinite(angle):
        raise ValueError('CFV angle must be finite')
    radians = math.radians(angle % 360.0)
    return math.sin(radians), -math.cos(radians)


def circular_error(angle_deg, reference_deg):
    """Smallest unsigned separation of two finite circular angles."""
    front_direction(angle_deg)
    front_direction(reference_deg)
    return abs((float(angle_deg) - float(reference_deg) + 180.0) % 360.0 - 180.0)


def reference_angle(sample):
    """Keep measured geometry distinct from a provided dataset angle label."""
    if sample.get('geometric_reference_deg') is not None:
        return 'Geometric reference', float(sample['geometric_reference_deg'])
    if sample.get('cfv_label_deg') is not None:
        return 'Dataset label', float(sample['cfv_label_deg'])
    return None


def draw_direction_inset(ax, angle_deg, reference=None):
    """Draw an explicitly schematic camera-relative top view inside an image."""
    from matplotlib.patches import Circle, FancyArrowPatch

    # Keep the inset in the upper sky/background in the simulation frames,
    # rather than covering the vehicle rear/plate around the image centre.
    inset = ax.inset_axes([0.73, 0.64, 0.26, 0.35])
    inset.set_facecolor((0.035, 0.05, 0.075, 0.96))
    inset.set_xlim(-1.15, 1.15)
    inset.set_ylim(-1.55, 1.42)
    inset.set_aspect('equal')
    inset.set_xticks([])
    inset.set_yticks([])
    for spine in inset.spines.values():
        spine.set_color('#657487')
    inset.text(0, 1.23, 'TOP VIEW', ha='center', va='center', color='white',
               fontsize=9, weight='bold')
    inset.text(0, 1.04, '(camera-relative)', ha='center', va='center',
               color='#cbd5e1', fontsize=7)
    inset.add_patch(Circle((0, 0), 0.78, fill=False, color='#657487', lw=0.8))
    inset.axhline(0, xmin=0.12, xmax=0.88, color='#475569', lw=0.7)
    inset.plot([0, 0], [-0.8, 0.8], color='#475569', lw=0.7)
    inset.scatter([0], [0], color='white', s=13, zorder=4)
    inset.text(-0.99, 0, 'L', ha='center', va='center', color='#cbd5e1', fontsize=8)
    inset.text(0.99, 0, 'R', ha='center', va='center', color='#cbd5e1', fontsize=8)
    if reference is not None:
        _, ref_angle = reference
        x_ref, y_ref = front_direction(ref_angle)
        inset.add_patch(FancyArrowPatch(
            (0, 0), (0.78 * x_ref, 0.78 * y_ref), arrowstyle='-|>',
            mutation_scale=17, lw=1.8, linestyle='--', color='#67e8f9', zorder=3))
    x_front, y_front = front_direction(angle_deg)
    inset.add_patch(FancyArrowPatch(
        (0, 0), (0.68 * x_front, 0.68 * y_front), arrowstyle='-|>',
        mutation_scale=21, lw=2.7, color='#ff9d2e', zorder=4))
    inset.scatter([0], [-1.04], s=43, marker='^', color='#e2e8f0')
    inset.text(0, -1.30, 'CAMERA / VIEWER', ha='center', va='center',
               color='#e2e8f0', fontsize=7)
    return inset


def plot_report(report_path, output_path, ids=None, columns=3):
    """Create one figure without overwriting any existing artifact."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    report_path = Path(report_path)
    output_path = Path(output_path)
    if output_path.exists():
        raise FileExistsError(f'Refusing to overwrite {output_path}')
    if columns < 1:
        raise ValueError('columns must be positive')
    report = json.loads(report_path.read_text())
    samples = report['samples']
    if ids is not None:
        by_id = {sample['id']: sample for sample in samples}
        unknown = set(ids) - set(by_id)
        if unknown:
            raise ValueError(f'Unknown sample IDs: {sorted(unknown)}')
        samples = [by_id[sample_id] for sample_id in ids]
    if not samples:
        raise ValueError('No samples to plot')
    columns = min(columns, len(samples))
    rows = math.ceil(len(samples) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(7.1 * columns, 5.1 * rows),
                                squeeze=False, facecolor='#101720')
    has_reference = False
    for ax, sample in zip(axes.flat, samples):
        angle = float(sample['angle_deg'])
        front_direction(angle)
        image_path = Path(sample.get('image_absolute', sample['image']))
        if not image_path.is_absolute():
            image_path = Path.cwd() / image_path
        ax.imshow(plt.imread(image_path))
        ax.set_axis_off()
        ref = reference_angle(sample)
        if ref is not None:
            front_direction(ref[1])
            has_reference = True
        title = (f"{sample.get('display_name', sample['id'])}\n"
                 f"CFV view angle {angle % 360:.1f}\N{DEGREE SIGN}  |  "
                 f"rear-view deviation {circular_error(angle, 180):.1f}\N{DEGREE SIGN}")
        ax.set_title(title, fontsize=12, color='white', loc='left', pad=10)
        draw_direction_inset(ax, angle, ref)
        if ref is not None:
            text = (f'{ref[0]}: {ref[1] % 360:.1f}\N{DEGREE SIGN}  |  '
                    f'angular difference: {circular_error(angle, ref[1]):.1f}\N{DEGREE SIGN}')
            ax.text(0.015, 0.025, text, transform=ax.transAxes, fontsize=10,
                    color='#9df3ff', va='bottom',
                    bbox={'facecolor': '#101720', 'alpha': 0.88, 'edgecolor': 'none', 'pad': 5})
    for ax in list(axes.flat)[len(samples):]:
        ax.set_axis_off()
    figure.suptitle('CFV orientation: predicted vehicle FRONT direction', color='white',
                   fontsize=19, weight='bold', y=0.985)
    legend = [Line2D([0], [0], color='#ff9d2e', lw=3,
                     label='Orange arrow: predicted vehicle front')]
    if has_reference:
        legend.append(Line2D([0], [0], color='#67e8f9', lw=2, linestyle='--',
                             label='Cyan dashed arrow: geometric reference / dataset label'))
    figure.legend(handles=legend, loc='lower center', bbox_to_anchor=(0.5, 0.047),
                  ncol=1 if columns == 1 else len(legend), frameon=False,
                  labelcolor='white', fontsize=11)
    footer = ('Inset = schematic top view, camera at bottom; L/R = camera left/right. '
              'Arrows are not projected onto the photo and are not motion commands.')
    figure.text(0.5, 0.018, textwrap.fill(footer, width=85 * columns),
                ha='center', va='bottom', color='#cbd5e1', fontsize=9, wrap=True)
    figure.subplots_adjust(left=0.018, right=0.982, top=1.0 - 1.05 / (5.1 * rows),
                           bottom=0.18 if rows == 1 else 0.10,
                           hspace=0.22, wspace=0.055)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=150, facecolor=figure.get_facecolor())
    plt.close(figure)
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--output', type=Path,
                        help='New PNG path (default: next to report, comparison_arrows.png)')
    parser.add_argument('--ids', help='Comma-separated sample IDs, in desired display order')
    parser.add_argument('--columns', type=int, default=3)
    args = parser.parse_args()
    output = args.output or args.report.parent / 'comparison_arrows.png'
    if output.suffix.lower() != '.png':
        parser.error('--output must use the .png extension')
    ids = [item.strip() for item in args.ids.split(',') if item.strip()] if args.ids else None
    try:
        result = plot_report(args.report, output, ids, args.columns)
    except (ValueError, FileExistsError, FileNotFoundError) as error:
        parser.error(str(error))
    print(result)


if __name__ == '__main__':
    main()
