"""Experimental stateless endpoint support; not used by any ROS node.

Keep the fitted line/inliers unchanged. An endpoint must itself be an inlier
and have several nearby inliers among the selected cluster's available points.
Local spacing uses ALL available cluster points, never just sparse tail inliers.
This estimates a supported *observed* extent, not the true ends of a vehicle.
"""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class SupportConfig:
    half_window_m: float = 0.05
    max_half_window_m: float = 0.15
    spacing_multiplier: float = 3.0
    support_fraction: float = 0.6
    min_support_points: int = 3
    adaptive: bool = True
    min_length_m: float = 0.5

    def __post_init__(self):
        for name in ('half_window_m', 'max_half_window_m', 'spacing_multiplier', 'min_length_m'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if self.max_half_window_m < self.half_window_m:
            raise ValueError('max_half_window_m must be >= half_window_m')
        if not math.isfinite(self.support_fraction) or not 0 < self.support_fraction <= 1:
            raise ValueError('support_fraction must be in (0,1]')
        if type(self.min_support_points) is not int or self.min_support_points < 2:
            raise ValueError('min_support_points must be an integer >= 2')
        if type(self.adaptive) is not bool:
            raise ValueError('adaptive must be boolean')


def supported_extent(record, config=SupportConfig()):
    """Return new endpoint geometry and diagnostics, without changing record.

    record is the JSON form of the production C++ replay: points [beam,x,y],
    clusters of point indices and an estimate with center/tangent/inlier_indices.
    Unsupported/short extents return valid=False with no fallback geometry.
    Each distinct available original beam counts once (duplicates are rejected).
    """
    estimate = record['estimate']
    if estimate is None:
        return {'valid': False, 'reason': 'invalid_line'}
    center = estimate['center']
    tangent = estimate['tangent']
    if (len(center) != 2 or len(tangent) != 2 or
            not all(math.isfinite(v) for v in (*center, *tangent)) or
            abs(math.hypot(*tangent)-1) > 1e-6):
        raise ValueError('finite center and unit tangent required')
    cluster_index = estimate['cluster_index']
    if type(cluster_index) is not int or not 0 <= cluster_index < len(record['clusters']):
        raise ValueError('invalid selected cluster index')
    cluster = record['clusters'][cluster_index]
    if len(cluster) != len(set(cluster)):
        raise ValueError('duplicate cluster point index')
    inlier_indices = estimate['inlier_indices']
    if any(type(i) is not int or not 0 <= i < len(record['points']) for i in inlier_indices):
        raise ValueError('invalid inlier point index')
    inliers = set(inlier_indices)
    if len(inliers) != len(inlier_indices) or not inliers <= set(cluster):
        raise ValueError('inliers must be unique selected-cluster indices')
    samples = []
    seen_beams = set()
    for index in cluster:
        if type(index) is not int or not 0 <= index < len(record['points']):
            raise ValueError('invalid cluster point index')
        beam, x, y = record['points'][index]
        if type(beam) is not int or beam < 0 or beam in seen_beams:
            raise ValueError('original beam indices must be unique nonnegative integers')
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('point coordinates must be finite')
        seen_beams.add(beam)
        projection = (x-center[0])*tangent[0] + (y-center[1])*tangent[1]
        if not math.isfinite(projection):
            raise ValueError('nonfinite projection')
        samples.append((projection, index, beam, index in inliers))
    samples.sort()
    projections = [s[0] for s in samples]
    cumulative = [0]
    for sample in samples:
        cumulative.append(cumulative[-1]+sample[3])
    supported = []
    for j, (projection, index, beam, inside) in enumerate(samples):
        if not inside:
            continue
        # Four available neighbors on each side; zero projection gaps carry
        # no spacing information. The metric cap prevents bridging a large gap.
        near = projections[max(0,j-4):min(len(samples),j+5)]
        gaps = [b-a for a,b in zip(near,near[1:]) if b-a > 1e-9]
        spacing = statistics.median(gaps) if gaps else 0.0
        radius = config.half_window_m
        if config.adaptive:
            radius = min(config.max_half_window_m,
                         max(radius,config.spacing_multiplier*spacing))
        lo = bisect_left(projections, projection-radius)
        hi = bisect_right(projections, projection+radius)
        available = hi-lo
        count = cumulative[hi]-cumulative[lo]
        fraction = count/available
        if count >= config.min_support_points and fraction >= config.support_fraction:
            supported.append(dict(projection_m=projection, beam_index=beam,
                                  point_index=index, radius_m=radius,
                                  support_points=count, available_points=available,
                                  support_fraction=fraction, local_spacing_m=spacing))
    diagnostics = {'supported_point_count':len(supported),
                   'input_inlier_count':len(inliers)}
    if len(supported) < 2:
        return dict(valid=False,reason='insufficient_support',**diagnostics)
    a,b = supported[0],supported[-1]
    length = b['projection_m']-a['projection_m']
    if length < config.min_length_m:
        return dict(valid=False,reason='supported_extent_too_short',**diagnostics)
    raw = [p for p,_,_,inside in samples if inside]
    return dict(valid=True,reason='supported',
                endpoint_a=[c+a['projection_m']*t for c,t in zip(center,tangent)],
                endpoint_b=[c+b['projection_m']*t for c,t in zip(center,tangent)],
                observed_length_m=length,
                trim_a_m=max(0,a['projection_m']-min(raw)),
                trim_b_m=max(0,max(raw)-b['projection_m']),
                support_a=a,support_b=b,**diagnostics)
