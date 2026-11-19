"""Experimental geometry-only rectification of upright, light plate crops.

Input is the original detector crop, not OCR text. A failed quad search retains
the crop unchanged. The target aspect is a declared plate-type prior, not an
estimate of its metric dimensions from one image.
"""
import cv2
import numpy as np

PARAMETERS = {'gaussian_kernel': 3, 'canny_low': 50, 'canny_high': 150,
              'close_kernel': 3, 'approx_perimeter_fraction': 0.02,
              'min_area_fraction': 0.2, 'min_width_fraction': 0.5,
              'aspect_range': [2.0, 8.0], 'min_edge_px': 5.0,
              'max_colored_edge_fraction': 0.2, 'color_neighborhood_kernel': 7}


def order_quad(points):
    points = np.asarray(points, dtype=np.float32).reshape(4, 2)
    if not np.isfinite(points).all() or len(np.unique(points, axis=0)) != 4:
        raise ValueError('Nonfinite or repeated corner')
    center = points.mean(axis=0)
    angles = np.arctan2(points[:, 1]-center[1], points[:, 0]-center[0])
    points = points[np.argsort(angles)]
    points = np.roll(points, -np.argmin(points.sum(axis=1)), axis=0)
    if not cv2.isContourConvex(points) or cv2.contourArea(points) < 1:
        raise ValueError('Degenerate quadrilateral')
    return points  # top-left, top-right, bottom-right, bottom-left for upright plates


def find_plate_quad(image):
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(gray, (3, 3), 0), 50, 150)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    saturation = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 1]
    # Canny/approximation can sit just beside a colored overlay stroke.
    colored_nearby = cv2.dilate((saturation > 100).astype(np.uint8), np.ones((7, 7), np.uint8))
    candidates = []
    for contour in contours:
        polygon = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)
        if len(polygon) != 4:
            continue
        try:
            quad = order_quad(polygon)
        except ValueError:
            continue
        area = cv2.contourArea(quad)
        if area/(height*width) < 0.2 or np.ptp(quad[:, 0]) < width*0.5:
            continue
        # Do not use a detector border at the crop boundary as a plate corner.
        if quad[:, 0].min() <= 1 or quad[:, 1].min() <= 1 or quad[:, 0].max() >= width-2 or quad[:, 1].max() >= height-2:
            continue
        lengths = np.linalg.norm(np.roll(quad, -1, axis=0)-quad, axis=1)
        if lengths.min() < 5:
            continue
        aspect = float((lengths[0]+lengths[2])/(lengths[1]+lengths[3]))
        if not 2 <= aspect <= 8:
            continue
        edge_mask = np.zeros((height, width), np.uint8)
        cv2.polylines(edge_mask, [quad.astype(np.int32)], True, 255, 2)
        colored = float(np.mean(colored_nearby[edge_mask > 0] > 0))
        if colored > 0.2:
            continue
        candidates.append({'corners_xy': quad.tolist(), 'area_fraction': area/(height*width),
                           'observed_edge_aspect': aspect, 'colored_edge_fraction': colored})
    candidates.sort(key=lambda candidate: candidate['area_fraction'], reverse=True)
    return (np.asarray(candidates[0]['corners_xy'], np.float32) if candidates else None,
            {'status': 'quad_found' if candidates else 'no_quad_original_retained',
             'candidates': candidates})


def warp_quad(image, quad, target_size):
    quad = order_quad(quad)
    width, height = target_size
    if width < 2 or height < 2:
        raise ValueError('Invalid target size')
    target = np.float32([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]])
    matrix = cv2.getPerspectiveTransform(quad, target)
    warped = cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_CUBIC,
                                 borderMode=cv2.BORDER_REPLICATE)
    return warped, matrix


def prepare_variants(image, target_size=(520, 110)):
    quad, diagnostics = find_plate_quad(image)
    images = {'original': image.copy(),
              'resize_only': cv2.resize(image, target_size, interpolation=cv2.INTER_CUBIC)}
    if quad is None:
        images.update(recrop_resize=image.copy(), perspective=image.copy())
        diagnostics.update(corners_xy=None, transform=None, recrop_xyxy=None)
    else:
        x1, y1 = np.floor(quad.min(axis=0)).astype(int)
        x2, y2 = np.ceil(quad.max(axis=0)).astype(int)+1
        images['recrop_resize'] = cv2.resize(image[y1:y2, x1:x2], target_size, interpolation=cv2.INTER_CUBIC)
        images['perspective'], matrix = warp_quad(image, quad, target_size)
        diagnostics.update(corners_xy=quad.tolist(), transform=matrix.tolist(),
                           recrop_xyxy=[int(x1), int(y1), int(x2), int(y2)])
    diagnostics['target_size_wh'] = list(target_size)
    return images, diagnostics
