"""Pixel-to-floor calibration tables; lookups outside the samples are None."""
import csv
import math
from pathlib import Path

import numpy as np
from scipy.interpolate import LinearNDInterpolator


HEADER = ('PixelX', 'PixelY', 'ValueX', 'ValueY')


def _samples(values, name):
    array = np.array(values, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2 or len(array) < 3:
        raise ValueError(f'A calibration needs at least three {name} samples of two values')
    if not np.isfinite(array).all():
        raise ValueError(f'Calibration {name} samples must be finite')
    if np.linalg.matrix_rank(array - array.mean(axis=0)) < 2:
        raise ValueError(f'Calibration {name} samples must not be collinear')
    return array


def write_table(path, rows):
    """Write (pixel x, pixel y, floor x, floor y) rows."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(HEADER)
        for row in rows:
            writer.writerow([f'{float(value):.3f}' for value in row])


class Calibration:
    def __init__(self, pixels, points):
        self.pixels = _samples(pixels, 'pixel')
        self.points = _samples(points, 'floor')
        if len(self.pixels) != len(self.points):
            raise ValueError('Each pixel sample needs exactly one floor sample')
        self._to_floor = LinearNDInterpolator(self.pixels, self.points)
        self._to_pixel = LinearNDInterpolator(self.points, self.pixels)

    @classmethod
    def from_csv(cls, path):
        pixels, points = [], []
        with open(path, newline='') as file:
            reader = csv.reader(file)
            header = next(reader, None)
            if header is None or tuple(cell.strip() for cell in header) != HEADER:
                raise ValueError(f'{path}: expected the header {",".join(HEADER)}')
            for number, row in enumerate(reader, start=2):
                if not any(cell.strip() for cell in row):
                    continue
                if len(row) != 4:
                    raise ValueError(f'{path} line {number}: expected four values')
                try:
                    pixel_x, pixel_y, x, y = map(float, row)
                except ValueError as error:
                    raise ValueError(f'{path} line {number}: {error}') from None
                pixels.append((pixel_x, pixel_y))
                points.append((x, y))
        return cls(pixels, points)

    @staticmethod
    def _lookup(interpolate, query):
        if len(query) != 2 or not all(math.isfinite(float(value)) for value in query):
            return None
        result = interpolate(np.array([[float(query[0]), float(query[1])]]))[0]
        if not np.isfinite(result).all():
            return None
        return (float(result[0]), float(result[1]))

    def to_floor(self, pixel):
        """Floor point (metres, in the table's frame) at a pixel, or None."""
        return self._lookup(self._to_floor, pixel)

    def to_pixel(self, point):
        """Pixel of a floor point, or None outside the samples."""
        return self._lookup(self._to_pixel, point)

    def pixels_for(self, points):
        """Vectorised to_pixel; NaN outside the samples."""
        return self._to_pixel(np.asarray(points, dtype=float))
