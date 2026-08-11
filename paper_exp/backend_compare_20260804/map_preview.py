#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from PIL import Image


def read_pgm(path):
    with open(path, 'rb') as handle:
        if handle.readline().strip() != b'P5':
            raise RuntimeError('expected binary PGM')
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = map(int, line.split())
        if int(handle.readline()) != 255:
            raise RuntimeError('unsupported PGM range')
        return np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)


def make_preview(input_path, output_path):
    image = read_pgm(input_path)
    known = image != 205
    ys, xs = np.where(known)
    if len(xs) == 0:
        raise RuntimeError('map has no known cells')
    margin = 12
    x0 = max(0, int(xs.min()) - margin)
    x1 = min(image.shape[1], int(xs.max()) + margin + 1)
    y0 = max(0, int(ys.min()) - margin)
    y1 = min(image.shape[0], int(ys.max()) + margin + 1)
    crop = image[y0:y1, x0:x1]
    height, width = crop.shape
    scale = min(1.0, 1400.0 / max(width, height))
    if scale < 1.0:
        resized = Image.fromarray(crop).resize(
            (max(1, int(width * scale)), max(1, int(height * scale))),
            Image.Resampling.NEAREST)
    else:
        resized = Image.fromarray(crop)
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    resized.save(output_path)
    print('%s crop=%dx%d output=%s' % (input_path, width, height, output_path))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_pgm')
    parser.add_argument('output_png')
    args = parser.parse_args()
    make_preview(args.input_pgm, args.output_png)
