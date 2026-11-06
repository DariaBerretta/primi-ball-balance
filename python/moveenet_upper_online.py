#!/usr/bin/env python3
"""MoveEnet upper-body YARP sidecar for the PRIMI online C++ pipeline.

A single blocking inference worker is intentional: C++ acquisition and optical
flow remain independent. Each reply echoes the request's original YARP Stamp.
"""
import argparse
import os
import signal
import sys
import time

import numpy as np
import torch
import yarp

from pycore.moveenet import MoveNet, Task
from pycore.moveenet.config import cfg as base_cfg

RUNNING = True


def stop_handler(signum, frame):
    global RUNNING
    RUNNING = False


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=['left', 'right'], default='left')
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--decode-confidence', type=float, default=0.1)
    return parser.parse_args()


def main():
    global RUNNING
    args = parse_args()
    if not os.path.isfile(args.checkpoint):
        raise FileNotFoundError(args.checkpoint)

    config = dict(base_cfg)
    config['num_classes'] = 7
    config['img_size'] = 192
    config['GPU_ID'] = '0'
    config['gpu'] = True
    config['cfg_verbose'] = False
    config['confidence_th'] = args.decode_confidence
    config['write_output'] = False

    torch.set_num_threads(2)
    model = MoveNet(num_classes=7, width_mult=1.0, mode='train')
    task = Task(config, model)
    task.modelLoad(args.checkpoint)
    task.model.eval()

    # modelLoad() in upstream hpe-core catches missing files itself. The explicit
    # file check above prevents the most common silent failure.
    print('[MoveEnet] Model ready, device:', task.device, flush=True)

    yarp.Network.init()
    if not yarp.Network.checkNetwork(2.0):
        raise RuntimeError('YARP name server is unavailable')
    base = '/primi-ball-balance/moveEnetOFK/{}/inference'.format(args.camera)
    image_in = yarp.BufferedPortImageMono()
    pose_out = yarp.Port()
    if not image_in.open(base + '/img:i'):
        raise RuntimeError('Cannot open ' + base + '/img:i')
    if not pose_out.open(base + '/sklt:o'):
        image_in.close()
        raise RuntimeError('Cannot open ' + base + '/sklt:o')

    input_stamp = yarp.Stamp()
    out_bottle = yarp.Bottle()
    processed = 0
    try:
        print('[MoveEnet] Waiting for EROS on', base + '/img:i', flush=True)
        while RUNNING:
            source_image = image_in.read(True)
            if source_image is None:
                break
            if not image_in.getEnvelope(input_stamp):
                print('[MoveEnet] Dropping image without source stamp', flush=True)
                continue

            width = source_image.width()
            height = source_image.height()
            image_buffer = np.empty((height, width), dtype=np.uint8)
            image = yarp.ImageMono()
            image.resize(width, height)
            image.setExternal(image_buffer.data, width, height)
            image.copy(source_image)

            start = time.perf_counter()
            output = task.predict_online(image_buffer)
            infer_ms = (time.perf_counter() - start) * 1000.0
            xy = np.asarray(output['joints'], dtype=np.float64).reshape(26)
            confidence = np.asarray(output['confidence'], dtype=np.float64).reshape(13)
            values = np.concatenate((xy, confidence))

            out_bottle.clear()
            out_bottle.addString('SKLT')
            arr = out_bottle.addList()
            for value in values:
                arr.addFloat64(float(value))

            # This is the request stamp, not inference completion time.
            # The C++ LC filter needs the original surface timestamp.
            output_stamp = yarp.Stamp(input_stamp.getCount(),
                                       input_stamp.getTime())
            pose_out.setEnvelope(output_stamp)
            pose_out.write(out_bottle)
            processed += 1
            if processed % 30 == 0:
                print('[MoveEnet] processed={}, inference_ms={:.1f}'.format(
                    processed, infer_ms), flush=True)
    finally:
        image_in.close()
        pose_out.close()
        yarp.Network.fini()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        sys.exit(main() or 0)
    except Exception as error:
        print('[MoveEnet] FATAL:', repr(error), file=sys.stderr, flush=True)
        raise
