#!/usr/bin/env python
# -*- coding: utf-8 -*-
import copy
import time
import argparse

import cv2 as cv
from pupil_apriltags import Detector


def get_args():
    parser = argparse.ArgumentParser()

    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--width", help='cap width', type=int, default=960)
    parser.add_argument("--height", help='cap height', type=int, default=540)

    parser.add_argument("--families", type=str, default='tag36h11')
    parser.add_argument("--nthreads", type=int, default=1)
    parser.add_argument("--quad_decimate", type=float, default=2.0)
    parser.add_argument("--quad_sigma", type=float, default=0.0)
    parser.add_argument("--refine_edges", type=int, default=1)
    parser.add_argument("--decode_sharpening", type=float, default=0.25)
    parser.add_argument("--debug", type=int, default=0)

    args = parser.parse_args()

    return args


def main(camera):
    # 引数解析 #################################################################
    args = get_args()

    cap_device = args.device
    cap_width = args.width
    cap_height = args.height

    families = args.families
    nthreads = args.nthreads
    quad_decimate = args.quad_decimate
    quad_sigma = args.quad_sigma
    refine_edges = args.refine_edges
    decode_sharpening = args.decode_sharpening
    debug = args.debug

    # カメラ準備 ###############################################################
    # cap = cv.VideoCapture(cap_device)
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, cap_width)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, cap_height)

    # Detector準備 #############################################################
    at_detector = Detector(
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=refine_edges,
        decode_sharpening=decode_sharpening,
        debug=debug,
    )

    elapsed_time = 0

    while True:
        start_time = time.time()

        # カメラキャプチャ #####################################################
        # ret, image = cap.read()
        image, depth = camera.read()
        debug_image = copy.deepcopy(image)
        debug_image = cv.cvtColor(debug_image, cv.COLOR_BGR2RGB)

        # 検出実施 #############################################################
        image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        #640x480  p[318.166 237.173]  f[386.933 386.933]  Brown Conrady [0 0 0 0 0] 

        tags = at_detector.detect(
            image,
            estimate_tag_pose=True,
            camera_params=(385.098, 385.098, 324.787, 240.768),
            # camera_params=(386.933, 386.933, 318.166, 237.173),
            tag_size=0.244#0.244, 0.174
        )

        #####

        


        # 描画 ################################################################
        debug_image = draw_tags(debug_image, tags, elapsed_time)

        elapsed_time = time.time() - start_time

        # キー処理(ESC：終了) #################################################
        key = cv.waitKey(1)
        if key == 27:  # ESC
            break

        # 画面反映 #############################################################
        cv.imshow('AprilTag Detect Demo', debug_image)

    # cap.release()
    cv.destroyAllWindows()


def draw_tags(
    image,
    tags,
    elapsed_time,
):
    for tag in tags:
        tag_family = tag.tag_family
        tag_id = tag.tag_id
        center = tag.center
        corners = tag.corners
        # print('tags pose_t:', tag.pose_t)
        # print('tags pose_R:', tag.pose_R)
        import numpy as np
        H_camera_T = np.concatenate([tag.pose_R, tag.pose_t], axis=1)
        H_camera_T = np.concatenate([H_camera_T, np.array([[0, 0, 0, 1]])], axis=0)

        H_R_T = np.array([[-1, 0, 0, 0.711], [0, 1, 0, 0.564], [0, 0, -1, 0.], [0, 0, 0, 1]])

        H_R_C = np.matmul(H_R_T, np.linalg.inv(H_camera_T))

        actual_H_R_C = np.asarray([[-0.20383387, -0.0052649,  -0.01631416,  0.07197962],
        [ 0.01297532,  0.07983816, -0.18788195,  1.60119342],
        [ 0.0112033,  -0.18825584 -0.07922333,  0.65564981],
        [ 0.,          0.,          0.,          1.        ]])


        print('H_R_C:', H_R_C)
        print('actual_H_R_C:', actual_H_R_C)


        center = (int(center[0]), int(center[1]))
        corner_01 = (int(corners[0][0]), int(corners[0][1]))
        corner_02 = (int(corners[1][0]), int(corners[1][1]))
        corner_03 = (int(corners[2][0]), int(corners[2][1]))
        corner_04 = (int(corners[3][0]), int(corners[3][1]))

        # 中心
        cv.circle(image, (center[0], center[1]), 5, (0, 0, 255), 2)

        # 各辺
        cv.line(image, (corner_01[0], corner_01[1]),
                (corner_02[0], corner_02[1]), (255, 0, 0), 2)
        cv.line(image, (corner_02[0], corner_02[1]),
                (corner_03[0], corner_03[1]), (255, 0, 0), 2)
        cv.line(image, (corner_03[0], corner_03[1]),
                (corner_04[0], corner_04[1]), (0, 255, 0), 2)
        cv.line(image, (corner_04[0], corner_04[1]),
                (corner_01[0], corner_01[1]), (0, 255, 0), 2)

        # タグファミリー、タグID
        # cv.putText(image,
        #            str(tag_family) + ':' + str(tag_id),
        #            (corner_01[0], corner_01[1] - 10), cv.FONT_HERSHEY_SIMPLEX,
        #            0.6, (0, 255, 0), 1, cv.LINE_AA)
        cv.putText(image, str(tag_id), (center[0] - 10, center[1] - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2, cv.LINE_AA)

    # 処理時間
    cv.putText(image,
               "Elapsed Time:" + '{:.1f}'.format(elapsed_time * 1000) + "ms",
               (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
               cv.LINE_AA)

    return image


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        device_id = int(sys.argv[1])
    else :
        device_id = 0

    from gello.cameras.realsense_camera import RealSenseCamera, get_device_ids
    device_ids = get_device_ids()
    print(f"Found {len(device_ids)} devices")
    rs = RealSenseCamera(flip=False, device_id=device_ids[device_id])
    im, depth = rs.read()
    main(camera=rs)