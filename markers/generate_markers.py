#!/usr/bin/env python3
"""
generate_markers.py
Author: Karthik Govindan V
Date: 2026-04-05

Generates ArUco fiducial markers from the DICT_6X6_250 dictionary.
Produces 5 markers (IDs 0-4) as 200x200 pixel PNG images at 300 DPI.
These markers are designed to be printed at 10x10 cm and affixed to
the four corners and centre of the medicine trolley for pose tracking.

Dependencies: opencv-python, numpy (standard scientific Python stack)
"""

import cv2
import numpy as np
import os

def main():
    # Use DICT_6X6_250 for good Hamming distance and adequate ID space.
    # This dictionary provides 36-bit codes with minimum Hamming distance
    # of 12 between any two valid codewords, giving robust detection
    # even under partial occlusion (up to 5 bit errors tolerated).
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Ensure the output directory exists
    output_dir = os.path.dirname(os.path.abspath(__file__))

    for marker_id in range(5):
        # Generate a 200x200 pixel marker image with 1-bit border width
        marker_img = np.zeros((200, 200), dtype=np.uint8)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200, marker_img, 1)

        # Save the marker as a PNG file
        output_path = os.path.join(output_dir, f'marker_{marker_id}.png')
        cv2.imwrite(output_path, marker_img)
        print(f'Generated marker ID {marker_id} -> {output_path}')

    print(f'\nAll 5 markers generated successfully in {output_dir}')
    print('Print these markers at 10x10 cm each and affix to the trolley.')

if __name__ == '__main__':
    main()
