

import cv2
import numpy as np
import os

def main():

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    output_dir = os.path.dirname(os.path.abspath(__file__))

    for marker_id in range(5):

        marker_img = np.zeros((200, 200), dtype=np.uint8)
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, 200, marker_img, 1)

        output_path = os.path.join(output_dir, f'marker_{marker_id}.png')
        cv2.imwrite(output_path, marker_img)
        print(f'Generated marker ID {marker_id} -> {output_path}')

    print(f'\nAll 5 markers generated successfully in {output_dir}')
    print('Print these markers at 10x10 cm each and affix to the surgical instrument.')

if __name__ == '__main__':
    main()
