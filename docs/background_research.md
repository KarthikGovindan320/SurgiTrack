# Background Research: OpenCV and ArUco Markers in Healthcare Robotics

## 1. OpenCV Library Overview

### 1.1 History and Evolution

OpenCV (Open Source Computer Vision Library) was founded in 1999 at Intel Research by Gary Bradski as an initiative to advance CPU-intensive computer vision applications. The project aimed to provide a common infrastructure for computer vision research and to accelerate the adoption of machine perception in commercial products. After its initial release in 2000, the library gained traction in both academic and industrial communities, leading to its transfer to the OpenCV Foundation under the Apache 2.0 license.

The library is written primarily in C++ with optimised assembly-level routines for performance-critical paths, and offers bindings for Python, Java, and MATLAB. Its modular architecture splits functionality into stable core modules (`core`, `imgproc`, `calib3d`, `features2d`, `video`, `highgui`, `videoio`) and community-contributed modules (`aruco`, `face`, `tracking`, `ximgproc`). This separation allows the core API to maintain backward compatibility while enabling experimental features to evolve rapidly in the contrib repository.

### 1.2 Version History Relevant to This Project

The ArUco marker detection module was originally developed at the University of Cordoba by S. Garrido-Jurado et al. and was merged into the OpenCV contrib repository in **version 3.1 (2016)**. For several years, users needed to build OpenCV with the contrib modules explicitly enabled. In **OpenCV 4.7 (2023)**, the ArUco detection functionality was promoted to the main distribution under the `objdetect` module, making it available in standard binary distributions without additional build configuration. This promotion reflects the maturity and widespread adoption of fiducial marker systems in production applications.

### 1.3 Domains of Application

OpenCV serves as the backbone computer vision library across diverse domains:

- **Autonomous Vehicles**: Lane detection, pedestrian recognition, and visual odometry pipelines rely on OpenCV's image processing and feature detection modules.
- **Robotics Localisation**: Mobile robots use fiducial markers and visual SLAM techniques built on OpenCV primitives for indoor navigation.
- **Medical Imaging**: Preprocessing, segmentation, and registration of radiological images leverage OpenCV's filtering and geometric transformation routines.
- **Industrial Quality Control**: Automated optical inspection systems use template matching, contour analysis, and calibration routines from OpenCV.
- **Augmented Reality**: Marker-based and markerless AR applications depend on camera calibration, homography estimation, and pose recovery provided by the `calib3d` module.

---

## 2. ArUco Fiducial Markers

### 2.1 What Are ArUco Markers?

ArUco (Augmented Reality University of Cordoba) markers are binary square fiducial markers consisting of a wide black border surrounding an internal grid that encodes a unique binary ID. The black border provides high contrast against typical backgrounds, enabling robust detection under varying lighting conditions. The internal binary matrix is read by perspective-correcting the detected quadrilateral and sampling the grid cells to determine 0 (black) or 1 (white) values.

### 2.2 Detection Pipeline

The ArUco detection pipeline operates in four sequential stages:

1. **Adaptive Thresholding**: The input frame is converted to greyscale and binarised using adaptive thresholding with multiple window sizes. This produces several binary images that are independently processed, ensuring that markers are detected regardless of local illumination gradients.

2. **Contour-Based Corner Detection**: Connected components are extracted from each binary image. Quadrilateral contours (four-sided polygons) that meet size and aspect ratio constraints are retained as marker candidates. Sub-pixel corner refinement (`cv::cornerSubPix`) is then applied to improve corner localisation accuracy.

3. **Perspective-Corrected Bit Decoding**: Each candidate quadrilateral is warped to a canonical square using a perspective transformation. The internal grid cells are sampled and classified as black or white based on their mean intensity, producing a binary code word.

4. **Dictionary Lookup for ID Verification**: The extracted code word is compared against all valid code words in the selected dictionary. If the code word matches a dictionary entry within the maximum allowed bit error threshold (determined by the inter-marker Hamming distance), the marker is accepted and assigned the corresponding ID. Rotational ambiguity is resolved by testing all four 90-degree rotations of the extracted code word.

### 2.3 Supported Dictionaries

OpenCV provides predefined dictionaries ranging from `DICT_4X4_50` (4×4 grid, 50 unique IDs) to `DICT_7X7_1000` (7×7 grid, 1000 unique IDs). Larger grid sizes encode more bits, enabling larger dictionaries, but require higher image resolution for reliable decoding. The trade-off is between inter-marker Hamming distance (robustness to bit errors) and dictionary size (number of unique markers). For this project, **DICT_6X6_250** provides an optimal balance: 36-bit codes with a minimum Hamming distance of 12 between any two valid code words, supporting up to 250 unique markers while tolerating up to 5 bit errors during decoding.

### 2.4 Pose Estimation

Once marker corners are detected, `cv::aruco::estimatePoseSingleMarkers` estimates the 6-DoF (six degrees of freedom) pose of each marker relative to the camera. Internally, this function constructs a set of 3D object points representing the four corners of the marker in the marker's local coordinate frame (at ±side/2 in X and Y, Z=0) and calls `cv::solvePnP` (default solver: `SOLVEPNP_ITERATIVE`) to compute the rotation vector (`rvec`) and translation vector (`tvec`) that minimise the reprojection error between the known 3D points and their detected 2D image projections.

The rotation vector is expressed in Rodrigues form (a compact 3-component vector where the direction indicates the rotation axis and the magnitude indicates the rotation angle in radians). The translation vector gives the 3D position of the marker centre in the camera's coordinate frame, expressed in the same units as the marker size (metres in our configuration).

### 2.5 Key Limitation

Standard ArUco detection has **no built-in occlusion handling**. When a marker is partially blocked by a hand, object, or the trolley's own geometry, corner localisation degrades because one or more corners may be obscured. The resulting pose estimate error increases nonlinearly with the degree of occlusion, as `solvePnP` relies on accurate 2D corner positions to solve the perspective-n-point problem. This limitation motivates the use of temporal filtering (Kalman filter) to maintain trajectory estimates through brief occlusion events.

---

## 3. Domain Context: Hospital Medicine Trolleys

### 3.1 The Operational Challenge

Medicine trolleys (also called medication carts or crash carts) are mobile units that carry controlled substances, syringes, IV bags, and fragile vials through hospital wards, corridors, intensive care units, and operating theatres. These trolleys are pushed manually by nurses and orderlies through environments characterised by narrow corridors (typically 2.5 to 3 metres wide), doorframes, patient beds positioned close to walkways, and unpredictable foot traffic from patients, visitors, and other staff.

Collision incidents involving medicine trolleys can result in:
- **Medication loss**: Breakage of glass vials or spilling of liquid medications.
- **Cross-contamination**: Contact between sterile supplies and non-sterile surfaces.
- **Patient injury**: Impact with patients in beds or wheelchairs positioned near corridor walls.
- **Equipment damage**: Collision with IV poles, monitoring equipment, or other sensitive medical devices.

### 3.2 Limitations of Existing Solutions

Current trolley tracking solutions in hospitals rely primarily on:
- **RFID room-level tracking**: Provides location information at room granularity (which room the trolley is in) but cannot determine position within a room or corridor with sub-metre accuracy.
- **Barcode scanning**: Requires manual scanning at checkpoints, providing identity verification but no real-time spatial tracking.
- **BLE beacons**: Bluetooth Low Energy solutions offer approximate proximity detection (2–5 metre accuracy) but lack the precision needed for collision avoidance.

None of these solutions provide **real-time sub-centimetre pose estimation** or **trajectory prediction**, which are the capabilities needed for proactive collision avoidance.

### 3.3 Why Camera-Based Tracking?

A camera-plus-marker system using ArUco fiducials provides:
- **Sub-centimetre translation accuracy** in the 0.3 to 3 metre operational range typical in ward corridors, when using 10 cm markers and a calibrated 1080p camera.
- **Sub-degree rotation accuracy** at distances under 2 metres, enabling estimation of the trolley's heading direction.
- **30+ FPS processing rate** on a Raspberry Pi 4, making it deployable on low-cost, trolley-mounted hardware without cloud dependency.
- **No electromagnetic interference**: Unlike radio-frequency solutions, optical systems do not conflict with medical device electromagnetic compatibility (EMC) regulations.

By augmenting instantaneous pose estimates with temporal filtering through a Kalman filter, the system can predict the trolley's trajectory up to 500 milliseconds into the future and issue collision warnings before contact occurs — addressing the critical gap in existing hospital logistics infrastructure.

---

## References

[1] G. Bradski, "The OpenCV Library," *Dr. Dobb's Journal of Software Tools*, 2000.

[2] S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez, "Automatic generation and detection of highly reliable fiducial markers under occlusion," *Pattern Recognition*, vol. 47, no. 6, pp. 2280–2292, 2014.

[3] F. J. Romero-Ramirez, R. Muñoz-Salinas, and R. Medina-Carnicer, "Speeded up detection of squared fiducial markers," *Image and Vision Computing*, vol. 76, pp. 38–47, 2018.

[4] R. E. Kalman, "A New Approach to Linear Filtering and Prediction Problems," *Journal of Basic Engineering*, vol. 82, no. 1, pp. 35–45, 1960.

[5] Z. Zhang, "A Flexible New Technique for Camera Calibration," *IEEE Transactions on Pattern Analysis and Machine Intelligence*, vol. 22, no. 11, pp. 1330–1334, 2000.
