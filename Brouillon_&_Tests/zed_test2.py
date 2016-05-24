import cv2
import numpy as np

# Load test images
# TEST_IMAGES is a list of paths to test images
TEST_IMAGES = ['chessboards/zed_picR_cal1.jpg', 'chessboards/zed_picL_cal1.jpg']
input_l, input_r = [cv2.imread(image, cv2.IMREAD_GRAYSCALE)
                    for image in TEST_IMAGES]
image_size = input_l.shape[:2]

# Retrieve chessboard corners
# CHESSBOARD_ROWS and CHESSBOARD_COLUMNS are the number of inside rows and
# columns in the chessboard used for calibration
CHESSBOARD_ROWS = 6
CHESSBOARD_COLUMNS = 9
pattern_size = CHESSBOARD_ROWS, CHESSBOARD_COLUMNS
object_points = np.zeros((np.prod(pattern_size), 3), np.float32)
object_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
# SQUARE_SIZE is the size of the chessboard squares in cm
SQUARE_SIZE = 2.5
object_points *= SQUARE_SIZE
image_points = {}
ret, corners_l = cv2.findChessboardCorners(input_l, pattern_size, True)
cv2.cornerSubPix(input_l, corners_l,
                 (11, 11), (-1, -1),
                 (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
                  30, 0.01))
image_points["left"] = corners_l.reshape(-1, 2)
ret, corners_r = cv2.findChessboardCorners(input_r, pattern_size, True)
cv2.cornerSubPix(input_r, corners_r,
                 (11, 11), (-1, -1),
                 (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
                  30, 0.01))
image_points["right"] = corners_r.reshape(-1, 2)

# Calibrate cameras
(cam_mats, dist_coefs, rect_trans, proj_mats, valid_boxes,
 undistortion_maps, rectification_maps) = {}, {}, {}, {}, {}, {}, {}
criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS,
            100, 1e-5)
flags = (cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_ZERO_TANGENT_DIST +
         cv2.CALIB_SAME_FOCAL_LENGTH)
(ret, cam_mats["left"], dist_coefs["left"], cam_mats["right"],
 dist_coefs["right"], rot_mat, trans_vec, e_mat,
 f_mat) = cv2.stereoCalibrate(object_points,
                              image_points["left"], image_points["right"],
                              image_size, criteria=criteria, flags=flags, distCoeffs1=dist_coefs)

(rect_trans["left"], rect_trans["right"],
 proj_mats["left"], proj_mats["right"],
 disp_to_depth_mat, valid_boxes["left"],
 valid_boxes["right"]) = cv2.stereoRectify(cam_mats["left"],
                                           dist_coefs["left"],
                                           cam_mats["right"],
                                           dist_coefs["right"],
                                           image_size,
                                           rot_mat, trans_vec, flags=0)
for side in ("left", "right"):
    (undistortion_maps[side],
     rectification_maps[side]) = cv2.initUndistortRectifyMap(cam_mats[side],
                                                             dist_coefs[side],
                                                             rect_trans[side],
                                                             proj_mats[side],
                                                             image_size,
                                                             cv2.CV_32FC1)

# Produce disparity map
rectified_l = cv2.remap(input_l, undistortion_maps["left"],
                        rectification_maps["left"],
                        cv2.INTER_NEAREST)
rectified_r = cv2.remap(input_r, undistortion_maps["right"],
                        rectification_maps["right"],
                        cv2.INTER_NEAREST)
cv2.imshow("left", rectified_l)
cv2.imshow("right", rectified_r)
block_matcher = cv2.StereoBM(0, 5)
# block_matcher = cv2.StereoBM(cv2.STEREO_BM_BASIC_PRESET, 0, 5)
disp = block_matcher.compute(rectified_l, rectified_r, disptype=cv2.CV_32F)
cv2.imshow("disparity", disp)
