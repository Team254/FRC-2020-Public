import numpy as np
import cv2

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# image resolution
w_px = 960
h_px = 720


def calibrate_ll(w, h, file_min, file_max):
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(h - 1,w - 1,0)
    pts = np.zeros((h * w, 3), np.float32)
    pts[:, :2] = np.mgrid[0:h, 0:w].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    obj_pts = []  # 3d point in real world space
    img_pts = []  # 2d points in image plane.

    files = range(file_min, file_max + 1)

    for file in files:
        img = cv2.imread(f'{file}.jpg')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (h, w), None)

        # If found, add object points, image points (after refining them)
        if ret:
            obj_pts.append(pts)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_pts.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (h, w), corners2, ret)
            cv2.imwrite(f'{file}-corners.png', img)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, gray.shape[::-1], None, None)

    for file in files:
        img = cv2.imread(f'{file}.jpg')

        dst = cv2.undistort(img, mtx, dist)
        cv2.imwrite(f'{file}-undistorted.png', dst)

    scaled_mtx = np.array([
        list(map(lambda x: x / w_px, mtx[0])),
        list(map(lambda x: x / h_px, mtx[1])),
        mtx[2]
    ])

    fovx, fovy, focal_length, principal_point, aspect_ratio = cv2.calibrationMatrixValues(mtx, (w_px, h_px), 10, 10)

    return mtx, scaled_mtx, dist, fovx, fovy
