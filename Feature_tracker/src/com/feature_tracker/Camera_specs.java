package com.feature_tracker;

import java.io.IOException;
import java.io.InputStream;

import Jama.Matrix;
/**
 * Holds intrinsic camera parameters and performs undistortion.
 * 
 * @author johannes
 */
public class Camera_specs {

        private final Matrix intrinsics;
        private final Matrix distCoeffs;
        private final Matrix invIntrinsics;

        private final double cx, cy, fx, fy;
        private final double k1, k2, p1, p2;

        /**
         * Create a new camera.
         * 
         * @param intr
         *            the file where the camera's intrinsic matrix is stored
         * @param dist
         *            the file where the camera's distortion coefficients are stored. If null, (0,0,0,0)
         *            will be assumed.
         * @throws IOException
         *             if one of the files couldn't be read
         */
        public Camera_specs(InputStream intr, InputStream dist) throws IOException {
                intrinsics = new Matrix(MatrixUtils.loadMatrix(intr), 3).transpose();
                distCoeffs = new Matrix(dist == null ? new double[] { 0, 0, 0, 0 }
                                : MatrixUtils.loadMatrix(dist), 1);
                invIntrinsics = intrinsics.inverse();

                cx = intrinsics.get(0, 2);
                cy = intrinsics.get(1, 2);
                fx = intrinsics.get(0, 0);
                fy = intrinsics.get(1, 1);

                k1 = distCoeffs.get(0, 0);
                k2 = distCoeffs.get(0, 1);
                p1 = distCoeffs.get(0, 2);
                p2 = distCoeffs.get(0, 3);
        }

        /**
         * Create a new camera with a given camera matrix and no distortion.
         * 
         * @param intr
         *            the camera matrix
         */
        public Camera_specs(Matrix intr) {
                intrinsics = intr;
                distCoeffs = new Matrix(new double[] { 0, 0, 0, 0 }, 1);
                invIntrinsics = intr.inverse();

                cx = intrinsics.get(0, 2);
                cy = intrinsics.get(1, 2);
                fx = intrinsics.get(0, 0);
                fy = intrinsics.get(1, 1);

                k1 = k2 = p1 = p2 = 0;
        }

        /**
         * Performs undistortion on a point.
         * 
         * @param p
         *            the point in image coordinates (observed point coordinates)
         * @return the ideal point coordinates
         */
        public synchronized Point2d undistort(HasCoordinates2d p) {
                double x = (p.getX() - cx) / fx;
                double y = (p.getY() - cy) / fy;

                final double r2 = x * x + y * y;
                final double r4 = r2 * r2;

                final double a = 1 + k1 * r2 + k2 * r4;
                double xp = x * a + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
                double yp = y * a + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

                return new Point2d(fx * xp + cx, fy * yp + cy);
        }

        /**
         * Return the inverse of the intrinsic matrix.
         * 
         * @return the inverse of the camera matrix
         */
        public Matrix getInvIntrinsics() {
                return invIntrinsics;
        }

        /**
         * Return the intrinsic matrix.
         * 
         * @return the camera matrix
         */
        public Matrix getIntrinsics() {
                return intrinsics;
        }

}
