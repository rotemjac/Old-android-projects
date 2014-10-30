package com.feature_tracker;

import java.util.List;
import Jama.Matrix;
import org.opencv.core.Point;
/**
 * Estimates the essential matrix.
 * 
 * @author EF
 */
public class Essential_Matrix_Est {

        private Matrix essentialMatrix, refinedEssentialMatrix;
        private double error, refinedError;
        public List<Point2d> pointsIM1;
    	public List<Point2d> pointsIM2;
    	public Matrix model_points1 = null;
    	public Matrix model_points2 = null;

        /**
         * Construct a new essential matrix estimator for two images.
         * 
         * @param im1
         *            the first view
         * @param im2
         *            the second view
         * @param c
         *            the camera used in both views
         */
        public Essential_Matrix_Est(List<Point> interest_pointsIM1, List<Point> interest_pointsIM2, Camera_specs c) {
        	this.pointsIM1 = VisionGeoTransforms.convertOcvPointToPoint2d(interest_pointsIM1);
        	this.pointsIM2 = VisionGeoTransforms.convertOcvPointToPoint2d(interest_pointsIM2);
        	estimateEssentialMatrix(c);
        }

        /**
         * Gets homogenized undistorted image points.
         * 
         * @param c
         *            the camera to use for undistorting
         * @param interest_points
         *            the points in pixels
         * @return a 3x(interest_points.size()) Matrix on image plane with z=1 [this is the known (u,v,1) style]
         */
        public Matrix HomoganizePoints(Camera_specs c, List<Point2d> interest_points) {
                Matrix result = new Matrix(3, interest_points.size());
                int i = 0;
                for (Point2d point : interest_points) {
                        Point2d p = c.undistort(point);
                        result.set(0, i, p.x);
                        result.set(1, i, p.y);
                        result.set(2, i, 1);
                        i++;
                }
                return result;
        }

        /**
         * Normalizes image points. meaning, multiply every point with Kinv
         * 
         * @param points
         *            the points to normalize
         * @param kInv
         *            the inverse of the camera matrix
         */
        private void normalize(Matrix points, Matrix kInv) {
                Matrix d = new Matrix(3, 1);
                for (int i = 0; i < points.getColumnDimension(); i++) {
                        for (int j = 0; j < 3; j++)
                                d.set(j, 0, points.get(j, i));/*simply take a point and put it in a col vector*/
                        points.setMatrix(0, 2, i, i, kInv.times(d));
                }
        }

        /**
         * Create the "A" matrix
         * 
         * @param points1
         *            normalized points from first image
         * @param points2
         *            normalized points from second image
         * @return matrix A (see slides)
         */
        /*forming the matrix for the 8 point ALG*/
        private Matrix buildA(Matrix points1, Matrix points2) {
                Matrix result = new Matrix(points1.getColumnDimension(), 9);
                for (int i = 0; i < result.getRowDimension(); i++) {
                        double x1i = points1.get(0, i);
                        double x2i = points2.get(0, i);
                        double y1i = points1.get(1, i);
                        double y2i = points2.get(1, i);

                        double[] row = new double[] { x1i * x2i, y1i * x2i, x2i, x1i * y2i, y1i * y2i, y2i,
                                        x1i, y1i, 1 };
                        for (int col = 0; col < 9; col++)
                                result.set(i, col, row[col]);
                }
                return result;
        }

        /**
         * Compute the error · (x2^T * E * x1)^2
         * 
         * @param e
         *            essential matrix
         * @param imgpts1
         *            3xn matrix: (xi, yi, 1)
         * @param imgpts2
         *            3xn matrix: (xi, yi, 1)
         * @return the error
         */
        private double getError(Matrix e, Matrix imgpts1, Matrix imgpts2) {
                double val = 0;
                for (int i = 0; i < imgpts1.getColumnDimension(); i++) {
                        Matrix pt2 = imgpts2.getMatrix(0, 2, i, i).transpose();
                        Matrix pt1 = imgpts1.getMatrix(0, 2, i, i);

                        Matrix pt2timesE = pt2.times(e);

                        val += Math.pow(pt2timesE.times(pt1).get(0, 0), 2);
                }
                return val;
        }

        /**
         * Estimate essential matrix
         * 
         * @param c
         *            the camera used in both views
         */
        private void estimateEssentialMatrix(Camera_specs c) {
                final Matrix imgpts2 = HomoganizePoints(c, this.pointsIM1);
                final Matrix imgpts1 = HomoganizePoints(c, this.pointsIM2);
                model_points1 = imgpts1.copy();
                model_points2 = imgpts2.copy();
                
                normalize(imgpts1, c.getInvIntrinsics());
                normalize(imgpts2, c.getInvIntrinsics());
                essentialMatrix = new Matrix(3, 3);
            
                final Matrix a = buildA(imgpts1, imgpts2);
                final Matrix ata = a.transpose().times(a);

                final Pair<double[], Matrix[]> eigs = MatrixUtils.eig(ata);
                // find evector
                double minEval = Math.abs(eigs.a[0]);
                Matrix evector = eigs.b[0];
                /* find min eigenvalue -> get nullspace vector and make it the essential matrix*/
                for (int i = 1; i < eigs.a.length; i++) {
                        if (minEval > Math.abs(eigs.a[i])) {
                                minEval = Math.abs(eigs.a[i]);
                                evector = eigs.b[i];
                        }
                }
                for (int i = 0; i < 9; i++) {
                        essentialMatrix.set(i / 3, i % 3, evector.get(i, 0));
                }
                /**/
                error = getError(essentialMatrix, imgpts1, imgpts2);
//                SingularValueDecomposition svd = new SingularValueDecomposition(essentialMatrix);
//                Matrix s = svd.getS();
//                System.out.println("S:\n" + MatrixUtils.matrixToString(s));
//                s.set(2, 2, 0);
//                refinedEssentialMatrix = svd.getU().times(s).times(svd.getV().transpose());
//
//                refinedError = getError(refinedEssentialMatrix, imgpts1, imgpts2);
        }

        /**
         * Return the estimated essential matrix.
         * 
         * @return the essential matrix
         */
        public Matrix getEssentialMatrix() {
                return essentialMatrix;
        }

        /**
         * <p>
         * Return the refined essential matrix.
         * </p>
         * <p>
         * The refined essential matrix's singular values are (s1, s2, 0).
         * </p>
         * 
         * @return the refined essential matrix.
         */
        public Matrix getRefinedEssentialMatrix() {
                return refinedEssentialMatrix;
        }

        /**
         * Return the error of the essential matrix: · (x2^T * E * x1)^2
         * 
         * @return the error
         */
        public double getError() {
                return error;
        }

        /**
         * Return the error of the refined essential matrix: · (x2^T * E' * x1)^2
         * 
         * @return the error
         */
        public double getRefinedError() {
                return refinedError;
        }

}