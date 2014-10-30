package com.feature_tracker;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

public class Abs_orientation extends RANSAC_wrapper<Triplet<Matrix, Matrix, Double>, Pair<Point2d, Point2d>, Double> {

	Matrix cam_mat = null;
	Abs_orientation(int iterations_limit, int modelset_size,
			int inliers_setsize, List<Pair<Point2d, Point2d>> all_constraints, Matrix camera) {
		super(iterations_limit, modelset_size, inliers_setsize, all_constraints);
		cam_mat = camera;
		RANSAC_loop();
	}


	@Override
	Triplet<Matrix, Matrix, Double> solver(
			List<Pair<Point2d, Point2d>> constraints) {
		Matrix R_ret = null;
		Matrix T_ret = null;
		Pair<Matrix, Matrix> model_points_sets = VisionGeoTransforms.get2CameraPointsMats(constraints,cam_mat);
		Matrix mean1 = CalculateMeanValue(model_points_sets.a);
		Matrix mean2 = CalculateMeanValue(model_points_sets.b);
		unbiasCoordinates(model_points_sets.a, mean1);
		unbiasCoordinates(model_points_sets.b, mean2);
		Double scale_factor = getScaleFactor(model_points_sets.a, model_points_sets.b);
		R_ret = rotationMatrix(model_points_sets.a, model_points_sets.b);
		T_ret = translationVector(R_ret, mean1, mean2, scale_factor);
		Triplet<Matrix, Matrix, Double> ret = new Triplet<Matrix, Matrix, Double>();
		ret.set(R_ret, T_ret, scale_factor);
		return ret;
	}

	@Override
	List<Integer> solutionQuality(Triplet<Matrix, Matrix, Double> chosen_solution, Triplet<Matrix, Matrix, Double> reference_solution) {
		List<Integer> good_indexes = new ArrayList<Integer>();
		Pair<Matrix, Matrix> model_points_sets = VisionGeoTransforms.get2CameraPointsMats(allConstraints,cam_mat);
		for (int i = 0; i < model_points_sets.a.getColumnDimension(); i++){
			Matrix pt2est_chosen = (chosen_solution.a).times(model_points_sets.a.getMatrix(0, 2, i, i)).times(chosen_solution.c);
			Matrix pt2est_reference = (reference_solution.a).times(model_points_sets.a.getMatrix(0, 2, i, i)).times(reference_solution.c);
			for (int j = 0; j < 3; j++)
			{
				pt2est_chosen.set(j, 0, pt2est_chosen.get(j, 0) + chosen_solution.b.get(j, 0));
				pt2est_reference.set(j, 0, pt2est_reference.get(j, 0) + reference_solution.b.get(j, 0));
			}
			double error_chosen = 0;
			double error_reference = 0;
			for (int j = 0; j < 3; j++) {
				error_chosen += Math.pow(pt2est_chosen.get(j, 0) - model_points_sets.b.get(j, i), 2);
				error_reference += Math.pow(pt2est_reference.get(j, 0) - model_points_sets.b.get(j, i), 2);
			}
			if (error_chosen <= error_reference){
				good_indexes.add(i); 
			}
		}
		return good_indexes;
	}

	@Override
	void estimateError() {
		Pair<Matrix, Matrix> model_points_sets = VisionGeoTransforms.get2CameraPointsMats(allConstraints,cam_mat);
		Matrix pts2est = (sol.a).times(model_points_sets.a).times(sol.c);
		for (int i = 0; i < pts2est.getColumnDimension(); i++) {
			pts2est.set(0, i, pts2est.get(0, i) + (sol.b).get(0, 0));
			pts2est.set(1, i, pts2est.get(1, i) + (sol.b).get(1, 0));
			pts2est.set(2, i, pts2est.get(2, i) + (sol.b).get(2, 0));
		}
		estError = (double) 0;
		for (int i = 0; i < pts2est.getColumnDimension(); i++) {
			estError += Math.pow(pts2est.get(0, i) - model_points_sets.b.get(0, i), 2);
			estError += Math.pow(pts2est.get(1, i) - model_points_sets.b.get(1, i), 2);
			estError += Math.pow(pts2est.get(2, i) - model_points_sets.b.get(2, i), 2);
		}
		// TODO Auto-generated method stub

	}


	/**
	 * Calculates mean value for each Matrix of points
	 * 
	 * @param points
	 *            Matrix with points
	 * @return Mean value (x,y,z)
	 */
	private static Matrix CalculateMeanValue(Matrix points) {
		double sumX = 0;
		double sumY = 0;
		double sumZ = 0;

		for (int i = 0; i < points.getColumnDimension(); i++) {
			sumX += points.get(0, i);
			sumY += points.get(1, i);
			sumZ += points.get(2, i);
		}
		sumX = sumX / points.getColumnDimension();
		sumY = sumY / points.getColumnDimension();
		sumZ = sumZ / points.getColumnDimension();
		Matrix mean = new Matrix(1, 3);
		mean.set(0, 0, sumX);
		mean.set(0, 1, sumY);
		mean.set(0, 2, sumZ);
		return mean;
	}

	/**
	 * Computes unbiased coordinates (x_i = x_i-x_mean)
	 * 
	 * @param points
	 *            Matrix of points
	 * @param mean
	 *            Mean value (x,y,z)
	 */
	private static void unbiasCoordinates(Matrix points, Matrix mean) {
		for (int i = 0; i < points.getColumnDimension(); i++) {
			points.set(0, i, (points.get(0, i) - mean.get(0, 0)));
			points.set(1, i, (points.get(1, i) - mean.get(0, 1)));
			points.set(2, i, (points.get(2, i) - mean.get(0, 2)));
		}
	}

	/**
	 * Computes scaling factor (for rigid transformation s = 1)
	 * 
	 * @param pts1
	 *            normalized coordinates 1
	 * @param pts2
	 *            normalized coordinates 2
	 * @return scaling factor (double)
	 */
	private static double getScaleFactor(Matrix pts1, Matrix pts2) {
		double r1 = 0;
		double r2 = 0;
		for (int i = 0; i < pts1.getColumnDimension(); i++) {
			r1 += pts1.get(0, i) * pts1.get(0, i) + pts1.get(1, i) * pts1.get(1, i)
					+ pts1.get(2, i) * (pts1.get(2, i));
			r2 += pts2.get(0, i) * pts2.get(0, i) + pts2.get(1, i) * pts2.get(1, i)
					+ pts2.get(2, i) * (pts2.get(2, i));
		}
		return Math.sqrt(r2 / r1);
	}

	/**
	 * Computes rotation Matrix R (3x3)
	 * 
	 * @param pts1
	 *            normalized coordinates 1
	 * @param pts2
	 *            normalized coordinates 2
	 * @return Matrix R
	 */
	private static Matrix rotationMatrix(Matrix pts1, Matrix pts2) {
		Matrix M = pts1.times(pts2.transpose());
		Matrix Q = M.times(M.transpose());
		Matrix V = Q.eig().getV();
		double[] d = Q.eig().getRealEigenvalues();
		Matrix A = new Matrix(3, 3);
		for (int i = 0; i < 3; i++) {
			A.set(i, i, 1 / Math.sqrt(d[i]));
		}
		return M.transpose().times(V).times(A).times(V.transpose());
	}

	/**
	 * Computes translation vector T
	 * 
	 * @param R
	 *            Rotation matrix
	 * @param mean1
	 *            Mean value for 1st set of pts
	 * @param mean2
	 *            Mean value for 2nd set of pts
	 * @return Translation vector T (1x3)
	 */
	private Matrix translationVector(Matrix R, Matrix mean1, Matrix mean2, double scale_factor) {
		return mean2.transpose().minus(R.times(scale_factor).times(mean1.transpose()));
	}






}
