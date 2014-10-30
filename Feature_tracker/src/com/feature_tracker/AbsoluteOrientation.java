package com.feature_tracker;


import java.util.ArrayList;
import java.util.List;
import android.util.Log;
import Jama.Matrix;

/**
 * Computes rotation matrix R and translation vector T
 * 
 * @author EF
 */
public class AbsoluteOrientation {

	public Matrix T;
	public Matrix R;
	public static double error = 0;
	public double scaleFactor;
	static String TAG = "AbsoluteOrientation";

	/** Construct a RANSAC based rotation & translation matrix. 
	 * 
	 * @param pairedList [first in every pair is the source, second is destination]
	 * @param cam_mat
	 */

	public AbsoluteOrientation(List<Pair<Point2d, Point2d>> pairedList, Matrix cam_mat) {

		Pair<Matrix, Matrix> complete3dmatrixes = get2CameraPointsMats(pairedList,cam_mat);
		int counter = 0;
		while (true){
			/*1. choose a sublist*/
			List<Pair<Point2d, Point2d>> chosen = new ArrayList<Pair<Point2d, Point2d>>();
			List<Pair<Point2d, Point2d>> others = new ArrayList<Pair<Point2d, Point2d>>();
			VisionGeoTransforms.chooseRandSublist(pairedList, chosen, others, 4);

			/*2. calculate the two transforms*/
			Pair<Matrix, Matrix> chosen3dmatrixes = get2CameraPointsMats(chosen,cam_mat);
			Pair<Matrix, Matrix> others3dmatrixes = get2CameraPointsMats(others,cam_mat);
			Triplet<Matrix, Matrix, Double> chosen_transform = getRTS(chosen3dmatrixes);
			Triplet<Matrix, Matrix, Double> others_transform = getRTS(others3dmatrixes);

			/*3. check the quality of the chosen transform - if good make a final transform and stop iterating*/
			List <Integer> good_indexes = TransformQuality(chosen_transform, others_transform, complete3dmatrixes);
			if (good_indexes.size() >= 7){
				List<Pair<Point2d, Point2d>> qualityset = VisionGeoTransforms.pickSublist(pairedList, good_indexes);
				Pair<Matrix, Matrix> quality3dmatrixes = get2CameraPointsMats(qualityset,cam_mat);
				Triplet<Matrix, Matrix, Double> quality_transform = getRTS(quality3dmatrixes);
				R = quality_transform.a;
				T = quality_transform.b;
				scaleFactor = quality_transform.c;
				calculateError(complete3dmatrixes);
				break;
			}
			counter ++;
			if (counter > 300)
			{
				Log.i(TAG, "RANSAC reached limit- calculating on full set");
				Triplet<Matrix, Matrix, Double> quality_transform = getRTS(complete3dmatrixes);
				R = quality_transform.a;
				T = quality_transform.b;
				scaleFactor = quality_transform.c;
				calculateError(complete3dmatrixes);
				break;
			}
			//else
				//Log.i(TAG, "Continuing RANSAC iterations, so far = " + counter);
		}
	}

	/**
	 * Construct a camera coordinates 3d points matrix. 
	 * @param interest_points
	 */
	public Matrix getCameraPointsMat(List<Point2d> pixels, Matrix cam_mat) {
		Matrix result = new Matrix(3, pixels.size());
		List<Point3d> points3d = VisionGeoTransforms.pixel2camera(pixels, cam_mat);
		int i = 0;
		for (Point3d point : points3d) {
			result.set(0, i, point.x);
			result.set(1, i, point.y);
			result.set(2, i, point.z);
			i++;
		}
		return result;
	}

	/**
	/* Construct a pair of camera coordinates 3d points matrix. 
	 * 
	 * @param pairedList
	 * @param cam_mat
	 * @return
	 */

	public Pair<Matrix, Matrix> get2CameraPointsMats(List<Pair<Point2d, Point2d>> pairedList, Matrix cam_mat) {
		List<Point2d> alist = new ArrayList<Point2d>();
		List<Point2d> blist = new ArrayList<Point2d>();
		VisionGeoTransforms.unPairLists(pairedList, alist, blist);
		Matrix mat1 = getCameraPointsMat(alist, cam_mat);
		Matrix mat2 = getCameraPointsMat(blist, cam_mat);
		Pair<Matrix, Matrix> matrixes = new  Pair<Matrix, Matrix>();
		matrixes.set(mat1, mat2);
		return matrixes;
	}


	/**
	 * Get the error of the projection of model1's coordinates in model2's coordinate frame.
	 * 
	 * @return the sum squared error
	 */
	public double getError() {
		return error;
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

	/**
	 * Calculates error
	 * 
	 * @return error (double)
	 */
	private void calculateError(Pair<Matrix, Matrix> model_points_sets) {
		Matrix pts2est = R.times(model_points_sets.a).times(scaleFactor);
		for (int i = 0; i < pts2est.getColumnDimension(); i++) {
			pts2est.set(0, i, pts2est.get(0, i) + T.get(0, 0));
			pts2est.set(1, i, pts2est.get(1, i) + T.get(1, 0));
			pts2est.set(2, i, pts2est.get(2, i) + T.get(2, 0));
		}
		error = 0;
		for (int i = 0; i < pts2est.getColumnDimension(); i++) {
			error += Math.pow(pts2est.get(0, i) - model_points_sets.b.get(0, i), 2);
			error += Math.pow(pts2est.get(1, i) - model_points_sets.b.get(1, i), 2);
			error += Math.pow(pts2est.get(2, i) - model_points_sets.b.get(2, i), 2);
		}
	}

	/** get R and T pair from two model points sets + scale_factor
	 * 
	 * @param model_points_sets
	 * @return
	 */
	private Triplet<Matrix, Matrix, Double> getRTS(Pair<Matrix, Matrix> model_points_sets){
		Matrix R_ret = null;
		Matrix T_ret = null;
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

	/** TODO: explain...*/
	public List<Integer> TransformQuality(Triplet<Matrix, Matrix, Double> chosenRTS, Triplet<Matrix, Matrix, Double> refernceRTS, Pair<Matrix, Matrix> model_points_sets) {
		List<Integer> good_indexes = new ArrayList<Integer>();
		for (int i = 0; i < model_points_sets.a.getColumnDimension(); i++){
			Matrix pt2est_chosen = (chosenRTS.a).times(model_points_sets.a.getMatrix(0, 2, i, i)).times(chosenRTS.c);
			Matrix pt2est_reference = (refernceRTS.a).times(model_points_sets.a.getMatrix(0, 2, i, i)).times(refernceRTS.c);
			for (int j = 0; j < 3; j++)
			{
				pt2est_chosen.set(j, 0, pt2est_chosen.get(j, 0) + chosenRTS.b.get(j, 0));
				pt2est_reference.set(j, 0, pt2est_reference.get(j, 0) + refernceRTS.b.get(j, 0));
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
}

//        /**
//         * Translates the points in model 1 into the second model's coordinate frame.
//         * 
//         * @return the first model's features mapped into the second model's coordinate frame.
//         */
//        public Map<Feature, Point3d> adjustModel1() {
//                Map<Feature, Point3d> map = new HashMap<Feature, Point3d>();
//                Matrix p = new Matrix(3, 1);
//                for (Entry<Feature, Point3d> e : model1.getPointMap().entrySet()) {
//                        p.set(0, 0, e.getValue().x);
//                        p.set(1, 0, e.getValue().y);
//                        p.set(2, 0, e.getValue().z);
//                        p = R.times(p).times(scaleFactor).plus(T);
//                        map.put(e.getKey(), new Point3d(p.get(0, 0), p.get(1, 0), p.get(2, 0)));
//                }
//                return map;
//        }
//
//        /**
//         * Translates the points in model 2 into the first model's coordinate frame.
//         * 
//         * @return the second model's features mapped into the first model's coordinate frame.
//         */
//        public Map<Feature, Point3d> adjustModel2() {
//                Map<Feature, Point3d> map = new HashMap<Feature, Point3d>();
//                Matrix p = new Matrix(3, 1);
//                Matrix r = R.inverse().times(1 / scaleFactor);
//                for (Entry<Feature, Point3d> e : model2.getPointMap().entrySet()) {
//                        p.set(0, 0, e.getValue().x);
//                        p.set(1, 0, e.getValue().y);
//                        p.set(2, 0, e.getValue().z);
//                        p = r.times(p.minus(T));
//                        map.put(e.getKey(), new Point3d(p.get(0, 0), p.get(1, 0), p.get(2, 0)));
//                }
//                return map;
//        }
