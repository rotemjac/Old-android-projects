package com.feature_tracker;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

public class Ess_matrix8pt extends RANSAC_wrapper<Matrix, Pair<Point2d, Point2d>, Double>  {

	Camera_specs cam_specs = null;
	private Pair<Matrix, Matrix> constraints3D = new Pair<Matrix, Matrix>();
			Ess_matrix8pt(int iterations_limit, int modelset_size, int inliers_setsize,
					List<Pair<Point2d, Point2d>> all_constraints, Camera_specs camera) {
		super(iterations_limit, modelset_size, inliers_setsize, all_constraints);
		cam_specs = camera;

		RANSAC_loop();
	}

	@Override
	Matrix solver(List<Pair<Point2d, Point2d>> constraints) {
		constraints3D = VisionGeoTransforms.HomoganizePointsPairs(cam_specs, constraints); 
		normalize(constraints3D.a, cam_specs.getInvIntrinsics());
		normalize(constraints3D.b, cam_specs.getInvIntrinsics());
		Matrix essentialMatrix = new Matrix(3, 3);
		final Matrix a = buildA(constraints3D.a, constraints3D.b);
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
		return essentialMatrix;
		//         SingularValueDecomposition svd = new SingularValueDecomposition(essentialMatrix);
		//         Matrix s = svd.getS();
		//         System.out.println("S:\n" + MatrixUtils.matrixToString(s));
		//         s.set(2, 2, 0);
		//         refinedEssentialMatrix = svd.getU().times(s).times(svd.getV().transpose());
		//
		//         refinedError = getError(refinedEssentialMatrix, imgpts1, imgpts2
	}

	@Override
	List<Integer> solutionQuality(Matrix chosen_solution, Matrix reference_solution) {
		List<Integer> good_indexes = new ArrayList<Integer>();
		Pair<Matrix, Matrix> model_points_sets = VisionGeoTransforms.get2CameraPointsMats(allConstraints, cam_specs.getIntrinsics());
		RotationTranslationEstimator RT_chosen = new RotationTranslationEstimator(chosen_solution); 
		Matrix T_chosen = RT_chosen.getT();
		Matrix R_chosen = RT_chosen.getR1();
		RotationTranslationEstimator RT_ref = new RotationTranslationEstimator(reference_solution);
		Matrix T_ref = RT_ref.getT();
		Matrix R_ref = RT_ref.getR1();
		for (int i = 0; i < model_points_sets.a.getColumnDimension(); i++){
			Matrix pt2est_chosen = (R_chosen.times(model_points_sets.a.getMatrix(0, 2, i, i)));
			Matrix pt2est_reference = (R_ref.times(model_points_sets.a.getMatrix(0, 2, i, i)));
			for (int j = 0; j < 3; j++)
			{
				pt2est_chosen.set(j, 0, pt2est_chosen.get(j, 0) + T_chosen.get(j, 0));
				pt2est_reference.set(j, 0, pt2est_reference.get(j, 0) + T_ref.get(j, 0));
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
		double val = 0;
		for (int i = 0; i < (constraints3D.a).getColumnDimension(); i++) {
			Matrix pt2 = (constraints3D.b).getMatrix(0, 2, i, i).transpose();
			Matrix pt1 = (constraints3D.a).getMatrix(0, 2, i, i);

			Matrix pt2timesE = pt2.times(sol);

			val += Math.pow(pt2timesE.times(pt1).get(0, 0), 2);
		}
		estError =  val;
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

}
