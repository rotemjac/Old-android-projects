package com.feature_tracker;

import Jama.Matrix;


public class AlternativeTransformations {

	/**affine 3d...stuck!*/
	//	List<Point3d> dsts3D = VisionGeoTransforms.pixel2camera(dsts, Camera_details.getIntrinsics());
	//	List<Point3d> srcs3D = VisionGeoTransforms.pixel2camera(srcs, Camera_details.getIntrinsics());
	//	Mat affine = new Mat(3, 4, CvType.CV_32FC1);
	//	Mat inliers = new Mat(CvType.CV_32FC3);
	//	Mat Rot = new Mat(3, 3, CvType.CV_32FC1);
	//	Mat Trans = new Mat(1, 4, CvType.CV_32FC1);
	//	Calib3d.estimateAffine3D(VisionGeoTransforms.Point3dsettoMat(srcs3D), VisionGeoTransforms.Point3dsettoMat(dsts3D), affine, inliers);
	//	Calib3d.decomposeProjectionMatrix(affine, MatrixUtils.matrixtoCVmat(Camera_details.getIntrinsics()), Rot, Trans);
	//	Matrix temp_mat = (MatrixUtils.CVmattoMatrix(Rot).times(MatrixUtils.point3dTtomatrix(BULLS3D))).plus(MatrixUtils.CVmattoMatrix(Trans.colRange(0, 2)));
	//	Log.i(RESULTS, "Rabsor:\n" + MatrixUtils.matrixToString(MatrixUtils.CVmattoMatrix(Rot)));
	//	Log.i(RESULTS, "Tabsor:\n" + MatrixUtils.matrixToString(MatrixUtils.CVmattoMatrix(Trans.colRange(0, 2))));

	/**Essential Matrix- add Ransac- could be good*/
	//Essential_Matrix_Est my_Essential_Matrix_Est = new Essential_Matrix_Est(my_points, prev_points, Camera_details);
	//Log.i(TIMES, "time essential im msecs:" + (toc-tic)*0.000001);
	//matrix essential_mat = my_Essential_Matrix_Est.getEssentialMatrix();
	//RotationTranslationEstimator my_RotationTranslationEstimator = new RotationTranslationEstimator(essential_mat); 
	//matrix T = my_RotationTranslationEstimator.getT();
	//matrix R1 = my_RotationTranslationEstimator.getR1();
	//matrix R2 = my_RotationTranslationEstimator.getR2();
	//	Log.i(RESULTS, "essential:\n" + MatrixUtils.matrixToString(essential_mat));
	//				Log.i(RESULTS, "translation:\n" + MatrixUtils.matrixToString(T));
	//Log.i(RESULTS, "R1:\n" + MatrixUtils.matrixToString(R1));
	//Log.i(RESULTS, "R2:\n" + MatrixUtils.matrixToString(R2));
	
	/**essmartix ransac- bad- maybe fix**/
//	Ess_matrix8pt Ess_ransac = new Ess_matrix8pt(400, 8, 12, VisionGeoTransforms.pairUplists(srcs, dsts), Camera_details);
//	RotationTranslationEstimator my_RotationTranslationEstimator = new RotationTranslationEstimator(Ess_ransac.sol); 
//	Matrix T = my_RotationTranslationEstimator.getT();
//	Matrix R1 = my_RotationTranslationEstimator.getR1();
//	temp_mat = ((R1).times(MatrixUtils.point3dTtomatrix(BULLS3D.get(i)))).plus(T);
	
	/** absor not wrapped- delete?*/
	//AbsoluteOrientation absor = new AbsoluteOrientation(VisionGeoTransforms.pairUplists(srcs, dsts), Camera_details.getIntrinsics());
	//temp_mat = ((absor.R).times(MatrixUtils.point3dTtomatrix(BULLS3D.get(i)))).times(absor.scaleFactor).plus(absor.T);
}
