package com.feature_tracker;

import Jama.Matrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Vector;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.KeyPoint;

import android.util.Log;

//TODO: consider making this not final to avoid code duplication
final class VisionGeoTransforms {

	static String TAG = "VisionGeoTransforms";

	/** this function transforms a pixel 2d coordinate to it's corresponding camera coordinates
	 * 
	 * @param pixels
	 * @param intrinsics
	 * @return
	 */
	public static List<Point3d> pixel2camera ( List<Point2d> pixels, Matrix intrinsics){
		List<Point3d> cam_cords = new ArrayList<Point3d>();
		double cx = intrinsics.get(0, 2);
		double cy = intrinsics.get(1, 2);
		double fx = intrinsics.get(0, 0);
		double fy = intrinsics.get(1, 1);
		for (int i = 0; i <pixels.size() ;i++) {
			double xfactor = (pixels.get(i).x-cx)/fx;
			double yfactor = (pixels.get(i).y-cy)/fy;
			double zc = 1/(Math.sqrt(xfactor*xfactor + yfactor*yfactor + 1));
			cam_cords.add(new Point3d(xfactor*zc,yfactor*zc,zc));
		}
		return cam_cords;
	}
	/** explain...
	 * 
	 * @param points
	 * @param mat
	 * @return
	 */
	public static Mat Point3dsettoMat(List<Point3d> points){
		Mat mat = new Mat(1,points.size(),CvType.CV_32FC3);
		for(int i = 0; i < points.size(); i++)
		{
			double[] data = {points.get(i).x, points.get(i).x, points.get(i).x};
	    	mat.put(i, 0, data);
		}
		return mat;
	}

	/** this function transforms a pixel 2d coordinate to it's corresponding camera coordinates- 1 point version
	 * 
	 * @param pixel
	 * @param intrinsics
	 * @return
	 */
	public static Point3d pixel2camera ( Point2d pixel, Matrix intrinsics){
		Point3d cam_cords = new Point3d();
		double cx = intrinsics.get(0, 2);
		double cy = intrinsics.get(1, 2);
		double fx = intrinsics.get(0, 0);
		double fy = intrinsics.get(1, 1);
		double xfactor = (pixel.x-cx)/fx;
		double yfactor = (pixel.y-cy)/fy;
		double zc = 1/(Math.sqrt(xfactor*xfactor + yfactor*yfactor + 1));
		cam_cords.set(xfactor*zc,yfactor*zc,zc);
		return cam_cords;
	}
	/** this function transforms a pixel 2d coordinate to it's corresponding camera coordinates with revesed t coordinates- 1 point version
	 * 
	 * @param pixel
	 * @param intrinsics
	 * @return
	 */
	public static Point3d pixel2Reversedcamera ( Point2d pixel, Matrix intrinsics, float frame_height){
		Point3d cam_cords = new Point3d();
		double cx = intrinsics.get(0, 2);
		double cy = intrinsics.get(1, 2);
		double fx = intrinsics.get(0, 0);
		double fy = intrinsics.get(1, 1);
		double xfactor = (pixel.x-cx)/fx;
		double yfactor = (frame_height - pixel.y-cy)/fy;
		double zc = 1/(Math.sqrt(xfactor*xfactor + yfactor*yfactor + 1));
		cam_cords.set(xfactor*zc,yfactor*zc,zc);
		return cam_cords;
	}
	
	
	/** this function transforms a 3d camera coordinate to corresponding pixel 2d coordinate - 1 point version
	 * 
	 * @param cam_cords
	 * @param intrinsics
	 * @return
	 */
	public static Point2d camera2pixel ( Point3d cam_cords, Matrix intrinsics){
		double cx = intrinsics.get(0, 2);
		double cy = intrinsics.get(1, 2);
		double fx = intrinsics.get(0, 0);
		double fy = intrinsics.get(1, 1);
		Point2d pixel = new Point2d();
		pixel.set((fx*cam_cords.x)/(cam_cords.z) + cx, (fy*cam_cords.y)/cam_cords.z + cy);
		return pixel;
	}
	/**This function adds small random noise to 2d measurements and optionally add big noise to outlier_rate of measurements
	 * 
	 * @param points
	 * @param outlier_rate
	 * @param small_noise_range
	 * @param big_noise_range
	 */
	public static void addnoise(List<Point2d> points, double outlier_rate, int small_noise_range, int big_noise_range){
		Random rand = new Random();
		int outliers = (int) (points.size()*outlier_rate);
		for (int i=0; i < points.size() - outliers; i++ ){
			Point2d temp = points.get(i);
			temp.set(temp.x + rand.nextDouble()*small_noise_range, temp.y + rand.nextDouble()*small_noise_range);
			points.set(i, temp);
		}
		for (int i=points.size() - outliers; i < points.size(); i++)
		{
			Point2d temp = points.get(i);
			temp.set(temp.x + rand.nextDouble()*big_noise_range, temp.y + rand.nextDouble()*big_noise_range);
			points.set(i, temp);

		}
	}

	/**This function is used to suppress matches with factor times greater distance than the closest match.
	/* Done mainly to save on later computations- so can be discarded if doesn't improve efficiency
	 * 
	 * @param matches
	 * @param factor
	 */
	public static void supressWeakMatches(List<DMatch> matches, double factor){
		float min_dist = 100;
		float temp_dist;
		/*find minimum and suppress some on the fly*/
		/* TODO: change these ugly for loops to nice ones */
		for( int i = 0; i < matches.size(); i++ ){ 
			temp_dist = matches.get(i).distance;
			if( temp_dist < min_dist ) 
				min_dist = temp_dist;
			else if (temp_dist > min_dist*factor)
			{
				matches.remove(i);
				i--;
			}
		}
		/*suppress the rest*/
		for (int i = 0;i < matches.size();i++){
			DMatch current_match = matches.get(i);
			if (current_match.distance > min_dist*factor)
			{
				matches.remove(i);
				i--;
			}
		}
	}

	/**This function strips corresponding OCV points lists from given matches and the two keypoints sets
	 * 
	 * @param matches
	 * @param query_keypoints
	 * @param train_keypoints
	 * @param query_points
	 * @param train_points
	 */
	public static void stripCorrespondingOCVPoints(List<DMatch> matches, List<KeyPoint> query_keypoints, List<KeyPoint> train_keypoints, List<Point> query_points, List<Point> train_points ){
		for( int i = 0; i < matches.size(); i++ ){
			try {
				query_points.add(query_keypoints.get(matches.get(i).queryIdx).pt);
				train_points.add(train_keypoints.get(matches.get(i).trainIdx).pt);
			} catch (IndexOutOfBoundsException e) {
				e.getCause();
			}
		}
	}

	/**This function strips corresponding JAVA points lists from given matches and the two keypoints sets
	 * 
	 * @param matches
	 * @param query_keypoints
	 * @param train_keypoints
	 * @param query_points
	 * @param train_points
	 */
	public static void stripCorrespondingJavaPoints(List<DMatch> matches, List<KeyPoint> query_keypoints, List<KeyPoint> train_keypoints, List<Point2d> query_points, List<Point2d> train_points ){
		for( int i = 0; i < matches.size(); i++ ){
			Point2d query = null;
			Point2d train = null;
			try {
				query = new Point2d(query_keypoints.get(matches.get(i).queryIdx).pt.x, query_keypoints.get(matches.get(i).queryIdx).pt.y);
				train = new Point2d(train_keypoints.get(matches.get(i).trainIdx).pt.x, train_keypoints.get(matches.get(i).trainIdx).pt.y);
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			query_points.add(query);
			train_points.add(train);
		}
	}
	/** A class to handle vector to Rect and vice versa conversions*/
	public static class Rect2Vect2Rect
	{
		/**This function get's a rectangle and returns a point vector containing it's 4 vertexes in order top-left->right->down->left
		 * 
		 * @param rect
		 * @return
		 */
		public static Vector<Point> rect2vect(Rect rect){
			Vector<Point> vect = new  Vector<Point>(4);
			Point point1 = new Point();
			Point point2 = new Point();
			vect.add(0, rect.tl());
			point1.x = rect.x + rect.width;
			point1.y = rect.y;
			vect.add(1, point1);
			vect.add(2, rect.br());
			point2.x = rect.x;
			point2.y = rect.y + rect.height;
			vect.add(3, point2);
			return vect;
		}
		/**This function get's a vector of 4 points and returns a corresponding rectangle [implicit vertexes order: top-left->right->down->left]
		 * 
		 * @param vect
		 * @return
		 */
		public static Rect vect2rect(Vector<Point> vect){
			double x_cords[] = {vect.elementAt(0).x, vect.elementAt(1).x, vect.elementAt(2).x, vect.elementAt(3).x};
			double y_cords[] = {vect.elementAt(0).y, vect.elementAt(1).y, vect.elementAt(2).y, vect.elementAt(3).y};
			Arrays.sort(x_cords);  
			Arrays.sort(y_cords); 
			Rect rect = new Rect((int)x_cords[0], (int)y_cords[0], (int)(x_cords[3] - x_cords[0]), (int)(y_cords[3] - y_cords[0]));			
			// / TODO: check if the vector really spells a rectangle
			return rect;
		}
	}

	/** Point2d type suits HasCoordinates2d interface *
	 * 
	 * @param ocv_points
	 * @return
	 */
	public static List<Point2d> convertOcvPointToPoint2d(List<Point> ocv_points){
		List<Point2d> ret_points = new ArrayList<Point2d>();
		for (Point point : ocv_points){
			ret_points.add( new Point2d(point.x,point.y));
		}
		return ret_points;
	}
	/** gets two lists [explicitly corresponding] and returns a correspondence preserving list of pairs **/ 
	public static <A, B> List<Pair<A, B>> pairUplists(List<A> a, List<B> b){
		List<Pair<A, B>> pairedList = new ArrayList<Pair<A, B>>();
		/*Pair up the lists*/
		for (int i = 0; i < a.size(); i++) {
			Pair<A, B> pair = new Pair<A, B>();
			pair.set(a.get(i), b.get(i));
			pairedList.add(pair);
		}
		return pairedList;
	}

	/** gets a combined list [corresponding] and creates two explicitly corresponding singular lists
	 * 
	 * @param paired_list
	 * @param alist-assumed initialized & empty
	 * @param blist-assumed initialized & empty
	 */
	public static <A, B> void unPairLists(List<Pair<A, B>> paired_list, List<A> alist, List<B> blist){
		for (Pair<A, B> pair: paired_list) {
			alist.add(pair.a);
			blist.add(pair.b);
		}
	}

	/** Randomly break a list into a chosen and others
	 * 
	 * @param list
	 * @param chosen- assumed empty
	 * @param others- assumed empty
	 * @param chosen_size - if larger than list.size() - chosen will get all members
	 */
	public static <A> void chooseRandSublist(List<A> list, List<A> chosen, List<A> others, int chosen_size){
		long seed = System.nanoTime();
		Collections.shuffle(list, new Random(seed));
		int i = 1;
		for (A member: list){
			if (i > chosen_size){
				others.add(member);
			}
			else{
				chosen.add(member);
			}
			i++;
		}

	}
	/** get a sub list of certain indexes
	 * 
	 * @param list
	 * @param indexes- max value in this list cannot must not exceed (list.size()-1) 
	 * @return
	 */
	public static <A> List<A> pickSublist(List<A> list, List<Integer> indexes)
	{
		List<A> ret_list = new ArrayList<A>();
		try {
			for (Integer index: indexes){
				ret_list.add(list.get(index));
			}
		} catch (Exception e) {
			/*TODO: find out if I can put hands on index in this form*/
			Log.e(TAG, "pickSublist: Some Index exceeded list size = " + list.size() + " returning null");
			return null;
		}
		return ret_list;
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
    public static Matrix HomoganizePoints(Camera_specs c, List<Point2d> interest_points) {
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
     * Gets homogenized undistorted image points pairs.
     * 
     * @param c
     *            the camera to use for undistorting
     * @param pairs_list
     *            the corresponding pairs of points in pixels
     * @return a pair of 3x(interest_points.size()) Matrix on image plane with z=1 [this is the known (u,v,1) style]
     */
    public static Pair<Matrix, Matrix> HomoganizePointsPairs(Camera_specs c, List<Pair<Point2d, Point2d>> pairs_list) {
    		Pair<Matrix, Matrix> result = new Pair<Matrix, Matrix> ();
    		List<Point2d> alist = new ArrayList<Point2d>();
    		List<Point2d> blist = new ArrayList<Point2d>();
    		unPairLists(pairs_list, alist, blist);
    		result.a = HomoganizePoints(c, alist);
    		result.b = HomoganizePoints(c, blist);
            return result;
    }
    
    /**
	/* Construct a pair of camera coordinates 3d points matrix. 
	 * 
	 * @param pairedList
	 * @param cam_mat
	 * @return
	 */

	public static Pair<Matrix, Matrix> get2CameraPointsMats(List<Pair<Point2d, Point2d>> pairedList, Matrix cam_mat) {
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
	 * Construct a camera coordinates 3d points matrix. 
	 * @param interest_points
	 */
	public static Matrix getCameraPointsMat(List<Point2d> pixels, Matrix cam_mat) {
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
}


