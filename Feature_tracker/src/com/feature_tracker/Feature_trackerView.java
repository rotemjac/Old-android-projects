package com.feature_tracker;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.KeyPoint;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.video.*;
import org.opencv.imgproc.*;

import Jama.Matrix;
import android.content.Context;
import android.graphics.Bitmap;
import android.hardware.SensorEventListener;
import android.util.Log;


public class Feature_trackerView extends CvViewBase {

	/* Initialize all members*/
	Mat prevDescriptors = new Mat();
	Mat grayMat = new Mat(640, 480, CvType.CV_8UC1);
	Mat rgbMat = new Mat(640, 480, CvType.CV_8UC3);
	List<KeyPoint> prevKeypoints = new ArrayList<KeyPoint>();
	FeatureDetector fastDetector = FeatureDetector.create(FeatureDetector.FAST);
	FeatureDetector siftDetector = FeatureDetector.create(FeatureDetector.SIFT);
	FeatureDetector orbDetector = FeatureDetector.create(FeatureDetector.ORB);
	FeatureDetector surfDetector = FeatureDetector.create(FeatureDetector.SURF);
	FeatureDetector testDetector = FeatureDetector.create(FeatureDetector.GRID_SURF);
	String fastDetectorConfig = new String("%YAML:1.0\nthreshold: 40\nnonmaxSuppression: 1\n");
	String surfDetectorConfig = new String("%YAML:1.0\nhessianThreshold: 500.\noctaves: 3\noctaveLayers: 2\nupright: 1\n");//assume horizontality due to sensor data
	Rect ROI  = new Rect(20, 50, 80, 50);
	Rect staticRect = new Rect(600, 400, 100, 100); 
	Point3d BULLS3D = new Point3d();
	DescriptorExtractor myDE = DescriptorExtractor.create(DescriptorExtractor.SURF);
	DescriptorMatcher dm = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE);
	boolean prevKptsFound = false;
	Camera_specs Camera_details = null;
	public  Sensors_data  acc_data= new Sensors_data();
	final String TIMES = "debug_times";
	final String RESULTS = "debug_results";
	SensorEventListener listener;


	public Feature_trackerView(Context context) {
		super(context);
	}

	protected Bitmap processFrame (VideoCapture capture) {
		/*initialize locals*/
		Mat query_descriptors = new Mat();
		long tic = 0;
		long toc = 0;
		Bitmap bmp = null;
		List<KeyPoint> my_keypoints = new ArrayList<KeyPoint>();
		if (!capture.grab()) {
			Log.e("error", "mCamera.grab() failed");
			return null;
		}
		capture.retrieve(grayMat, Highgui.CV_CAP_ANDROID_GREY_FRAME);
		capture.retrieve(rgbMat, Highgui.CV_CAP_ANDROID_COLOR_FRAME_RGBA);
		tic = System.nanoTime();
		surfDetector.detect(grayMat, my_keypoints);
		toc  = System.nanoTime();
		Log.i(TIMES, "Detection task:" + (toc-tic)*0.000001);
		Log.i(RESULTS, "kpts found = :" + my_keypoints.size());

		if (0 != my_keypoints.size() && prevKptsFound)
		{
			List<List<DMatch>> matches_constrained = new ArrayList<List<DMatch>>();
			List<DMatch> matches = new ArrayList<DMatch>();
			List<Point2d> srcs = new ArrayList<Point2d>();
			List<Point2d> dsts = new ArrayList<Point2d>();
			tic =System.nanoTime();
			myDE.compute(grayMat, my_keypoints, query_descriptors);
			toc  = System.nanoTime();
			Log.i(TIMES, "descripting task:" + (toc-tic)*0.000001);
			float radius = 10;
			tic =System.nanoTime();
			dm.radiusMatch(query_descriptors, prevDescriptors, matches_constrained, radius);
			toc  = System.nanoTime();
			Log.i(TIMES, "matching task 1:" + (toc-tic)*0.000001);
			tic =System.nanoTime();
			dm.match(query_descriptors, prevDescriptors, matches);
			toc  = System.nanoTime();
			Log.i(TIMES, "matching task 2:" + (toc-tic)*0.000001);
			VisionGeoTransforms.supressWeakMatches(matches, 2);/*turn on to suppress wick/distinct matches*/
			/*TODO: no point in working with Point2d- Point is just fine- remove unnecessary conversions*/
			VisionGeoTransforms.stripCorrespondingJavaPoints(matches, my_keypoints, prevKeypoints, dsts, srcs);
			//List<Byte> status = new ArrayList<Byte>();
			//List<Float> err = new ArrayList<Float>();
			//			Imgproc.goodFeaturesToTrack(grayMat, dsts,50, 10, 10);
			//			Video.calcOpticalFlowPyrLK(grayMat, grayMat, srcs, srcs, status , err);
			//Log.i(TIMES, "good features + optical flow" + (toc-tic)*0.000001);


			//toc  = System.nanoTime();
			//Log.i(TIMES, "matching task:" + (toc-tic)*0.000001);

			if (srcs.size() >= 4 ) {
				//tic =System.nanoTime();
				//Log.i(RESULTS, "matches found = :" + srcs.size());
				Abs_orientation absor_wrapped = new Abs_orientation(200, 4, 8, VisionGeoTransforms.pairUplists(srcs, dsts), Camera_details.getIntrinsics());
				//toc  = System.nanoTime();
				//Log.i(TIMES, "ransac took [msec]" + (toc-tic)*0.000001);
				Matrix temp_mat;
				temp_mat = ((absor_wrapped.sol.a).times(MatrixUtils.point3dTtomatrix(BULLS3D))).times(absor_wrapped.sol.c).plus(absor_wrapped.sol.b);
				BULLS3D = MatrixUtils.matrixTopoint3d(temp_mat);
				poI = VisionGeoTransforms.camera2pixel(BULLS3D, Camera_details.getIntrinsics());
			}
			bmp = Bitmap.createBitmap(rgbMat.cols(), rgbMat.rows(), Bitmap.Config.ARGB_8888);
			/*store for next match*/
			prevDescriptors = query_descriptors;
			prevKeypoints = my_keypoints;
		}
		else if (0 != my_keypoints.size())
		{
			prevKptsFound = true;
			myDE.compute(grayMat, my_keypoints, query_descriptors);
			bmp = Bitmap.createBitmap(rgbMat.cols(), rgbMat.rows(), Bitmap.Config.ARGB_8888);
			/*store for next match*/
			prevDescriptors = query_descriptors;
			prevKeypoints = my_keypoints;
		}
		else
		{
			prevKptsFound = false;
			bmp = Bitmap.createBitmap(rgbMat.cols(), rgbMat.rows(), Bitmap.Config.ARGB_8888);
		}
		if (touched){
			//BULLS3D = VisionGeoTransforms.pixel2camera(poI, Camera_details.getIntrinsics());
			BULLS3D = VisionGeoTransforms.pixel2Reversedcamera(poI, Camera_details.getIntrinsics(), 480);
			
			prevKptsFound = false;
			synchronized (my_keypoints) {
				touched = false;
			}
		}
		if (Utils.matToBitmap(rgbMat, bmp))
			return bmp;
		bmp.recycle();
		return null;
	}

	/** just put a captured image on a bitmap*/
	protected Bitmap sensortrack(VideoCapture capture){
		Bitmap bmp;
		Sensors_data.startListening(listener);
		if (!capture.grab()) {
			Log.e("error", "mCamera.grab() failed");
			return null;
		}
		capture.retrieve(rgbMat, Highgui.CV_CAP_ANDROID_COLOR_FRAME_BGRA);
		bmp = Bitmap.createBitmap(rgbMat.cols(), rgbMat .rows(), Bitmap.Config.ARGB_8888);
		Matrix acc_mat = Sensors_data.INV_mMatrix_R; 
		Sensors_data.stopListening();
		Matrix temp_mat = acc_mat.times(MatrixUtils.point3dTtomatrix(BULLS3D));
		Point3d temp_point = MatrixUtils.matrixTopoint3d(temp_mat);
		poI = VisionGeoTransforms.camera2pixel(temp_point, Camera_details.getIntrinsics());
		poI.set(poI.x, 480 - poI.y);
		if (Utils.matToBitmap(rgbMat, bmp))
			return bmp;
		bmp.recycle();
		return null;
	}


	/**This function is used to config the detector- string compatibility to the certain detector is on the user */
	private static void paramsToDetectorWrkAround(FeatureDetector detector, String param_string){	
		File file = new File("data/data/com.feature_tracker/test11.txt");
		if (!file.exists()) {
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
			OutputStream fo;
			try {
				fo = new FileOutputStream(file);
				OutputStreamWriter osw = new OutputStreamWriter(fo);
				try {
					osw.write(param_string);
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				try {
					osw.flush();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				try {
					osw.close();
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} catch (FileNotFoundException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}     
			detector.read("data/data/com.feature_tracker/test11.txt");
			file.delete();
		}
	}

	@Override
	protected Point2d getPOI() {
		return poI;
	}


	@Override
	public void run() {
		paramsToDetectorWrkAround(fastDetector, fastDetectorConfig);
		paramsToDetectorWrkAround(surfDetector, surfDetectorConfig);
		listener= Sensors_data.mySensorListener;
		/*TODO: get rid of this stupid camera class*/
		InputStream dist = getResources().openRawResource(R.raw.dist_coeff);
		InputStream intr = getResources().openRawResource(R.raw.intrinsic_params);
		try {
			Camera_details = new Camera_specs(intr, dist);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			Camera_details = null;
			e.printStackTrace();
		}
		poI = new Point2d(300, 300);
		//BULLS3D = VisionGeoTransforms.pixel2camera(poI, Camera_details.getIntrinsics());
		BULLS3D = VisionGeoTransforms.pixel2Reversedcamera(poI, Camera_details.getIntrinsics(), 480);
		Sensors_data.mMatrix_R.set(0, 0, 1);
		Sensors_data.mMatrix_R.set(1, 1, 1);
		Sensors_data.mMatrix_R.set(2, 2, 1);
		super.run();
	}

	@Override
	protected void ptpincher(int numberofpoints) {
		for (int i=0; i<numberofpoints; i++)
		{
			//			while (!touch_buffer){}
			//			poI.add(i,new Point2d(touchx, touchy));
			//			BULLS3D.add(i, VisionGeoTransforms.pixel2camera(poI.get(i), Camera_details.getIntrinsics()));
			//			//			try {
			//			//				this.wait(500);
			//			//			} catch (InterruptedException e) {
			//			//				// TODO Auto-generated catch block
			//			//				e.printStackTrace();
			//			//			}
			//			touch_buffer = false;
		}
	}

	@Override
	protected Point2d rectifypoi(List<Point2d> src, List<Point2d> dst,Point2d poi) {
		AbsoluteOrientation absor = new AbsoluteOrientation(VisionGeoTransforms.pairUplists(src, dst), Camera_details.getIntrinsics());
		Point3d temp3d = VisionGeoTransforms.pixel2camera(poi, Camera_details.getIntrinsics());
		Log.i("debug_results", "Rabsor:\n" + MatrixUtils.matrixToString(absor.R));
		Matrix temp_mat = ((absor.R).times(MatrixUtils.point3dTtomatrix(temp3d))).plus(absor.T);
		temp3d = MatrixUtils.matrixTopoint3d(temp_mat);
		Point2d temp2 = VisionGeoTransforms.camera2pixel(temp3d, Camera_details.getIntrinsics());
		poi.set(temp2);
		return poi;
	}
}
