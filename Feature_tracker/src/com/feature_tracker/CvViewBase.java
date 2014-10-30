package com.feature_tracker;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Size;
import org.opencv.highgui.VideoCapture;
import org.opencv.highgui.Highgui;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Canvas.VertexMode;
import android.graphics.Paint;
import android.hardware.SensorEventListener;
import android.util.Log;
import android.util.Pair;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.widget.TextView;

public abstract class CvViewBase extends SurfaceView implements
SurfaceHolder.Callback, Runnable {
	private static final String TAG = "CvViewBase";

	private SurfaceHolder       mHolder;
	private VideoCapture        mCamera;/*maybe CVcapture type will be faster*/
	Paint my_paint = new Paint();
	boolean touched = false;  
	int debug_step = 0;
	Point2d poI  = null;
	final boolean isInDebug = false;

	@Override
	public boolean onTouchEvent(MotionEvent e)
	{	
		float touchx = e.getX();
		float touchy = e.getY();
		synchronized (this){
			poI = new Point2d(touchx, touchy);
		}//TODO: stop all calculations
		synchronized (poI) {
			touched = true;
		}
		return true;
	}

	public CvViewBase(Context context) {
		super(context);
		mHolder = getHolder();
		mHolder.addCallback(this);
		Log.i(TAG, "Instantiated new " + this.getClass());
	}

	public void surfaceChanged(SurfaceHolder _holder, int format, int width, int height) {
		Log.i(TAG, "surfaceCreated");
		synchronized (this) {
			if (mCamera != null && mCamera.isOpened()) {
				Log.i(TAG, "before mCamera.getSupportedPreviewSizes()");
				List<Size> sizes = mCamera.getSupportedPreviewSizes();
				Log.i(TAG, "after mCamera.getSupportedPreviewSizes()");
				int mFrameWidth = width;
				int mFrameHeight = height;

				// selecting optimal camera preview size
				{
					double minDiff = Double.MAX_VALUE;
					for (Size size : sizes) {
						if (Math.abs(size.height - height) < minDiff) {
							mFrameWidth = (int) size.width;
							mFrameHeight = (int) size.height;
							minDiff = Math.abs(size.height - height);
						}
					}
				}

				mCamera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, mFrameWidth);
				mCamera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, mFrameHeight);
			}
		}
	}

	public void surfaceCreated(SurfaceHolder holder) {
		Log.i(TAG, "surfaceCreated");
		my_paint.setARGB(255, 80, 150, 80);
		my_paint.setTextSize(20);
		mCamera = new VideoCapture(Highgui.CV_CAP_ANDROID);
		if (mCamera.isOpened()) {
			(new Thread(this)).start();
		} else {
			mCamera.release();
			mCamera = null;
			Log.e(TAG, "Failed to open native camera!!!");
		}
	}

	public void surfaceDestroyed(SurfaceHolder holder) {
		Log.i(TAG, "surfaceDestroyed");
		if (mCamera != null) {
			synchronized (this) {
				mCamera.release();
				mCamera = null;
			}
		}
	}

	protected abstract Bitmap processFrame(VideoCapture capture);
	protected abstract Bitmap sensortrack(VideoCapture capture);
	protected abstract Point2d getPOI();
	protected abstract void ptpincher(int numberofpoints);
	protected abstract Point2d rectifypoi(List<Point2d> src, List<Point2d> dst, Point2d poi);


	public void run() {
		Log.i(TAG, "Starting processing thread");
		long toc = 0;
		long tic = 0;
//		synchronized (this) {
//			if (!mCamera.grab()) {
//				Log.e(TAG, "mCamera.grab() failed");
//			}
//		}
//		Bitmap bmp_temp = getnaturalframe(mCamera);
//		drawoncanvas(bmp_temp, null, "Touch desired POI's", my_paint, 3);
//		ptpincher(3);
//		bmp_temp.recycle();
		while (true) {
			Bitmap bmp = null;
			synchronized (this) {
				if (mCamera == null)
					break;

//				if (!mCamera.grab()) {
//					Log.e(TAG, "mCamera.grab() failed");
//					break;
//				}
			}

			//if (!isInDebug)
			//{
			//bmp = processFrame(mCamera);
			bmp = sensortrack(mCamera);
			toc = System.nanoTime();
			
			Log.i("debug_times", "time full loop im msecs:" + (toc-tic)*0.000001);
			tic  = System.nanoTime();
			List<Point2d> pp = new ArrayList<Point2d>();
			//pp.add(getPOI());
			drawoncanvas(bmp, poI, "Estimated error [pixels] = " + AbsoluteOrientation.error*800, my_paint, 15);
			
		}
		Log.i(TAG, "Finishing processing thread");
	}

	protected void onDraw(Canvas canvas) { 
		// TODO Auto-generated method stub 
		Paint paint = new Paint(); 
		paint.setStyle(Paint.Style.FILL); 
		paint.setTextSize(30);
		paint.setColor(Color.BLACK); 
		canvas.drawCircle(700, 450, 20, paint);
		super.onDraw(canvas); 
	} 
	private void drawoncanvas(Bitmap bmp, Point2d point, String text, Paint my_paint, float radius){
		if (bmp != null) {
			Canvas canvas = mHolder.lockCanvas();
			if (canvas != null) {
				bmp = Bitmap.createScaledBitmap(bmp, canvas.getWidth(), canvas.getHeight(), false);   
				canvas.drawBitmap(bmp, 0, 0, null);
				my_paint.setARGB(255, 150, 80, 150);
				canvas.drawText(text, 100, 100, my_paint);
				my_paint.setARGB(255, 80, 150, 80);
				if (point != null){
					canvas.drawCircle((float)point.x,(float)point.y, radius, my_paint);
				}
				mHolder.unlockCanvasAndPost(canvas);
			}
			bmp.recycle();
		}
	}
}