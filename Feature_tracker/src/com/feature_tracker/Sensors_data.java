package com.feature_tracker;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.io.Reader;
import java.text.*;
import java.util.Arrays;
import java.util.List;
import java.text.FieldPosition;
import java.text.Format;
import java.text.ParsePosition;
import Jama.Matrix;

import android.app.Activity;
import android.content.Context;
import android.graphics.*;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;



//import android.hardware.Camera;
//import android.hardware.Camera.Parameters;
//import android.hardware.Camera.CameraInfo;
//import android.hardware.Camera.PreviewCallback;



public class Sensors_data  {


	private static final String rotem = "Sample::SurfaceView";

	private static SensorManager myManager;
	//private final SensorEventListener mySensorListener;

	private List<Sensor> accSensors,gyroSensors;
	public static Sensor accelerometer,gyro;
	private static float[]  accelerometervalues;
	//private float[] Linear_acc_values;
	//private float[] gyrovalues;
	//private double[] accelerometervalues_double;
	double  Rotation[] = new double[9];


	//private float[] Rx_rotate_acc=new float[9];
	//private float[] Rz_rotate_acc=new float[9];
	private static double[] R_rotate_acc=new double[9];

	double Rotation_gyro_double[] = new double[9];


	float I[] = new float[9];


	///////////Acc///////////////////////////////// 	 	
	static double  R_acc,Axr,Ayr,Azr;
	static double begin_Axr,begin_Ayr,begin_Azr;
	static double	overall_movement;

	static double	Rx_acc,Ry_acc,Rz_acc;

	static double begin_R_acc,begin_Rx_acc,begin_Ry_acc,begin_Rz_acc;

	//double last_Rx_acc=0,last_Rz_acc=0;

	
	
	static double diff_acc_x=0,diff_acc_y=0,diff_acc_z=0;
	
	static double diff_Axr_acc=0;
	
	static double diff_Ayr_acc=0;

	static double diff_Azr_acc=0;

	static double last_Rx_acc=0,last_Ry_acc=0,last_Rz_acc=0;
	static double diff_Rx_acc=100,diff_Ry_acc=100,diff_Rz_acc=100;
	
	double overall_acc_changed_x_pitch,overall_acc_changed_y,overall_acc_changed_z_roll;


	//double accResult[] = new double[4];
	static double counter_acc=0;
	
	///////////Linear Acc///////////////////////////////// 	 	
	//	double	 Linear_acc_x,Linear_acc_y,Linear_acc_z;
	
	//	double	begin_Linear_acc_x,begin_Linear_acc_y,begin_Linear_acc_z;
	//
	//	double	diff_Linear_acc_x=0,diff_Linear_acc_y=0,diff_Linear_acc_z=0;
	//
	//	double	counter_Linear_acc;

	///////////Gyro///////////////////////////////// 
	//	double	gyro_x,gyro_y,gyro_z;
	//	
	//	double	Gyro_xr,Gyro_yr,Gyro_zr;
	//	
	//	double begin_Rx_gyro=0,begin_Ry_gyro=0,begin_Rz_gyro=0;
	//	double overall_gyro_changed_x_pitch,overall_gyro_changed_y_yaw,overall_gyro_changed_z_roll;
	//	double gyroResult[] = new double[4]; 
	//	double resultVec_gyro[] = new double[4];
	//
	//	double counter_gyro=0;
	//	private static final float NS2S = 1.0f / 1000000000.0f;
	//	private float timestamp;
	//	
	//	float dT;
	//    double omegaMagnitude;
	//    double thetaOverTwo;

	/////////////////////////////////////////////////	 
	static double cos_Theta_x,sin_Theta_x,cos_Theta_y,sin_Theta_y,cos_Theta_z,sin_Theta_z;
	static double Cos_X,Cos_Y,Cos_Z;

	public static Matrix mMatrix_R=new Matrix(3,3);
	//public static double[][]gravitation_changed_array=new double [200][3];
	public static int gravitation_changed_array_counter=0;
	//public static double[][]angles_changed_array=new double [200][2];
	public static int angles_changed_array_counter=0;
	public static double[][]data_array=new double [200][19];
	public static Matrix INV_mMatrix_R=new Matrix(3,3);
	
	static public int k=0;
	static long time=0;
	static long begin_time=0;
	static long diff_time=0;
	
	public static int low_pass_counter=0;
	static public double Sum_Rx_acc=0,Sum_Ry_acc=0,Sum_Rz_acc=0;
	
	static double[] Array_x=new double [10];
	static double[] Array_y=new double [10];
	static double[] Array_z=new double [10];
	
	static double sum_array_x=0,sum_array_y=0,sum_array_z=0;
	
	static int counter_array=0;



	/** indicates whether or not Accelerometer Sensor is supported */
	private static Boolean supported;
	/** indicates whether or not Accelerometer Sensor is running */
	private static boolean running = false;

	/**
	 * Returns true if the manager is listening to orientation changes
	 */

	/* public static boolean isListening() {
        return running;
    }

	 *//**
	 * Unregisters listeners
	 */
	
    public static void stopListening() {
        running = false;
        try {
            if (myManager != null && mySensorListener != null) {
            	myManager.unregisterListener(mySensorListener);
            }
        } catch (Exception e) {}
    }

	  /**
	  * Returns true if at least one Accelerometer sensor is available
	  *//*
    public static boolean isSupported() {
        if (supported == null) {
            if (Feature_trackerActivity.getContext() != null) {
            	myManager = (SensorManager) Feature_trackerActivity.getContext().
                        getSystemService(Context.SENSOR_SERVICE);
                List<Sensor> sensors = myManager.getSensorList(
                        Sensor.TYPE_ACCELEROMETER);
                supported = new Boolean(sensors.size() > 0);
            } else {
                supported = Boolean.FALSE;
            }
        }
        return supported;
    }
	   */



	/**
	 * Registers a listener and start listening
	 * @param accelerometerListener
	 *             callback for accelerometer events
	 */
	public static void startListening(SensorEventListener  mySensorListener)

	{	
		myManager = (SensorManager) Feature_trackerActivity.getContext().
				getSystemService(Context.SENSOR_SERVICE);

		accelerometer = myManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		// gyro = myManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
		//Sensor Linear_accelerometer = myManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

		if (accelerometer != null  ) {
			running=myManager.registerListener(mySensorListener, accelerometer, SensorManager.SENSOR_DELAY_NORMAL);
			//myManager.registerListener(mySensorListener, gyro, SensorManager.SENSOR_DELAY_NORMAL);
			

		}

	}

	/////////////////////////////////////////////////////////////////////////////////////	

	public static  SensorEventListener mySensorListener = new SensorEventListener()
	{    

		public void onAccuracyChanged(Sensor sensor, int accuracy) {}

		public void onSensorChanged(SensorEvent event) {
			
			//double threshold=0.05;//IF WE WANT THRESHOLD
			
			/////////////////// start of capturing values////////////////////////////    		 
			switch (event.sensor.getType()) { 

				case (Sensor.TYPE_ACCELEROMETER): 
					{

					accelerometervalues= event.values.clone();
					Rx_acc=accelerometervalues[0];
					Ry_acc=accelerometervalues[1];
					Rz_acc=accelerometervalues[2];
					time = event.timestamp;
				
					}//end case
			
			}//end switch
			

/////////////////// end of capturing values////////////////////////////  
			
				//////////////////for the saving begin values///////////////////////
				if (counter_acc<1) 
				{
					begin_Rx_acc=Rx_acc;
					begin_Ry_acc=Ry_acc;
					begin_Rz_acc=Rz_acc;
					
					//begin_time = event.timestamp/1000000;//nanosec to milisec
					begin_time = time;
					
					
					begin_R_acc= Math.sqrt(Math.pow(begin_Rx_acc,2)+Math.pow(begin_Ry_acc,2)+Math.pow(begin_Rz_acc,2));

					if (begin_R_acc==0) //for the case of dividing by 0
					{
						Axr=0;
						Ayr=0;
						Azr=0;	 
					}

					else			//for all other cases

					{
						begin_Axr=Math.acos(begin_Rx_acc/begin_R_acc);
						begin_Ayr=Math.acos(begin_Ry_acc/begin_R_acc);
						begin_Azr=Math.acos(begin_Rz_acc/begin_R_acc);
					}
				}
				/////////////end of saving the begin values/////////////

				counter_acc++;
	 

////////now we will calculate the gravitation differences between now and the begining///////////////
		
		
		diff_time=time-begin_time;
				
		diff_acc_x=Rx_acc-begin_Rx_acc;
		diff_acc_y=Ry_acc-begin_Ry_acc;
		diff_acc_z=Rz_acc-begin_Rz_acc;
		
		

		
		Log.i(rotem, "The gravitation changed in x:  " + diff_acc_x);			
		Log.i(rotem, "The gravitation changed in y:  " + diff_acc_y);
		Log.i(rotem, "The gravitation changed in z:  " + diff_acc_z);
////////end of calculating the gravitation differences between now and the begining///////////////
		
//JUMP TO LINE 334 IF YOU DONT WANT TO FILTER THE SAMPLES!!		
		
//this part is when we want to filter the examples-dont touch				
		//shifthing left 
/*	for (int i=0;i<=8;i++) {
	
		Array_x[i]=Array_x[i+1];
		Array_y[i]=Array_y[i+1];
		Array_z[i]=Array_z[i+1];
	}
	
	Array_x[9]=Rx_acc;
	Array_y[9]=Ry_acc;
	Array_z[9]=Rz_acc;

	//calculating
	
	sum_array_x=0;
	sum_array_y=0;
	sum_array_z=0;
	
	for (int j=0;j<=9;j++) {
		
		sum_array_x=sum_array_x+Array_x[j];
		sum_array_y=sum_array_y+Array_y[j];
		sum_array_z=sum_array_z+Array_z[j];
	}
	
	
	if (counter_array>10)	
	{
		Rx_acc=sum_array_x/10;
		Ry_acc=sum_array_y/10;
		Rz_acc=sum_array_z/10;
	}
				

	//for the first time values (for the last time values part)
							 if ((last_Rx_acc==0) &&   (last_Ry_acc==0) && (last_Rz_acc==0)) {

								 last_Rx_acc=Rx_acc;
								 last_Ry_acc=Ry_acc;
								 last_Rz_acc=Rz_acc; 
							 }

	//after the first time we will always enter here (for the last time values)
							else	{

								diff_Rx_acc=Rx_acc-last_Rx_acc;
								diff_Ry_acc=Ry_acc-last_Ry_acc;
								diff_Rz_acc=Rz_acc-last_Rz_acc;

								last_Rx_acc=Rx_acc;
								last_Ry_acc=Ry_acc;
								last_Rz_acc=Rz_acc;
							 }*/
						////////end of the part of when we want to get the last sample values///////////////						 
	//end of part  when we want to filter the examples-dont touch
	
		
			//Eq.1				 
			R_acc= Math.sqrt(Math.pow(Rx_acc,2)+Math.pow(Ry_acc,2)+Math.pow(Rz_acc,2));//the total R in the equations
			
			
			//now we calculate the angle around each axis
			if (R_acc==0) //for the case of dividing by 0
			{
				Axr=0;
				Ayr=0;
				Azr=0;	 
			}

			else			//for all other cases

			{
				//Eq.3
				Axr= Math.acos(Rx_acc/R_acc);
				Ayr= Math.acos(Ry_acc/R_acc);
				Azr= Math.acos(Rz_acc/R_acc);
			}
			//end of calculating the angle around each axis
			
			//if we want to check that the total movement in all axis is 1 (not very important, dont pay attention)
			//Eq. 2
		/*	Cos_X= Math.cos(Axr);
			Cos_Y= Math.cos(Ayr);
			Cos_Z= Math.cos(Azr);
			
			
			overall_movement= Math.sqrt(Math.pow(Cos_X,2)+Math.pow(Cos_Y,2)+Math.pow(Cos_Z,2));
			//end of checking that the total movement in all axis is 1
*/			
			
			//this is angles differences from the start  
			diff_Axr_acc=Axr-begin_Axr;
			diff_Ayr_acc=Ayr-begin_Ayr;
			diff_Azr_acc=Azr-begin_Azr;

			Log.i(rotem, "The angle changed in x:  " + diff_Axr_acc);
			Log.i(rotem, "The angle changed in z:  " + diff_Azr_acc);
			//end of angles differences from the start 
			
		//////////////////now put use the values to calculate the matrix rotation///////////////////////////// 

			//calculating the rotation matrix components 
			
			cos_Theta_x=  Math.cos(diff_Axr_acc);
			sin_Theta_x=  Math.sin(diff_Axr_acc);
			
			cos_Theta_y=  Math.cos(diff_Ayr_acc);
			sin_Theta_y=  Math.sin(diff_Ayr_acc);

			cos_Theta_z=  Math.cos(diff_Azr_acc);
			sin_Theta_z=  Math.sin(diff_Azr_acc);

			
			//building the rotation matrix
			R_rotate_acc[0]=cos_Theta_y*cos_Theta_z;
			R_rotate_acc[1]=(cos_Theta_z*sin_Theta_x*sin_Theta_y)-(cos_Theta_x*sin_Theta_z);
			R_rotate_acc[2]=(sin_Theta_x*sin_Theta_z)+(cos_Theta_x*cos_Theta_z*sin_Theta_y);
			R_rotate_acc[3]=(cos_Theta_y*sin_Theta_z);
			R_rotate_acc[4]= (sin_Theta_x*sin_Theta_y*sin_Theta_z) + (cos_Theta_x*cos_Theta_z);
			R_rotate_acc[5]= (cos_Theta_x*sin_Theta_y*sin_Theta_z) - (cos_Theta_z*sin_Theta_x);
			R_rotate_acc[6]=-sin_Theta_y;
			R_rotate_acc[7]=cos_Theta_y*sin_Theta_x;
			R_rotate_acc[8]=cos_Theta_x*cos_Theta_y;
			

			//insert all to a matrix type
			mMatrix_R.set(0, 0, R_rotate_acc[0]);
			mMatrix_R.set(0, 1,	R_rotate_acc[1]);
			mMatrix_R.set(0, 2, R_rotate_acc[2]);
			mMatrix_R.set(1, 0, R_rotate_acc[3]);
			mMatrix_R.set(1, 1, R_rotate_acc[4]);
			mMatrix_R.set(1, 2, R_rotate_acc[5]);
			mMatrix_R.set(2, 0, R_rotate_acc[6]);
			mMatrix_R.set(2, 1, R_rotate_acc[7]);
			mMatrix_R.set(2, 2, R_rotate_acc[8]);
		
			
			
			
			if (k<200) {
				
				data_array[k][0]=diff_time;
				
				data_array[k][1]=diff_acc_x;
				data_array[k][2]=diff_acc_y;
				data_array[k][3]=diff_acc_z;
				
				data_array[k][4]=Math.toDegrees(Axr);
				data_array[k][5]=Math.toDegrees(Ayr);
				data_array[k][6]=Math.toDegrees(Azr);
				
				data_array[k][7]=Math.toDegrees(diff_Axr_acc);
				data_array[k][8]=Math.toDegrees(diff_Ayr_acc);
				data_array[k][9]=Math.toDegrees(diff_Azr_acc);
				
				data_array[k][10]=R_rotate_acc[0];
				data_array[k][11]=R_rotate_acc[1];
				data_array[k][12]=R_rotate_acc[2];
				data_array[k][13]=R_rotate_acc[3];
				data_array[k][14]=R_rotate_acc[4];
				data_array[k][15]=R_rotate_acc[5];
				data_array[k][16]=R_rotate_acc[6];
				data_array[k][17]=R_rotate_acc[7];
				data_array[k][18]=R_rotate_acc[8];
			}
			
			k=k+1;
			
			Log.i(rotem, "K length:  " + k);
			
			INV_mMatrix_R=mMatrix_R.inverse();//if we want to send out tne inverse matrix
	
			

			
			//print to the log window
			Log.i(rotem, "rotation  matrix0:  " + mMatrix_R.get(0, 0));
			Log.i(rotem, "rotation  matrix1:  " + mMatrix_R.get(0, 1));
			Log.i(rotem, "rotation  matrix2:  " + mMatrix_R.get(0, 2));
			Log.i(rotem, "rotation  matrix3:  " + mMatrix_R.get(1, 0));
			Log.i(rotem, "rotation  matrix4:  " + mMatrix_R.get(1, 1));
			Log.i(rotem, "rotation  matrix5:  " + mMatrix_R.get(1, 2));
			Log.i(rotem, "rotation  matrix6:  " + mMatrix_R.get(2, 0));
			Log.i(rotem, "rotation  matrix7:  " + mMatrix_R.get(2, 1));
			Log.i(rotem, "rotation  matrix8:  " + mMatrix_R.get(2, 2));
			
			if (k==200)
				
			{
			
			try {
				 PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter("/mnt/sdcard/data_values.text")));
					
				 
				for (int p=0;p<200;p++)
				{

					String aString1 = Double.toString(data_array[p][0]);
					
					String aString2 = Double.toString(data_array[p][1]);
					String aString3 = Double.toString(data_array[p][2]);
					String aString4 = Double.toString(data_array[p][3]);
					
					String aString5 = Double.toString(data_array[p][4]);
					String aString6 = Double.toString(data_array[p][5]);
					String aString7 = Double.toString(data_array[p][6]);
					
					String aString8 = Double.toString(data_array[p][7]);
					String aString9 = Double.toString(data_array[p][8]);
					String aString10 = Double.toString(data_array[p][9]);
					
					String aString11 = Double.toString(data_array[p][10]);
					String aString12 = Double.toString(data_array[p][11]);
					String aString13 = Double.toString(data_array[p][12]);
					String aString14 = Double.toString(data_array[p][13]);
					String aString15 = Double.toString(data_array[p][14]);
					String aString16 = Double.toString(data_array[p][15]);
					String aString17 = Double.toString(data_array[p][16]);
					String aString18 = Double.toString(data_array[p][17]);
					String aString19 = Double.toString(data_array[p][18]);
					
					
					out.print("\n\r");
					out.print(aString1);
					out.print("\t\t\t");
					
					
					out.print(aString2);
					out.print("\t\t\t");
					out.print(aString3);
					out.print("\t\t\t");
					out.print(aString4);
					out.print("\t\t\t");
					
					
					out.print(aString5);
					out.print("\t\t\t");
					out.print(aString6);
					out.print("\t\t\t");
					out.print(aString7);
					out.print("\t\t\t");
					
					
					out.print(aString8);
					out.print("\t\t\t");
					out.print(aString9);
					out.print("\t\t\t");
					out.print(aString10);
					out.print("\t\t\t");
					
					
					out.print(aString11);
					out.print("\t\t\t");
					out.print(aString12);
					out.print("\t\t\t");
					out.print(aString13);
					out.print("\t\t\t");
					out.print(aString14);
					out.print("\t\t\t");
					out.print(aString15);
					out.print("\t\t\t");
					out.print(aString16);
					out.print("\t\t\t");
					out.print(aString17);
					out.print("\t\t\t");
					out.print(aString18);
					out.print("\t\t\t");
					out.print(aString19);
					
					out.print("\n\r");
					out.flush();
				}
			} catch (IOException e) {System.err.println("Error:" + e.getMessage());}	
	
		}
			//////////////////////////////end of using the values to calculate the matrix rotation//////////////////////////////////////////////////// 

  	 
		};////////////////////////end of onSensorChange

	};////////////////////////end of SensorEventListener    

}














