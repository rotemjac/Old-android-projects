package com.feature_tracker;


import android.app.Activity;
import android.content.Context;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Window;

public class Feature_trackerActivity extends Activity {
	 private static final String TAG             = "Feature_tracker_Activity";
	 public SensorManager myManager;
	 private static Context CONTEXT;
	 
	 public Feature_trackerActivity() {
	        Log.i(TAG, "Instantiated new " + this.getClass());
	 }	
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "onCreate");
    	super.onCreate(savedInstanceState);
    	requestWindowFeature(Window.FEATURE_NO_TITLE);
    	CONTEXT = this;
        setContentView(new Feature_trackerView(this));
        myManager = (SensorManager)getSystemService(Context.SENSOR_SERVICE);
    }
    
    public static Context getContext() {
        return CONTEXT;
    }
}