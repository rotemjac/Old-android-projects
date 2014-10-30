package com.feature_tracker;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;
import android.util.Log;

/**
 * 
 * @author Erez Farhan
 *This is a wrapper that helps applying Ransac on your solution method
 *just give it what it needs and invoke "Ransac_loop()";
 * @param <S> The solution type 
 * @param <C> The constraints type [of a single constraint]
 * @param <E> The error size type
 */

public abstract class RANSAC_wrapper<S, C, E> {

	S sol;
	List<C> allConstraints;
	E estError;
	int iterations;
	int modelsetSize;
	int inliersSetsize;
	String TAG = "RANSAC"; 


	/**
	 * 
	 * @param iterations_limit- how many max Ransac iterations
	 * @param modelset_size- length of the set that tries to find agreers [inliers]
	 * @param inliers_setsize- how many inliers needed to agree on a solution
	 * @param all_constraints- a list of all constraints ["measurements"]
	 */
	RANSAC_wrapper (int iterations_limit, int modelset_size, int inliers_setsize, List<C> all_constraints ){
		iterations = iterations_limit;
		modelsetSize = modelset_size;
		allConstraints = all_constraints;
		inliersSetsize = inliers_setsize;
	}


	abstract S solver(List<C> constraints);
	abstract List <Integer> solutionQuality (S chosen_solution, S reference_solution);
	abstract void estimateError ();

	void RANSAC_loop()
	{
		int counter = 0;
		while (true){
			/*1. choose a sublist*/
			List<C> inspected_constrains = new ArrayList<C>();
			List<C> other_constraints= new ArrayList<C>();
			VisionGeoTransforms.chooseRandSublist(allConstraints, inspected_constrains, other_constraints, modelsetSize);

			/*2. calculate the two transforms*/
			S inspected_transform = solver(inspected_constrains);
			S others_transform = solver(other_constraints);

			/*3. check the quality of the chosen transform - if good make a final transform and stop iterating*/
			List <Integer> good_indexes = solutionQuality(inspected_transform, others_transform);
			if (good_indexes.size() >= inliersSetsize){
				List<C> qualityset = VisionGeoTransforms.pickSublist(allConstraints, good_indexes);
				sol = solver(qualityset);
				estimateError();
				break;
			}
			counter ++;
			if (counter > iterations)
			{
				Log.i(TAG, "RANSAC reached limit- calculating on full set");
				sol = solver(allConstraints);
				estimateError();
				break;
			}
			else
				Log.i(TAG, "Continuing RANSAC iterations, so far = " + counter);
		}
	}
}

