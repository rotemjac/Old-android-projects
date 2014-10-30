package com.feature_tracker;

public class Triplet <A, B, C> extends Pair<A, B> {

	public C c;
	public void set(A reca, B recb, C recc)
	{
		super.set(reca, recb);
		c= recc;
	}
}
