package com.feature_tracker;

public class Point3d implements HasCoordinates3d {

	
	public double x, y, z;

    public Point3d() {
    }

    public Point3d(double x, double y, double z) {
            set(x, y, z);
    }

    public Point3d(HasCoordinates3d other) {
            set(other);
    }

    public void set(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
    }

    public void set(HasCoordinates3d v) {
            this.x = v.getX();
            this.y = v.getY();
            this.z = v.getZ();
    }

    public void round(){
            x = Math.round(x);
            y = Math.round(y);
            z = Math.round(z);
    }

    public void floor() {
            x = Math.floor(x);
            y = Math.floor(y);
            z = Math.floor(z);
    }

    public void ceil() {
            x = Math.ceil(x);
            y = Math.ceil(y);
            z = Math.ceil(z);
    }

    public double getX() {
            return x;
    }

    public double getY() {
            return y;
    }
    
	public double getZ() {
		return z;
	}

    public double distanceTo(HasCoordinates3d pt) {
            double dx = x - pt.getX();
            double dy = y - pt.getY();
            double dz = z - pt.getZ();
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    @Override
    public String toString() {
            return "(" + x + ", " + y + ")";
    }

	public double distanceTo(HasCoordinates2d pt) {
		double dx = x - pt.getX();
        double dy = y - pt.getY();
        double dz = z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

}
