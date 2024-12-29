package frc.robot;

public class Vector {
    private double x;
    private double y;
    private double z;
    private boolean polarXY = false;
    private boolean cartXY = false;
    private double angleXYdegree;
    private double angleXYradian;
    private double lengthXY;
    
    private Vector() {
    }

    public static Vector fromCart(double x, double y, double z) {
        Vector v = new Vector();
        v.x = x;
        v.y = y;
        v.z = z;
        v.cartXY = true;
        return v;
    }

    public static Vector fromPolarDegrees(double lengthXY, double angleXY, double z) {
        Vector v = new Vector();
        v.lengthXY = lengthXY;
        v.angleXYdegree = angleXY;
        v.angleXYradian = angleXY*(Math.PI/180.0);
        v.z = z;
        v.polarXY = true;
        return v;
    }
    public static Vector fromPolarRadians(double lengthXY, double angleXY, double z) {
        Vector v = new Vector();
        v.lengthXY = lengthXY;
        v.angleXYradian = angleXY;
        v.angleXYdegree = (180.0/Math.PI)*v.angleXYdegree;
        v.z = z;
        v.polarXY = true;
        return v;
    }

    public Vector addVector(Vector v){
        Vector r = new Vector();
        r.x = this.getX() + v.getX();
        r.y = this.getY() + v.getY();
        r.z = this.getZ() + v.getZ();
        r.cartXY = true;
        return r;
    }

    public Vector timesScalar(double scalar) {
        Vector r = new Vector();
        r.x = this.getX() * scalar;
        r.y = this.getY() * scalar;
        r.z = this.getZ() * scalar;
        r.cartXY = true;
        return r;
    }

    public double getX() {
        if (!cartXY) {
            calcCartXY();
        }
        return x;
    }

    public double getY() {
        if (!cartXY) {
            calcCartXY();
        }
        return y;
    }

    public double getZ() {
        return z;
    }

    public double getAngleXYDegree() {
        if (!polarXY) {
            calcPolarXY();
        }
        return angleXYdegree;
    }
    public double getAngleXYRadian() {
        if (!polarXY) {
            calcPolarXY();
        }
        return angleXYradian;
    }

    public double getLengthXY() {
        if (!polarXY) {
            calcPolarXY();
            polarXY = true;
        }
        return lengthXY;
    }

    public String toString() {
        return "x: " + x + ", y: " + y + ", z: " + z;
    }

    private void calcPolarXY() {

        lengthXY = Math.sqrt(x * x + y * y);
        if (x > 0.0) {
            angleXYradian= Math.atan(y / x);
            angleXYdegree = angleXYradian * 180.0 / Math.PI;
        } else if (x < 0.0) {
            angleXYradian= Math.atan(y / x)+Math.PI;
            angleXYdegree = angleXYradian * 180.0 / Math.PI;
        } else if (y > 0.0) {
            angleXYradian = (Math.PI/2.0);
            angleXYdegree = 90.0;
        } else {
            angleXYradian = (Math.PI*1.5);
            angleXYdegree = 270.0;
        }
        if (angleXYdegree < 0.0) {
            angleXYradian = angleXYradian+Math.PI*2.0;
            angleXYdegree = angleXYdegree + 360.0;
        }
        polarXY = true;

    }

    private void calcCartXY() {
        x = lengthXY * Math.cos((angleXYdegree * Math.PI) / 180.0);
        y = lengthXY * Math.sin((angleXYdegree * Math.PI) / 180.0);
        cartXY = true;
    }

}
