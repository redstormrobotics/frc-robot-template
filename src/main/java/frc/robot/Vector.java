package frc.robot;

public class Vector {
    private double x;
    private double y;
    private double z;
    private boolean polarXY = false;
    private boolean cartXY = false;
    private double angleXY;
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

    public static Vector fromPolar(double lengthXY, double angleXY, double z) {
        Vector v = new Vector();
        v.lengthXY = lengthXY;
        v.angleXY = angleXY;
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

    public double getAngleXY() {
        if (!polarXY) {
            calcPolarXY();
        }
        return angleXY;
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
        if (x > 0) {
            angleXY = Math.atan(y / x) * 180 / Math.PI;
        } else if (x < 0) {
            angleXY = Math.atan(y / x) * 180 / Math.PI + 180;
        } else if (y > 0) {
            angleXY = 90;
        } else {
            angleXY = 270;
        }
        if (angleXY < 0) {
            angleXY = angleXY + 360;
        }
        polarXY = true;

    }

    private void calcCartXY() {
        x = lengthXY * Math.cos((angleXY * Math.PI) / 180);
        y = lengthXY * Math.sin((angleXY * Math.PI) / 180);
        cartXY = true;
    }

}