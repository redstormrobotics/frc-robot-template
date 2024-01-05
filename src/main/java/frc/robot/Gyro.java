package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

public class Gyro {
    
    Pigeon2 pigeon;
    double pitchOffset = 0;
    double yawOffset = 0;
    double skewOffset = 0;
    
    public Gyro(int ID) throws Exception
    {
        pigeon = new Pigeon2(ID);
        if (pigeon.getUpTime() <= 0)
        {
            throw new Exception("Pigeon " + String.valueOf(ID) + " Not Found");
        }
    }

    public void gyroOutput()
    {
        Health.info("Gyro Yaw", getYaw());
        Health.info("Gyro Pitch", getPitch());
        Health.info("Gyro Skew", getSkew());
    }

    public void zero(){
        pitchOffset = pigeon.getRoll();
        yawOffset = pigeon.getYaw();
        skewOffset = pigeon.getRoll();  
    }
    
    public double tiltModifier(){
        if (Math.abs(getPitch()) >= 17.0){
            return -1.0;
        }
        else {
            return 1.0;
        }
    }
    public double getPitch(){
        double pitch = pigeon.getRoll(); //TODO: skew for pitch line 29, 
        return pitch - pitchOffset;
    }

    public double getYaw(){
        double yaw = pigeon.getYaw();
        return yaw - yawOffset;
    }

    public double getSkew(){
        double skew = pigeon.getRoll(); 
        return skew - skewOffset;
    }
}