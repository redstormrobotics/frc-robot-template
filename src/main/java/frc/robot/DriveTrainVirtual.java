/*
 * ===============================================================================================
 * Red Storm Robotics 2022/23
 * ===============================================================================================
 * Virtual Drive Train
 * ===============================================================================================
 * Hardware: / / None. This is a stub.
 * ===============================================================================================
 * 3rd Party Dependencies: / / None
 * ===============================================================================================
 * Permission is hereby granted, free of charge, to any / person obtaining a copy of this software
 * and associated / documentation files (the "Software"), to deal in the / Software without
 * restriction, including without limitation / the rights to use, copy, modify, merge, publish,
 * distribute, / sublicense, and/or sell copies of the Software, and to / permit persons to whom the
 * Software is furnished to do so, / subject to the following conditions: / / The above copyright
 * notice and this permission notice shall / be included in all copies or substantial portions of
 * the Software. / / THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY / KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE / WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR / PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS / OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR / OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR /
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE / SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 * ===============================================================================================
 */
package frc.robot;

public class DriveTrainVirtual implements DriveTrain {

    public DriveTrainVirtual() {}

    public void Init() {}

    // Driving Controls
    public void drive(double forward, double turnRight) {}
    public void drive(double forward, double turnRight, double strafeRight) {}
    public void drive(double forward, double turnRight, double strafeRight, double limit) {}
    public void drive(double forward, double turnRight, double strafeRight, double limit, boolean useBrake) {}
    public void stopAll() {}

    // Driving Speed Limits
    public void setSpeedLimit(SPEED speedLimit) {}
    public void setSpeedLimit(double speedLimit) {}
    public void setSpeedBoost() {}
    public void setSpeedCreep() {}
    public void setNormalSpeed() {}

    // Drivetrain Features
    public void configBrakeOnStop() {}
    public void configCoastOnStop() {}

    // Auton Support
    public boolean setTargetPosition(double x, double y, double heading) { return true; }
    public boolean areWeThereYet() { return true; }
}
