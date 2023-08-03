/*
 * ===============================================================================================
 * Red Storm Robotics 2022/23
 * ===============================================================================================
 * Drive Train Base
 * ===============================================================================================
 * Hardware: / / None. This is an interface
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

public interface DriveTrainTank {

    public enum SPEED {
        FAST, MEDIUM, SLOW
    };

    public void Init();

    public void configBrakeOnStop();

    public void configCoastOnStop();

    public void drive(double forward, double right);

    public void drive(double forward, double right, double limit);

    public void drive(double forward, double right, double limit, boolean useBrake);


    public double adjustLeftDrive(double forward, double right, double limit);

    public double adjustRightDrive(double forward, double right, double limit);

    public void setSpeedLimit(SPEED speedLimit);

    public void setSpeedLimit(double speedLimit);

    public void setSpeedBoost();

    public void setSpeedCreep();

    public void setNormalSpeed();

    public void stopAll();

    public void brake();

    public void regoToHoldingPosition();

    public void markEncoderLeftPosition();

    public void markEncoderRightPosition();

    public double getMarkedEncoderLeftPosition();

    public double getMarkedEncoderRightPosition();

    public void updateBrakePosition(double leftInches, double rightInches);

    public void setTargetPosition(double leftInches, double rightInches);

    public boolean areWeThereYet();
}
