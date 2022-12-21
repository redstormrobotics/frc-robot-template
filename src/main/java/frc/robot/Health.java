
/*============================================================
/ Red Storm Robotics 2020/21
/=============================================================
/ HEALTH
/
/ This monitors and reports system health information
/=============================================================
/ Hardware:
/
/ N/A
/=============================================================
/ 3rd Party Dependencies: None
/=============================================================
/ Permission is hereby granted, free of charge, to any 
/ person obtaining a copy of this software and associated 
/ documentation files (the "Software"), to deal in the 
/ Software without restriction, including without limitation 
/ the rights to use, copy, modify, merge, publish, distribute, 
/ sublicense, and/or sell copies of the Software, and to 
/ permit persons to whom the Software is furnished to do so, 
/ subject to the following conditions:
/ 
/ The above copyright notice and this permission notice shall 
/ be included in all copies or substantial portions of the Software.
/
/ THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY 
/ KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE 
/ WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
/ PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS 
/ OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR 
/ OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
/ OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
/ SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
/=================================================================*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Health 
{
    private String errors = "";
    private double cur_battery_voltage = 15.0;
    private double min_battery_voltage = 15.0;
    private double cur_current_input = 0.0;
    private double max_current_input = 0.0;
    private int loop_counter = 0;
    private double last_loop_time = 0;
    private double cur_loop_time = 0;


    public Health(int robotNum)
    {
        errors = "";
    }

    public void Loop()
    {
        // Track Loops
        loop_counter++;
        double this_loop_time = Timer.getFPGATimestamp();
        cur_loop_time = ((this_loop_time - last_loop_time) + 7 * cur_loop_time) / 8.0;
        last_loop_time = this_loop_time;

        // Track latest battery voltage
		cur_battery_voltage = RobotController.getBatteryVoltage();
		if (cur_battery_voltage < min_battery_voltage)
		{
			min_battery_voltage = cur_battery_voltage;
		}

        // Track latest input current draw
        cur_current_input = RobotController.getInputCurrent();
        if (cur_current_input > max_current_input)
        {
            max_current_input = cur_current_input;
        }

        // Periodically report all health information
		if ((loop_counter % 10) == 0)
		{
			SmartDashboard.putNumber("health - loop count", loop_counter);
			SmartDashboard.putNumber("health - loop time", cur_loop_time);
            SmartDashboard.putNumber("health - battery", cur_battery_voltage);
            SmartDashboard.putNumber("health - battery min", min_battery_voltage);
            SmartDashboard.putNumber("health - current", cur_current_input);
            SmartDashboard.putNumber("health - current max", max_current_input);
            if (errors.length() > 0) {
                SmartDashboard.putString("health - errors", errors);
                SmartDashboard.putBoolean("health - ok", false);
            } else {
                SmartDashboard.putString("health - errors", "none");
                SmartDashboard.putBoolean("health - ok", true);
            }
		}
    }

    public void addError(String newError)
    {
        errors += newError;
    }

    public void clearErrors()
    {
        errors = "";
    }
}