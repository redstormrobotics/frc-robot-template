/*
 * ===============================================================================================
 * Red Storm Robotics 2020/21
 * ===============================================================================================
 * HEALTH / / This monitors and reports system health information
 * ===============================================================================================
 * Hardware: / / N/A
 * ===============================================================================================
 * 3rd Party Dependencies: None
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

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Health {
    private static Health _me = null;
    private String errors = "";
    private double cur_battery_voltage = 15.0;
    private double min_battery_voltage = 15.0;
    private double cur_current_input = 0.0;
    private double max_current_input = 0.0;
    private int loop_counter = 0;
    private double last_loop_time = 0;
    private double cur_loop_time = 0;

    private static int verbosityLevel;
    public static final int ERROR = 0;
    public static final int WARNING = 1;
    public static final int INFO = 2;
    public static final int DEBUG = 3;
    public static final int TRACE = 4;

    public Health(int robotNum) {
        errors = "";
        verbosityLevel = DEBUG; // INFO;
    }

    public void loop() {
        loop(null);
    }

    public void loop(PowerDistribution pdu) {
        // Track Loops
        loop_counter++;
        double this_loop_time = Timer.getFPGATimestamp();
        cur_loop_time = ((this_loop_time - last_loop_time) + 7 * cur_loop_time) / 8.0;
        last_loop_time = this_loop_time;

        // Track latest battery voltage
        cur_battery_voltage = RobotController.getBatteryVoltage();
        if (cur_battery_voltage < min_battery_voltage) {
            min_battery_voltage = cur_battery_voltage;
        }

        // Track latest input current draw
        cur_current_input = RobotController.getInputCurrent();
        if (cur_current_input > max_current_input) {
            max_current_input = cur_current_input;
        }

        // Periodically report all health information
        if ((loop_counter % 10) == 0) {

            info("version", 1);
            info("health - loop count", loop_counter);
            info("health - loop time", cur_loop_time);
            info("health - battery", cur_battery_voltage);
            info("health - battery min", min_battery_voltage);
            info("health - current", cur_current_input);
            info("health - current max", max_current_input);
            if (errors.length() > 0) {
                error("health - errors", errors);
                error("health - ok", false);
            } else {
                error("health - errors", ":D");
                error("health - ok", true);
            }
            if (pdu != null) {
                StringBuilder pduStatus = new StringBuilder();
                PowerDistributionFaults faults = pdu.getFaults();
                if (faults.Brownout) {
                    pduStatus.append("BROWNOUT ");
                }
                if (faults.CanWarning) {
                    pduStatus.append("CAN ");
                }
                if (faults.HardwareFault) {
                    pduStatus.append("HARDWARE ");
                }
                error("power errors", pduStatus.toString());
            } else {
                error("power errors", "not found");
            }
        }
    }

    public void addError(String newError, Exception e) {
        errors += newError + "; ";
        if (e != null) {
            System.err.println(e.toString());
            e.printStackTrace();
        }
    }

    public void addError(String newError) {
        addError(newError, null);
    }

    public void clearErrors() {
        errors = "";
    }

    public static Health getHealth() {
        if (_me == null) {
            _me = new Health(0);
        }
        return _me;
    }

    public static void verbosity(int level) {
        verbosityLevel = level;
    }

    public static void error(String key, boolean value) {
        if (verbosityLevel >= ERROR) {
            SmartDashboard.putBoolean(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void error(String key, double value) {
        if (verbosityLevel >= ERROR) {
            SmartDashboard.putNumber(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void error(String key, String value) {
        if (verbosityLevel >= ERROR) {
            SmartDashboard.putString(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void warning(String key, boolean value) {
        if (verbosityLevel >= WARNING) {
            SmartDashboard.putBoolean(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void warning(String key, double value) {
        if (verbosityLevel >= WARNING) {
            SmartDashboard.putNumber(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void warning(String key, String value) {
        if (verbosityLevel >= WARNING) {
            SmartDashboard.putString(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void info(String key, boolean value) {
        if (verbosityLevel >= INFO) {
            SmartDashboard.putBoolean(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void info(String key, double value) {
        if (verbosityLevel >= INFO) {
            SmartDashboard.putNumber(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void info(String key, String value) {
        if (verbosityLevel >= INFO) {
            SmartDashboard.putString(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void debug(String key, boolean value) {
        if (verbosityLevel >= DEBUG) {
            SmartDashboard.putBoolean(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void debug(String key, double value) {
        if (verbosityLevel >= DEBUG) {
            SmartDashboard.putNumber(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void debug(String key, String value) {
        if (verbosityLevel >= DEBUG) {
            SmartDashboard.putString(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void trace(String key, boolean value) {
        if (verbosityLevel >= TRACE) {
            SmartDashboard.putBoolean(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void trace(String key, double value) {
        if (verbosityLevel >= TRACE) {
            SmartDashboard.putNumber(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }

    public static void trace(String key, String value) {
        if (verbosityLevel >= TRACE) {
            SmartDashboard.putString(key, value);
        }
        if (verbosityLevel >= TRACE) {
            System.out.print(key);
            System.out.print(": ");
            System.out.println(value);
        }
    }
}
