/*============================================================
/ Red Storm Robotics 2020/21
/=============================================================
/ TELE-OP MODE
/
/ This contains the Tele-Op controls and drivers
/=============================================================
/ Hardware:
/
/ None Directly (Only That Used By Dependencies)
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ModeTeleOp extends Mode {

    // Copies of objects that are used by this mode:
    Gamepad gp1;
    Gamepad gp2;

    public ModeTeleOp(Config config, Gamepad gp1, Gamepad gp2) 
    {
        super(config);
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    protected boolean Init() 
    {
        return true;
    }

    protected void Loop() 
    { 
        SmartDashboard.putNumber("Left Y",  gp1.getLeftY());
        SmartDashboard.putNumber("Right Y", gp1.getRightY());
    }
}