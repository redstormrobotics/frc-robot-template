
/*============================================================
/ Red Storm Robotics 2020/21
/=============================================================
/ MODE
/
/ This is the basic interface for all of the Robot's main
/ control modes
/=============================================================
/ Hardware:
/
/ Is given all required hardware to support this mode
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

public class Mode 
{
    public Mode(Config config)
    {
        initOkay = false;
        
        this.config = config;
    }

    public void Initialize()
    {
        if (config == null)
        {
            return;
        }

        initOkay = Init();
    }

    public void Periodic()
    {
        if (initOkay)
        {
            SmartDashboard.putString("Mode", "Running");
            Loop();
        }
        else
        {
            SmartDashboard.putString("Mode", "Failed to Init");
        }
    } 

    protected boolean Init()
    { 
        return true; 
    }

    protected void Loop()
    { 
        //
    }

    protected Config config;
    protected boolean initOkay;
}