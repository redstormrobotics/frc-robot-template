/*============================================================
/ Red Storm Robotics 2020/21
/=============================================================
/ GAMEPAD
/
/ This is an abstraction to make the gamepad controller 
/ simpler to read from.
/=============================================================
/ Hardware:
/
/ 1+ gamepads attached to the driver station
/=============================================================
/ 3rd Party Dependencies:
/
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
import edu.wpi.first.wpilibj.Joystick;

public class Gamepad {

	private final Joystick controller;
	
	/*
	 * BUTTONS AND AXES
	 */
	public enum Button
	{
		A, B, X, Y, RB, LB, RT, LT, START, SELECT, LS, RS;
	}
	
	public enum Direction
	{
		NORTH, SOUTH, EAST, WEST, NORTHEAST, SOUTHEAST, SOUTHWEST, NORTHWEST;
	}

	// Mapping of Axis
	private enum Axis
	{
		LeftStickX(0), LeftStickY(1), RightStickX(2), RightStickY(3), LeftTrigger(4), RightTrigger(5);

		private int value;
		Axis(int value) { this.value = value; }
		private int getValue() { return value; }
	}
	private final int AxisMapLinux[] = { 0, 1, 2, 3, 4, 5 };
	private final int AxisMapWindows[] = { 0, 1, 4, 5, 2, 3 };
	private int AxisMap[] = AxisMapLinux;
		
	public Gamepad(int port)
	{
		controller = new Joystick(port);
		System.out.println(port);
		AxisMap = AxisMapLinux;
	}
		
	public Gamepad(int port, boolean windows)
	{
		controller = new Joystick(port);
		System.out.println(port);
		if (windows)
		{
			AxisMap = AxisMapWindows;
		}
		else
		{
			AxisMap = AxisMapLinux;
		}
	}
	
	// These are the normal Stick Reading Functions
	public double getLeftX()
	{
		return controller.getRawAxis( AxisMap[Axis.LeftStickX.getValue()] );
	}
	
	public double getLeftY()
	{
		return -controller.getRawAxis( AxisMap[Axis.LeftStickY.getValue()] );
	}
	
	public double getRightX()
	{
		return controller.getRawAxis( AxisMap[Axis.RightStickX.getValue()] );
	}
	
	public double getRightY()
	{
		return -controller.getRawAxis( AxisMap[Axis.RightStickY.getValue()] );
	}
	
	// These Stick Reading Functions Scale for Higher Resolution Towards the Center
	public double getAdjustedLeftX()
	{
		double val = controller.getRawAxis( AxisMap[Axis.LeftStickX.getValue()] );
        return Math.pow(val, 3);
	}
	
	public double getAdjustedLeftY()
	{
		double val = -controller.getRawAxis( AxisMap[Axis.LeftStickX.getValue()]  );
        return Math.pow(val, 3);
	}
	
	public double getAdjustedRightX()
	{
		double val = controller.getRawAxis( AxisMap[Axis.RightStickX.getValue()] );
        return Math.pow(val, 3);
	}
	
	public double getAdjustedRightY()
	{
		double val = -controller.getRawAxis( AxisMap[Axis.RightStickY.getValue()] );
        return Math.pow(val, 3);
	}
	
	// Other Control Inputs:
	public double getLT()
	{
		return controller.getRawAxis( AxisMap[Axis.LeftTrigger.getValue()] );
	}
	
	public boolean DpadDirectionIsPressed(Direction d)
	{
		switch(d)
		{
		case NORTH:
			return controller.getPOV() == 0;
		case EAST:
			return controller.getPOV() == 90;
		case SOUTH:
			return controller.getPOV() == 180;
		case WEST:
			return controller.getPOV() == 270;
		case NORTHEAST:
			return controller.getPOV() == 45;
		case SOUTHEAST:
			return controller.getPOV() == 135;
		case SOUTHWEST:
			return controller.getPOV() == 225;
		case NORTHWEST:
			return controller.getPOV() == 315;
		default:
			return false;
		}
	}
	
	public int getDpadDirection()
	{
		return controller.getPOV();
	}
	
	public double getRT()
	{
		return controller.getRawAxis( AxisMap[Axis.RightTrigger.getValue()] );
	}
	
	public boolean isPressed(Button button)
	{
		/* Xbox controller (X mode) */
		switch(button)
		{
		case A:
			return controller.getRawButton(1);
		case B:
			return controller.getRawButton(2);
		case X:
			return controller.getRawButton(3);
		case Y:
			return controller.getRawButton(4);
		case LB:
			return controller.getRawButton(5);
		case RB:
			return controller.getRawButton(6);
		case SELECT:
			return controller.getRawButton(7);
		case START:
			return controller.getRawButton(8);
		case LS:
			return controller.getRawButton(9);
		case RS:
			return controller.getRawButton(10);
		default:
			return false;
		}
	}
}


