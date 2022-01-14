/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.oi;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveStraightCommand;

/**
 * Driver OI Controls
 */
public class DriverOI extends OI {

    public DriverOI(int channel, RobotContainer robotContainer) {
        super(channel);
        
        a = new JoystickButton(joystick, Constants.A);
        y = new JoystickButton(joystick, Constants.Y);

        b = new JoystickButton(joystick, Constants.B);

       //b.whileHeld(new DriveToTargetPIDCommand(Robot.DRIVE_KP, Robot.DRIVE_KI, Robot.DRIVE_KD, Robot.DRIVE_BASEspeed_));
        a = new JoystickButton(joystick, Constants.A);
        a.whileHeld(new DriveStraightCommand(robotContainer));
    }
}
