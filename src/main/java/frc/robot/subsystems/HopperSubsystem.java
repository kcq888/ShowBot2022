// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.OI;

public class HopperSubsystem extends SubsystemBase {
  private MotorController hopperMotor_;
  private DigitalInput hopperSwitch_;
  
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    hopperMotor_ = new Talon(Constants.HOPPER_MOTOR);
    hopperSwitch_ = new DigitalInput(Constants.HOPPER_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopHopper() {
    hopperMotor_.set(0);
  }

  public void driveHopper(double speed) {
    hopperMotor_.set(-speed);
  }

  public void driveHopper(OI oi) {
    driveHopper(deadzone(oi, Constants.RIGHT_JOYSTICK_VERTICAL_AXIS));
  }

  private double deadzone(OI oi, int axis) {
    double rawAxis = oi.getJoystick().getRawAxis(axis);
    if(Math.abs(rawAxis) > 0.04)
      return rawAxis;
    return 0;
  }

  public boolean getHopperSwitchState() {
    return hopperSwitch_.get();
  }
}
