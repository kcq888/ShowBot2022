// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LauncherPIDSubsystem;

public class ChangeLauncherSpeedCommand extends CommandBase {
  private double setpoint_;
  private boolean isFinished_ = false;
  private RobotContainer robotContainer_ = null;
  private LauncherPIDSubsystem launcherPIDSubsystem_ = null;

  /** Creates a new ChangeLauncherSpeedCommand. */
  public ChangeLauncherSpeedCommand(double setpoint, RobotContainer robotContainer) {
    launcherPIDSubsystem_ = robotContainer.getLauncherPIDSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    this.setpoint_ = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcherPIDSubsystem_.setSetpoint(setpoint_);
    launcherPIDSubsystem_.enable();
    isFinished_ = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished_;
  }
}
