// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.HopperSubsystem;

public class IncrementHopperCommand extends CommandBase {
  private Timer timer_;
  private double speed_;
  private HopperSubsystem hopperSubsystem_ = null;
  private boolean initialState_;
  private int increment_;
  private boolean lastState_;
  private boolean thisState_;

  /** Creates a new IncrementHopperCommand. */
  public IncrementHopperCommand(double speed, RobotContainer robotContainer) {
    hopperSubsystem_ = robotContainer.getHopperSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopperSubsystem_);
    this.speed_ = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer_ = new Timer();
    // timer_.schedule(new TimerTask() {
    //   @Override
    //   public void run() {
    //     SmartDashboard.putString("Hopper status", "***Hopper Jammed***");
    //     end(false);
    //   }
    // }, 750L);
    initialState_ = hopperSubsystem_.getHopperSwitchState();
    lastState_ = hopperSubsystem_.getHopperSwitchState();
    increment_ = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisState_ = hopperSubsystem_.getHopperSwitchState();
    if(thisState_ != lastState_) {
      increment_++;
      lastState_ = thisState_;
    }
    hopperSubsystem_.driveHopper(speed_);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopperSubsystem_.driveHopper(0);
    timer_.reset();
    if (initialState_)
      increment_ = 1;
    else
      increment_ = 2;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
