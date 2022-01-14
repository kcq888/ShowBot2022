// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class LauncherPIDSubsystem extends PIDSubsystem {
  private MotorController launcherMotor_;
  private Encoder launcherEncoder_;
  private Random randomizer_ = new Random();
  private double currentSpeed_;
  private double currentOutput_;
  
  /** Creates a new LauncherPIDSubsystem. */
  public LauncherPIDSubsystem() {
    super(new PIDController(Constants.kPl, Constants.kIl, Constants.kDl));
    launcherMotor_ = new Talon(Constants.LAUNCHER_MOTOR);
    launcherEncoder_ = new Encoder(Constants.LAUNCHER_ENCODER_A, Constants.LAUNCHER_ENCODER_B);
    //launcherEncoder_.setPIDSourceType(PIDSourceType.kRate);
    setSetpoint(0);
    //setAbsoluteTolerance(0);
    disable();    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    SmartDashboard.putNumber("launcher output", output);
    currentOutput_ = output;
    launcherMotor_.set(output);
  }

  @Override
  public double getMeasurement() {
    return launcherEncoder_.getRate();
  }

  public void stop() {
    double initialGap = currentOutput_;
    double gap = initialGap;
    double initialTime = System.currentTimeMillis();
    while(gap > 0) {
      double newOutput = initialGap - (1000 - (System.currentTimeMillis()-initialTime)) * initialGap;
      gap = newOutput;
      launcherMotor_.set(newOutput);
    }
    launcherMotor_.set(0);
  }

  /**
   * 9.549 is a conversion factor from rad/sec (units of the getRate() method) to rev/min (units of the setpoint)
   * @return
   */
  private double getSpeed() {
    return launcherEncoder_.getRate() / 9.549;
  }
}
