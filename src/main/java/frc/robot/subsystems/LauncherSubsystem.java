// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LauncherSubsystem extends SubsystemBase {
  private MotorController launcherMotor__ = null;
  private Encoder launcherEncoder__ = null;

  /** Creates a new LauncherSubsystem. */
  public LauncherSubsystem() {
    launcherMotor__ = new Talon(Constants.LAUNCHER_MOTOR);
    launcherEncoder__ = new Encoder(Constants.LAUNCHER_ENCODER_A, Constants.LAUNCHER_ENCODER_B);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveLauncher(double speed) {
    System.out.println("Encoder count: " + launcherEncoder__.get());
    launcherMotor__.set(speed);
  }
}
