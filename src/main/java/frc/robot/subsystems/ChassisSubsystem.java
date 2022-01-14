// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.oi.OI;

public class ChassisSubsystem extends SubsystemBase {

  public enum RobotSide {
    LEFT, RIGHT;
  }

  Talon frontLeftMotor_ = new Talon(Constants.FRONT_LEFT_MOTOR);
  Talon frontRightMotor_ = new Talon(Constants.FRONT_RIGHT_MOTOR);
  Talon rearLeftMotor_ = new Talon(Constants.REAR_LEFT_MOTOR);
  Talon rearRightMotor_ = new Talon(Constants.REAR_RIGHT_MOTOR);

  MotorControllerGroup leftMotors_ = new MotorControllerGroup(frontLeftMotor_, frontLeftMotor_);
  MotorControllerGroup rightMotors_ = new MotorControllerGroup(frontRightMotor_, rearRightMotor_);

  DifferentialDrive driveTrain_ = new DifferentialDrive(leftMotors_, rightMotors_);

  //drive constants
  /**
  * The scaling factor between the joystick value and the speed controller
  */
  private static double speedMultiplier = 1.0;

  /**
   * The scale factor for normal mode
   */
  private static final double normal = 1.0;

  /**
   * The scale factor for crawl mode
   */
  private static final double crawl = 0.5;  

    /**
   * The minimum (closest to 0) speed controller command for the right side of the drive train to start moving forward. Must be empirically derived.
   */
  private static double mechDeadbandRightForward = 0.25;

  /**
   * The maximum (closest to 0) speed controller command for the right side of the drive train to start moving backward. Must be empirically derived.
   * Must be negative.
   */
  private static double mechDeadbandRightBackward = -0.27;

  /**
   * The minimum (closest to 0) speed controller command for the left side of the drive train to start moving forward. Must be empirically derived.
   */
  private static double mechDeadbandLeftForward = 0.23;

  /**
   * The maximum (closest to 0) speed controller command for the left side of the drive train to start moving backward. Must be empirically derived.
   * Must be negative.
   */
  private static double mechDeadbandLeftBackward = -0.29;

  /**
   * The minimum joystick value to actually send a command to the speed controller, to prevent noise near 0.
   */
  private static double softwareDeadband = 0.05;

  //arcade drive constant
private double turnGain = 0.75;
  
  /**
   * The direction which is "forward"; 1 represents the hatch side and -1 represents the cargo side.
   */
  private int dir = 1;

  /** Creates a new ChassisSubsystem. */
  public ChassisSubsystem() {
    rightMotors_.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param leftspeed - The left joystick controller spped -1 to 1
   * @param rightspeed - The right joystick controller speed -1 to 1
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    driveTrain_.tankDrive(leftSpeed, rightSpeed);
  }

  /**
   *  The Archade drive uses the speed to control to forwrad and backwar movement where the rotation is for making
   *  left or right turn
   * @param xSpeed - The X joystick controller speed -1 to 1
   * @param zRotation - The Z joystick controller speed -1 to 1
   */
  public void driveArcade(double xSpeed, double zRotation) {
    driveTrain_.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * drive straigth
   * @param oi - driver interface
   * @param scale - scale factor
   */
  public void driveStraight(OI oi, int scale) {
    double rightVal = -getScaledValue(oi.getJoystick().getRawAxis(3), scale, RobotSide.RIGHT);
    double leftVal = -getScaledValue(oi.getJoystick().getRawAxis(3), scale, RobotSide.LEFT);
    tankDrive(0.7 * leftVal, 0.7 * rightVal);
  }

  /**
   * Method to drive in tank drive based on joystick input.
   * @param oi The oi to base joystick values off of (Driver or CoDriver.)
   * @param scale Whether or not to scale output values to account for mechanical deadband. 0 is no scaling; 1 is linear scaling; 2 is quadratic scaling
   */
  public void drive(OI oi, int scale) {
    speedMultiplier = oi.getJoystick().getRawButton(Constants.RIGHT_BUMPER) ? crawl : normal;
    dir = oi.getJoystick().getRawButton(Constants.LEFT_BUMPER) ? -1 : 1;

    double rightVal = 0;
    double leftVal = 0;

    if(dir == 1) {
        rightVal = getScaledValue(oi.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(oi.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.LEFT);
    } else if(dir == -1) {
        rightVal = getScaledValue(oi.getJoystick().getRawAxis(Constants.LEFT_JOYSTICK_Y), scale, RobotSide.RIGHT);
        leftVal = getScaledValue(oi.getJoystick().getRawAxis(Constants.RIGHT_JOYSTICK_Y), scale, RobotSide.LEFT);
    }
    
    tankDrive(leftVal * speedMultiplier * dir, rightVal * speedMultiplier * dir);
  }

  /**
   * Find the scale factor based on the input value and scale on each side of the Robot's motors
   * 
   * @param val
   * @param scale
   * @param side
   * @return
   */
  private double getScaledValue(double val, int scale, RobotSide side) {
    double forward, back;

    if(side.equals(RobotSide.RIGHT)) {
        forward = Math.abs(mechDeadbandRightForward);
        back = Math.abs(mechDeadbandRightBackward);
    } else {
        forward = Math.abs(mechDeadbandLeftForward);
        back = Math.abs(mechDeadbandLeftBackward);
    }

    if(Math.abs(val) < softwareDeadband) {
        return 0;
    } else if(scale == 0) {
        return deadzone(val);
    } else if(scale == 1) {
        if(val > 0) {
            return (((1.0 - forward) * (val - 1.0) / (1.0 - softwareDeadband)) + 1.0);
        } else {
            return (((1.0 - back) * (val + 1.0) / (1.0 - softwareDeadband)) -  1.0);
        }
    } else if(scale == 2) {
        if(val > 0) {
            return (1- forward) / Math.pow(1 - softwareDeadband, 2) * Math.pow(val - softwareDeadband, 2) + forward;
        } else {
            return (-1 + back) / Math.pow(-1 + softwareDeadband, 2) * Math.pow(val + softwareDeadband, 2) - back;
        }
    } else {
        return 0;
    }
  }

  /**
   * Method to reduce noise around the 0 position of the joystick.
   * @param val The raw joystick input.
   * @return If the input is outside the range <code> (-softwareDeadband < val < softwareDeadband)</code>, it is returned. Else, 0 is returned.
   */
  private double deadzone(double val) {
    return Math.abs(val) > softwareDeadband ? val : 0;
  }
}
