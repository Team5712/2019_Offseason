/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.sim.mockdata.PDPDataJNI;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.RoboMath;

public class Robot extends IterativeRobot {
  // CAN IDs can be changed below
  // new comment - this is new
  // new comment - this is chaytons comment
  // create motor controllers. CTRE Installer required before using in code
  WPI_TalonSRX l_master = new WPI_TalonSRX(1);
  WPI_VictorSPX l_slave = new WPI_VictorSPX(2);
  WPI_TalonSRX r_master = new WPI_TalonSRX(5);
  WPI_VictorSPX r_slave = new WPI_VictorSPX(6);
  // create drive - specify which motor controllers you use. Arcade or Tank will
  // be specified later
  DifferentialDrive drive = new DifferentialDrive(l_master, r_master);
  // create joysticks - we use two for Tank Drive
  Joystick joystick_l = new Joystick(0);
  Joystick joystick_r = new Joystick(1);
  // create gyro. NAVX installer required before using in code
  AHRS gyro = new AHRS(Port.kMXP);

  double robotPositionX = 0;
  double robotPositionY = 0;
  double adjustedHeading;
  // Gets the current time of computer in Milliseconds
  long startTime = System.currentTimeMillis();
  // Find ratio encoders to inches
  double encoderToIn = 12.57f / 517;
  double distance = 0.0f;
  double oldLeft = 0.0f;
  double oldRight = 0.0f;

  double leftDrive, rightDrive;

  double max_spd_l;
  double max_spd_r;
  double max_err;

  PID turnpid;
  PID drivepid;

  int timeout = 20;

  final double APT = (double) (89.73339f / 1034.0);

  AngleMath angleMath;

  @Override
  public void robotInit() {
    
    Values values = new Values();
    angleMath = new AngleMath();
    // robot init is run when your robot is turned on
    // reset gyro Yaw
    gyro.zeroYaw();
    // reset encoders to zero
    l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    // invert drive motors because they were running the wrong way
    l_master.setInverted(true);
    r_master.setInverted(true);

    // i = .008 iz = 5
    //turnpid = new PID(0.04, 0, 0, .04, 0);
    turnpid = new PID(0.05, 0, 0, .04, 0);
    // .4
    drivepid = new PID(.05, 0, 0, 0, 0);

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopInit() {

    l_master.setInverted(false);
    r_master.setInverted(false);
    // reset gyro Yaw
    l_master.setSelectedSensorPosition(0);
    r_master.setSelectedSensorPosition(0);
    robotPositionX = 0;
    robotPositionY = 0;
    gyro.zeroYaw();
    l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    initPID();
  }

  public void initPID() {
    l_master.selectProfileSlot(0, 0);
    // l_master.config_kF(0, pid.kF, timeout);
    // l_master.config_kP(0, pid.kP, timeout);
    // l_master.config_kI(0, pid.kI, timeout);
    // l_master.config_kD(0, pid.kD, timeout);

  }

  @Override
  public void teleopPeriodic() {

    double[] destinationPoint = {-48, 24*3};
    double[] getError = angleMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),destinationPoint);
    //angleMath.getRelativeCoorinateAngle();
    double turnSpeed = this.getTurnPIDOutput(getError[1]);
    double driveSpeed = this.getDrivePIDOutput(getError[0]);
    System.out.println("Angle: "+getError[1]);
    //System.out.println("Angle: "+angleMath.calulateTurnAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw())));
    //System.out.println("Distance: "+angleMath.getRelativeDistance());
    //System.out.println("Relative Angle: "+angleMath.getRelativeCoorinateAngle());
    leftDrive = -turnSpeed +driveSpeed;
    rightDrive = turnSpeed +driveSpeed;

    drive.tankDrive(leftDrive, rightDrive);
    

    this.updatePosition();
    this.updateDashboard();
  }

  public boolean threshold(double value, double setpoint, double threshold) {
    if (value < (setpoint + threshold) && value > (setpoint - threshold)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void autonomousInit() {

    l_master.setSelectedSensorPosition(0);
    r_master.setSelectedSensorPosition(0);
    // reset gyro Yaw
    gyro.zeroYaw();
    // reset encoders to zero
    l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
  }

  @Override
  public void autonomousPeriodic() {

  }

  public double getTurnPIDOutput(double setpoint) {

    // error is the distance from where we want to go from where we are now
    // double error = RoboMath.getAngleToDegree(setpoint,
    // RoboMath.toUnitCircleDegrees(gyro.getYaw()));
    double error = setpoint;

    /*if (setpoint - RoboMath.toUnitCircleDegrees(gyro.getYaw()) > 0)
      error = setpoint - RoboMath.toUnitCircleDegrees(gyro.getYaw());
    else
      error = (setpoint - RoboMath.toUnitCircleDegrees(gyro.getYaw())) + 360;
      
    if (error > 180) {
      error -= 360;
    }
  */
    // calculate proportion value
    double p = turnpid.kP * error;

    // i_zone for perfecting distance to target
    if (Math.abs(error) <= turnpid.i_zone || turnpid.i_zone == 0.0f) {
      turnpid.i_state = turnpid.i_state + (error * turnpid.kI);
    } else {
      turnpid.i_state = 0;
    }
    double d = (error - turnpid.prev_err);
    turnpid.prev_err = error;
    d *= turnpid.kD;

    // static feed forward value based on how far we need to go
    double f = error * turnpid.kF;

    // add up all of our values for our output
    double output = p + d + turnpid.i_state;
    return output;
  }

  public double getDrivePIDOutput(double setpoint) {

    // error is the distance from where we want to go from where we are now
    double err1 = setpoint;
    // calculate proportion value
    double p1 = drivepid.kP * err1;

    if (Math.abs(err1) <= drivepid.i_zone || drivepid.i_zone == 0.0f) {
      drivepid.i_state = drivepid.i_state + (err1 * drivepid.kI);
    } else {
      drivepid.i_state = 0;
    }

    double d1 = (err1 - drivepid.prev_err);
    drivepid.prev_err = err1;
    d1 *= drivepid.kD;

    // static feed forward value based on how far we need to go
    double f1 = err1 * drivepid.kF;

    // add up all of our values for our output
    double driveOutput = p1;

    double output = driveOutput;
    // make sure the output is not greater than our max or less than our min
    // System.out.println(output);
    // double final_output = fminf(fmaxf(output, pid.Kminoutput), pid.Kmaxoutput);;
    return output;

    // set power
  }

  @Override
  public void testPeriodic() {
  }

  public void updatePosition() {
    RoboMath.calculateCoordinatePosition(l_master.getSelectedSensorPosition(0), r_master.getSelectedSensorPosition(0),
        gyro.getYaw());
    RoboMath.calculateLeftRightCoordinates(RoboMath.toUnitCircleDegrees(gyro.getYaw()));
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Velocity l", l_master.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Velocity r", r_master.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("Bus voltage L", l_master.getMotorOutputVoltage());
    SmartDashboard.putNumber("Bus voltage R", r_master.getMotorOutputVoltage());
  }
}