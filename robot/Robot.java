/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.*;

import javax.lang.model.util.ElementScanner6;

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
  int coordIndex = 0;
  // Find ratio encoders to inches
  double encoderToIn = 12.57f / 517;
  double distance = 0.0f;
  double oldLeft = 0.0f;
  double oldRight = 0.0f;
  double maxspeed = 0;
  double leftDrive, rightDrive;

  double max_spd_l;
  double max_spd_r;
  double max_err;

  

  PID turnpid;
  PID drivepid;

  int timeout = 20;

  RoboMath RoboMath;

  @Override
  public void robotInit() {

    RoboMath  = new RoboMath();
    // robot init is run when your robot is turned on
    // reset gyro Yaw
    gyro.zeroYaw();
    // reset encoders to zero
    l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    // invert drive motors because they were running the wrong way
    l_master.setInverted(true);
    r_master.setInverted(true);

    //i = .008 iz = 5
    turnpid = new PID(.03, 0, 0, 0, 0);
    //l_master.config_kF(0,2.17659);
    // .4
    drivepid = new PID(.2, 0, 0, .1, 0);

    l_slave.follow(l_master);
    r_slave.follow(r_master);

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
    //sets PID values to 0
    l_master.selectProfileSlot(0, 0);
    
    l_master.config_kF(0, 0, timeout);
    l_master.config_kP(0, 0, timeout);
    l_master.config_kI(0, 0, timeout);
    l_master.config_kD(0, 0, timeout);

    r_master.selectProfileSlot(0, 0);

    r_master.config_kF(0, 0, timeout);
    r_master.config_kP(0, 0, timeout);
    r_master.config_kI(0, 0, timeout);
    r_master.config_kD(0, 0, timeout);

  }

  @Override
  public void teleopPeriodic() {

    double coordinateSequence[][] = {{3 * 24, 4 * 24}, {1 * 24, 8 * 24}, {-2 * 24, 19 * 24}, {-3 * 24, 11 * 24}, {-1 * 24, 2 * 24}, {0, 0}};

    //Checks for button to be pushed
    if (joystick_l.getRawButton(1)==true){
      
      if(coordIndex < coordinateSequence.length) {
        
        double[] getError = RoboMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),coordinateSequence[coordIndex]);
        double angleError = getError[1];
        double distanceError = getError[0];
        //Calulates the speed it needs to turn at and drive speed using PID
        double turnSpeed = this.getTurnPIDOutput(angleError);
        double driveSpeed = this.getDrivePIDOutput(distanceError);
        System.out.println("Angle: "+getError[1]);
        System.out.println("Distance: "+getError[0]);
        //Checks to see if the angle error is with inside the tolerance degrees if not it will turn then continue to drive forwards
        double tolerance = 10;
        if(Math.abs(angleError)<tolerance && Math.abs(distanceError) > 1)
        {
          rightDrive = (driveSpeed+turnSpeed);
          leftDrive = (driveSpeed-turnSpeed);
        }
        else
        {
          rightDrive = turnSpeed;
          leftDrive = -turnSpeed;
        }
  
        if(Math.abs(getError[0]) < 6) {
          coordIndex += 1;
        }
      }
      
      drive.tankDrive(leftDrive, rightDrive);
      

      //driveCurve(coordinateSequence);
      /*
      //Ask for destination point
      double driveSpeeds[] = driveToPoint(3*24, 6*24);
      rightDrive = driveSpeeds[0];
      leftDrive = driveSpeeds[1];
      //Applys power to drive train
      drive.tankDrive(leftDrive, rightDrive);
      */
  } 
  //if button isn't pushed allow you to drive
  else{
    drive.tankDrive(-joystick_l.getY(),-joystick_r.getY());
  }
    //Updates our dashboard
    this.updateDashboard();
    //Gets the current x and y position
    this.updatePosition();
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


  public void updateDashboard() {
    //Displays values on our dashboard
    SmartDashboard.putNumber("Velocity l", l_master.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Velocity r", r_master.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("Bus voltage L", l_master.getMotorOutputVoltage());
    SmartDashboard.putNumber("Bus voltage R", r_master.getMotorOutputVoltage());
  }

  public void updatePosition() {
    //Gets current x and y positions
    RoboMath.calculateCoordinatePosition(l_master.getSelectedSensorPosition(0), r_master.getSelectedSensorPosition(0),
    gyro.getYaw());
  }

  public double[] driveToPoint(double x, double y) {
    //Gets the error that is required to get robot to correct angle and distance to point
    double[] points = {x, y};
  
    //getError[0] is distance getError[1] is angle
    double[] getError = RoboMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),points);
    double angleError = getError[1];
    double distanceError = getError[0];
    //Calulates the speed it needs to turn at and drive speed using PID
    double turnSpeed = this.getTurnPIDOutput(angleError);
    double driveSpeed = this.getDrivePIDOutput(distanceError);
    System.out.println("Angle: "+getError[1]);
    System.out.println("Distance: "+getError[0]);
    //Checks to see if the angle error is with inside the tolerance degrees if not it will turn then continue to drive forwards
    double tolerance = 10;
    if(Math.abs(angleError)<tolerance && Math.abs(distanceError) > 1)
    {
    rightDrive = (driveSpeed+turnSpeed);
    leftDrive = (driveSpeed-turnSpeed);
    }
    else
    {
      rightDrive = turnSpeed;
      leftDrive = -turnSpeed;
    }
    double[] driveSpeeds = {rightDrive,leftDrive};
    return driveSpeeds;
  }

  public double getDistance(double coordinate[]) {
    double[] points = {coordinate[0], coordinate[1]};
    double[] getError = RoboMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),points);
    return getError[0];
  }

  public double[] driveToPoint(double coordinate[]) {
    //Gets the error that is required to get robot to correct angle and distance to point
    double[] points = {coordinate[0], coordinate[1]};
  
    //getError[0] is distance getError[1] is angle
    double[] getError = RoboMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),points);
    double angleError = getError[1];
    double distanceError = getError[0];
    //Calulates the speed it needs to turn at and drive speed using PID
    double turnSpeed = this.getTurnPIDOutput(angleError);
    double driveSpeed = this.getDrivePIDOutput(distanceError);
    System.out.println("Angle: "+getError[1]);
    System.out.println("Distance: "+getError[0]);
    //Checks to see if the angle error is with inside the tolerance degrees if not it will turn then continue to drive forwards
    double tolerance = 10;
    if(Math.abs(angleError)<tolerance && Math.abs(distanceError) > 1)
    {
    rightDrive = (driveSpeed+turnSpeed);
    leftDrive = (driveSpeed-turnSpeed);
    }
    else
    {
      rightDrive = turnSpeed;
      leftDrive = -turnSpeed;
    }
    double[] driveSpeeds = {rightDrive,leftDrive};
    return driveSpeeds;
  }

  /*
  public double[] driveCurve(double curve[][]) {

    double[] getError = RoboMath.calculateRelativeAngle(RoboMath.toUnitCircleDegrees(gyro.getYaw()),coordinateSequence[coordIndex]);
    double angleError = getError[1];
    double distanceError = getError[0];
    //Calulates the speed it needs to turn at and drive speed using PID
    double turnSpeed = this.getTurnPIDOutput(angleError);
    double driveSpeed = this.getDrivePIDOutput(distanceError);
    System.out.println("Angle: "+getError[1]);
    System.out.println("Distance: "+getError[0]);
    //Checks to see if the angle error is with inside the tolerance degrees if not it will turn then continue to drive forwards
    double tolerance = 10;
    if(Math.abs(angleError)<tolerance && Math.abs(distanceError) > 1)
    {
      rightDrive = (driveSpeed+turnSpeed);
      leftDrive = (driveSpeed-turnSpeed);
    }
    else
    {
      rightDrive = turnSpeed;
      leftDrive = -turnSpeed;
    }

    if(getError[0] < 6) {
      coordIndex += 1;
    }

    double drive[] = {leftDrive, rightDrive};
    return drive;
  }   
*/
}