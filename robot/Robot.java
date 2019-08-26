/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.RoboMath;
import frc.robot.pathfinding.AStar;
import frc.robot.pathfinding.Node;

public class Robot extends IterativeRobot {

    WPI_TalonSRX l_master = new WPI_TalonSRX(1);
    WPI_VictorSPX l_slave = new WPI_VictorSPX(2);
    WPI_TalonSRX r_master = new WPI_TalonSRX(5);
    WPI_VictorSPX r_slave = new WPI_VictorSPX(6);

    DifferentialDrive drive = new DifferentialDrive(l_master, r_master);

    Joystick joystick_l = new Joystick(0);
    Joystick joystick_r = new Joystick(1);

    AHRS gyro = new AHRS(Port.kMXP);

    Node startingNode;

    double xPosition;
    double yPosition;

    PID turnpid;
    PID drivepid;

    RoboMath RoboMath;


    @Override
    public void robotInit() {

        RoboMath = new RoboMath();

        gyro.zeroYaw();

        l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        turnpid = new PID(.025, 0, 0, 0, 0);
        drivepid = new PID(.05, 0, 0, 0, 0);

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
        l_master.setSelectedSensorPosition(0);
        r_master.setSelectedSensorPosition(0);

        startingNode = new Node(2, 0);

        xPosition = startingNode.getX() * Const.FIELD_SCALE;
        yPosition = startingNode.getY() * Const.FIELD_SCALE;

        gyro.zeroYaw();
        l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    }

    
    @Override
    public void teleopPeriodic() {
        updateDashboard();

        // implementation of pathfinding
        if (joystick_r.getRawButton(1)) {
            int calX = (int) Math.rint(this.xPosition / 48);
            int calY = (int) Math.rint(this.yPosition / 48);
            int[][] blocked = {{0, 2}, {1, 3}, {1, 2}, {2, 3}};
            int width = 6;
            int height = 6;
            Node start = new Node(calX, calY);
            Node end = new Node(0, 4);
            AStar astar = new AStar(height, width, start, end);
            astar.setBlocks(blocked);
            List<Node> listing = astar.findPath();
            double[][] points = new double[listing.size()][2];

            for (int i = 0; i < listing.size(); i++) {
                points[i][0] = listing.get(i).getX() * 48;
                points[i][1] = listing.get(i).getY() * 48;
                System.out.println("x: "+listing.get(i).getX()+" y:// "+listing.get(i).getY());
            }

            double[] motorOutput = driveToPoints(points);

            drive.tankDrive(motorOutput[0], motorOutput[1]);

        }

        else {
            drive.tankDrive(-joystick_l.getY(), -joystick_r.getY());
        }
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

    /**
     * will return motor output to spin on a dime using PID
     * 
     * @param setpoint
     * @return
     */
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

    /**
     * will return the motor output for a given setpoint using PID
     * 
     * @param setpoint
     * @return
     */
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

    /**
     * displays some general info to the smartdashboard
     * 
     * TODO: create a script on the driver station client with a gui for our grid
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("Velocity l", l_master.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Velocity r", r_master.getSelectedSensorVelocity(0));
        SmartDashboard.putNumber("Heading", gyro.getYaw());
        SmartDashboard.putNumber("Bus voltage L", l_master.getMotorOutputVoltage());
        SmartDashboard.putNumber("Bus voltage R", r_master.getMotorOutputVoltage());
    }

    /**
     * runs some calculations using RoboMath and updates the robots current position
     * this must be called on every iteration to update the position!
     */
    public void updatePosition() {
        // Gets current x and y positions
        double coordinates[] = RoboMath.calculateCoordinatePosition(xPosition, yPosition,
                l_master.getSelectedSensorPosition(0), r_master.getSelectedSensorPosition(0), gyro.getYaw());
        this.xPosition = coordinates[0];
        this.yPosition = coordinates[1];
        gyro.getYaw();
    }

    public double getDistance(double coordinate[]) {
        double[] points = { coordinate[0], coordinate[1] };
        double[] getError = RoboMath.calculateRelativeAngle(this.xPosition, this.yPosition,
                RoboMath.toUnitCircleDegrees(gyro.getYaw()), points);
        return getError[0];
    }

    /**
     * drive to each point in the given array of [x, y] pairs
     * 
     * @param coordinateSequence
     */
    public double[] driveToPoints(double coordinateSequence[][]) {
        double[] output;

        if (coordinateSequence.length != 1) {
            output = driveToPoint(coordinateSequence[1]);
        } else {
            output = driveToPoint(coordinateSequence[0]);
        }

        return output;
    }

    /**
     * turn towards the given point if it's outside of our threshold range and drive towards it once
     * we are within it. This is effectively a "point and shoot" method of iterating our points
     * 
     * TODO: smooth out our driving process by removing the threshold
     * 
     * This method derives the angle to turn from RoboMath.calulateRelativeAngle
     * 
     * @param point
     * @return
     */
    public double[] driveToPoint(double[] point) {

        double[] getError = RoboMath.calculateRelativeAngle(this.xPosition, this.yPosition,
                RoboMath.toUnitCircleDegrees(gyro.getYaw()), point);
        double angleError = getError[1];
        double distanceError = getError[0];
        // Calulates the speed it needs to turn at and drive speed using PID
        double turnSpeed = this.getTurnPIDOutput(angleError);
        double driveSpeed = this.getDrivePIDOutput(distanceError);
        System.out.println("dist " + distanceError);
        // Checks to see if the angle error is with inside the tolerance degrees if
        // not it will turn then continue to drive forwards
        double tolerance = 8;

        if (driveSpeed > 1)
            driveSpeed = 1;
        if (driveSpeed < -1)
            driveSpeed = -1;

        double leftDrive;
        double rightDrive;

        if (Math.abs(angleError) < tolerance && Math.abs(distanceError) > 2) {
            rightDrive = (driveSpeed * Const.PATHFINDING_SPEED + turnSpeed);
            leftDrive = (driveSpeed * Const.PATHFINDING_SPEED - turnSpeed);
        } else {
            rightDrive = turnSpeed;
            leftDrive = -turnSpeed;
        }

        return new double[] { leftDrive, rightDrive };
    }
}