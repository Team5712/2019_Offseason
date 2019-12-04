/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.RoboMath;
import frc.robot.pathfinding.AStar;
import frc.robot.pathfinding.Node;
import frc.robot.vision.VisionProcessing;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends IterativeRobot {

    WPI_TalonSRX l_master = new WPI_TalonSRX(1);
    WPI_VictorSPX l_slave = new WPI_VictorSPX(2);
    WPI_TalonSRX r_master = new WPI_TalonSRX(5);
    WPI_VictorSPX r_slave = new WPI_VictorSPX(6);

    DifferentialDrive drive = new DifferentialDrive(l_master, r_master);

    Joystick joystick_l = new Joystick(0);
    Joystick joystick_r = new Joystick(1);

    NetworkTableServer networkTableServer;

    AHRS gyro = new AHRS(Port.kMXP);

    AStar astar;
    Node startingNode;

    double previousTime;
    Timer timer;

    double xPosition;
    double yPosition;

    double angle;

    PID turnpid;
    PID drivepid;

    RoboMath RoboMath;

    NetworkTableInstance instance;
    NetworkTable table;
    NetworkTableEntry clickX;

    List<Node> drivePoints;

    @Override
    public void robotInit() {

        timer = new Timer();

        RoboMath = new RoboMath();

        gyro.zeroYaw();

        l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        turnpid = new PID(.025, 0, 0, 0, 0);
        drivepid = new PID(.05, 0, 0, 0, 0);

        l_slave.follow(l_master);
        r_slave.follow(r_master);
        

        networkTableServer = new NetworkTableServer();
        networkTableServer.init();


        instance = NetworkTableInstance.getDefault();
        if (!instance.isConnected()) {
			// System.out.println("Not connected...");
		}
        table = instance.getTable("drivePoint");
        clickX = table.getEntry("click_x_position");

        drivePoints = new ArrayList<Node>();
        clickX.addListener((entry) -> {
            // System.out.println("clickx " + entry.value.getDoubleArray()[0] + " clicky" + entry.value.getDoubleArray()[1]);
            drivePoints.add(new Node((int)entry.value.getDoubleArray()[0], (int)entry.value.getDoubleArray()[1]));
		}, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        
        
        startingNode = new Node(0, 0);

        xPosition = startingNode.getX() * Const.FIELD_SCALE;
        yPosition = startingNode.getY() * Const.FIELD_SCALE;
        
        int[][] blocked = {};
        

        //int width = 324;
        //int height = 648;

        astar = new AStar(Const.HEIGHT, Const.WIDTH, startingNode, startingNode);
        astar.setBlocks(blocked);
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

        gyro.zeroYaw();
        l_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        r_master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        timer.start();
        
    }

    @Override
    public void teleopPeriodic() {
        double offset[] = VisionProcessing.getCoordinateOffsetFromTarget();

        System.out.println("x " + offset[0] + " y " + offset[1]);

        if(drivePoints.size() == 0) {
            
        } else {
            // double[] motorOutput = driveToPoints(drivePoints);
            
            // drive.tankDrive(motorOutput[0], motorOutput[1]);

            // System.out.println(xPosition + " cur x, " + yPosition);

            // for (Node var : drivePoints) {
                // System.out.println(var.getX() + " x, " + var.getY());
            // }

            // for (int i = 0; i < drivePoints.size(); i++) {
            //     System.out.println(i + "=i " + drivePoints.get(i).getX() + " x, " + drivePoints.get(i).getY());
            // }
            // System.out.println(this.xPosition + "=robotX " + this.yPosition + "=robotY");
            // System.out.println(";");
            // System.out.println(";");

            // System.out.println(this.xPosition - drivePoints.get(0).getX());
            // System.out.println(this.yPosition - drivePoints.get(0).getY());

            // if (Math.abs(xPosition - drivePoints.get(0).getX()) < .5 && Math.abs(yPosition - drivePoints.get(0).getY())< .5) {
            //     System.out.println(drivePoints.get(0).getX() +" remove " + drivePoints.get(0).getY());
            //     drivePoints.remove(0);
            // }

            double[] motorOutput = driveToPoint(new double[] {drivePoints.get(drivePoints.size() - 1).getX(), drivePoints.get(drivePoints.size() - 1).getY()});
            // double[] motorOutput = driveToPoint(new double[] {80, 80});
            drive.tankDrive(motorOutput[0], motorOutput[1]);
            networkTableServer.sendPackets(this);
        }

        
        // System.out.println(motorOutputFromGUI[0] + " "+ motorOutputFromGUI[1]);
        // drive.tankDrive(motorOutputFromGUI[0], motorOutputFromGUI[1]);
        // System.out.println("delta: " + ((timer.get() - previousTime) * 1000) + " ms");
        // previousTime = timer.get();


        // updateDashboard();

        // // System.out.println("x " + this.xPosition + " y " + this.yPosition);

        // // implementation of pathfinding
        // if (joystick_r.getRawButton(1)) {
        //     int calX = (int) Math.rint(this.xPosition / Const.FIELD_SCALE);
        //     int calY = (int) Math.rint(this.yPosition / Const.FIELD_SCALE);

        //     Node start = new Node(calX, calY);
        //     Node end = new Node(1, 5);

        //     astar = new AStar(Const.HEIGHT, Const.WIDTH, start, end);
        //     astar.setBlocks(new int[][] {{1, 4},{1, 3},{0, 4},{2, 4},});

        //     List<Node> points = AStar.transformGrid(astar.findPath());


        //     System.out.println("x " + this.xPosition + " y " + this.yPosition);
        //     System.out.println("going to " + points);
            

        //     if(points.size() == 0) {
        //         System.out.println("No possible path to " + end.toString());
        //     } else {
        //         double[] motorOutput = driveToPoints(points);

        //         drive.tankDrive(motorOutput[0], motorOutput[1]);
        //     }
        // }

        // else {
        //     drive.tankDrive(-joystick_l.getY(), -joystick_r.getY());
        // }

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

        this.angle = RoboMath.toUnitCircleDegrees(gyro.getYaw());
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
        double[] output = null;

        // if (coordinateSequence.length != 1) {
        //     output = driveToPoint(coordinateSequence[1]);
        // } else {
        //     output = driveToPoint(coordinateSequence[0]);
        // }


        output = driveToPoint(coordinateSequence[coordinateSequence.length - 1]);


        return output;
    }

    /**
     * drive to each point in the given array of [x, y] pairs
     * 
     * @param coordinateSequence
     */
    public double[] driveToPoints(List<Node> coordinateSequence) {
        double[] output;
        
        if (coordinateSequence.size() != 1) {
            output = driveToPoint(coordinateSequence.get(1).toDoubleArray());
        } else {
            output = driveToPoint(coordinateSequence.get(0).toDoubleArray());
        }

        return output;
    }

    /**
     * turn towards the given point if it's outside of our threshold range and drive
     * towards it once we are within it. This is effectively a "point and shoot"
     * method of iterating our points
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

        // System.out.println(distanceError + " distance Error");

        // Calulates the speed it needs to turn at and drive speed using PID
        double turnSpeed = this.getTurnPIDOutput(angleError);
        double driveSpeed = this.getDrivePIDOutput(distanceError);

        // System.out.println(turnSpeed + ":turnSpeed  " + driveSpeed + ":driveSpeed");


        // Checks to see if the angle error is with inside the tolerance degrees if
        // not it will turn then continue to drive forwards
        double tolerance = 15;

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

        // System.out.println(leftDrive + ":leftDrive  " + rightDrive + ":rightDrive");
        return new double[] { leftDrive, rightDrive };
    }
}