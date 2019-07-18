package frc;

import java.util.Arrays;

import com.sun.jdi.Value;

import frc.robot.Const;
import frc.robot.Values;

/**
 * RoboMath
 */
public class RoboMath {

    // Calculate coordinate place


    // public void calculatePathPoints(double targetX, double targetY, double
    // startSpeed, double endSpeed,
    // double startAngle, double endAngle) {

    // double endLeftX = Values.robotPositionX + Math.cos(Math.toRadians(endAngle +
    // 90)) * Const.robotRadius;
    // double endLeftY = Values.robotPositionY + Math.sin(Math.toRadians(endAngle +
    // 90)) * Const.robotRadius;

    // double endRightX = Values.robotPositionX + Math.cos(Math.toRadians(endAngle -
    // 90)) * Const.robotRadius;
    // double endRightY = Values.robotPositionY + Math.sin(Math.toRadians(endAngle -
    // 90)) * Const.robotRadius;

    // // This is used to calculate the gray points that are shown in desmos
    // double x2 = startSpeed * Math.cos(Math.toRadians(startAngle)) + Values.lx;
    // double y2 = startSpeed * Math.sin(Math.toRadians(startAngle)) + Values.ly;
    // double x4 = endSpeed * Math.cos(Math.toRadians(endAngle + 180)) + targetX;
    // double y4 = endSpeed * Math.sin(Math.toRadians(endAngle + 180)) + targetY;

    // // Calculate 1000 x and y points
    // double counter = 0;
    // while (counter < 0.025) {
    // int placement = (int) (counter * 1000);

    // Values.xPointsLeft[placement] = Math.pow((1 - counter), 3) * Values.lx
    // + 3 * counter * Math.pow(1 - counter, 2) * endLeftX + 3 * Math.pow(counter,
    // 2) * (1 - counter) * x2
    // + Math.pow(counter, 3) * x4;
    // Values.yPointsLeft[placement] = Math.pow((1 - counter), 3) * Values.ly
    // + 3 * counter * Math.pow(1 - counter, 2) * endLeftY + 3 * Math.pow(counter,
    // 2) * (1 - counter) * y2
    // + Math.pow(counter, 3) * y4;

    // Values.xPointsRight[placement] = Math.pow((1 - counter), 3) * Values.rx
    // + 3 * counter * Math.pow(1 - counter, 2) * endRightX + 3 * Math.pow(counter,
    // 2) * (1 - counter) * x2
    // + Math.pow(counter, 3) * x4;
    // Values.yPointsRight[placement] = Math.pow((1 - counter), 3) *
    // Values.robotPositionY
    // + 3 * counter * Math.pow(1 - counter, 2) * endRightY + 3 * Math.pow(counter,
    // 2) * (1 - counter) * y2
    // + Math.pow(counter, 3) * y4;

    // counter += .001;
    // }

    // System.out.println("X: " + Arrays.toString(Values.xPointsLeft));
    // System.out.println("Y: " + Arrays.toString(Values.yPointsLeft));

    // }

    /**
     * calculates the coordinates of the left and rightmost sides of the robot
     * 
     * @param angle
     */
    public static void calculateLeftRightCoordinates(double angle) {
        Values.lx = Values.robotPositionX + Math.cos(Math.toRadians(angle + 90)) * Const.robotRadius;
        Values.ly = Values.robotPositionY + Math.sin(Math.toRadians(angle + 90)) * Const.robotRadius;

        Values.rx = Values.robotPositionX + Math.cos(Math.toRadians(angle - 90)) * Const.robotRadius;
        Values.ry = Values.robotPositionY + Math.sin(Math.toRadians(angle - 90)) * Const.robotRadius;
    }

    

    /**
     * calculate the angle between two points
     * 
     * @param coordinate
     * @return
     */
    public static double getAngleToPoint(double[] coordinate) {
        // pointOne will be our current position
        double[] pointOne = { Values.robotPositionX, Values.robotPositionY };
        System.out.println("X point: "+pointOne[0]);
        System.out.println("Y point: "+pointOne[1]);
        double[] pointTwo = coordinate;
        double angle = 0;
        double changeInX = pointTwo[0] - pointOne[0];
        double changeInY = pointTwo[1] - pointOne[1];
        
        if ((changeInX == 0) && (changeInY > 0)) {
            angle = 90;
        }
        if ((changeInX == 0) && (changeInY > 0)) {
            angle = 270;
        }
        if ((changeInX > 0) && (changeInY == 0)) {
            angle = 180;
        }

        if ((changeInX > 0) && (changeInY == 0)) {
            angle = 0;
        } else {

            angle = Math.toDegrees(Math.atan(Math.abs(changeInY) / Math.abs(changeInX)));

            if (changeInX < 0 && changeInY > 0) {
                angle = 180 - angle;
            }
            if (changeInX < 0 && changeInY < 0) {
                angle = 180 + angle;
            }
            if (changeInX > 0 && changeInY < 0) {
                angle = 360 - angle;
            }
        }

        return angle;
    }

    /**
     * get the delta angle between two given degrees
     * 
     * 
     * 
     * @param angle1 angle (in unit circle form)
     * @param angle2 angle (in unit circle form)
     * @return
     */
    public static double getAngleToDegree(double angle1, double angle2) {

        double error = 0;
        if (angle1 - (angle2) > 0)
            error = angle1 - (angle2);
        else
            error = angle1 - angle2 + 360;

        if (error > 180) {
            error -= 360;
        }

        return error;
    }

    /**
     * converts our gyro heading into an absolute position on the unit circle
     * 
     * @param heading
     * @return
     */
    public static double toUnitCircleDegrees(double heading) {
        // first quadrant
        if (heading >= 0 && heading <= 90) {
            return 90-heading;
            // second quadrant
        } else if (heading < 0) {
            return Math.abs(heading - 90);
            // third quadrant
        } 
         else if (heading > 90 && heading <= 180) {
            return 360 - (heading - 90);
        } else {
            return 0;
        }
    };

    /**
     * calculates our xy coordinates given our polar radius and angle
     * 
     * @param encoderLeftCurrent
     * @param encoderRightCurrent
     * @param angle
     */
    public static void calculateCoordinatePosition(double encoderLeftCurrent, double encoderRightCurrent, double angle) {
        // System.out.println("enc " + encoderLeftCurrent + " r " + encoderRightCurrent
        // + " angle " + angle);

        // System.out.println("l " + encoderLeftCurrent * encoderToIn);
        double gyroHeading = toUnitCircleDegrees(angle);
        double leftChange = -encoderLeftCurrent + Values.oldLeft;
        double rightChange = -encoderRightCurrent + Values.oldRight;
        // Finds change of encoder
        double position = (leftChange - rightChange) * Const.encoderToIn;
        Values.distance = position - Values.distance;
        Values.robotPositionX += Math.cos(Math.toRadians(gyroHeading)) * Values.distance;
        Values.robotPositionY += Math.sin(Math.toRadians(gyroHeading)) * Values.distance;
        // System.out.println("Gyro:"+gyroHeading+" robotVel: "+robotVelocity+"changed
        // Time: "+changedTime );
        // System.out.println("x position: "+robotPositionX+" y position:
        // "+robotPositionY);
        Values.oldLeft = encoderLeftCurrent;
        Values.oldRight = encoderRightCurrent;
    }
}