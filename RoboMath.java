package frc;

import java.util.Arrays;

import com.sun.jdi.Value;

import frc.robot.Const;


/**
 * RoboMath
 */
public class RoboMath {
  //Zeros out all values
    private double xPosition;
    private double yPosition;
    private double oldRight = 0;
    private double oldLeft = 0;
    private double distance = 0;
    private double lx = 0;
    private double ly = 0;
    private double rx = 0;
    private double ry = 0;
    
    //Changes gyro to unit circle
    public double toUnitCircleDegrees(double heading) {
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
    public void calculateCoordinatePosition(double encoderLeftCurrent, double encoderRightCurrent, double angle) {
      //Changes angle to unit circle
      double gyroHeading = toUnitCircleDegrees(angle);
      double leftChange = -encoderLeftCurrent + oldLeft;
      double rightChange = -encoderRightCurrent + oldRight;
      //Finds average change of encoders from last cycle multiplied by encoder to inches so unit are in inches
      double position = (leftChange - rightChange)/2 * Const.encoderToIn;
      distance = position;
      //Calulates x and y 
      xPosition += Math.cos(Math.toRadians(gyroHeading)) * distance;
      yPosition += Math.sin(Math.toRadians(gyroHeading)) * distance;
      //Set old value to current
      oldLeft = encoderLeftCurrent;
      oldRight = encoderRightCurrent;
    }


    //Calculates angle to point and distance
    public double[] calculateRelativeAngle(double c_angle,double[] goalxy){ 
        //Sets angle to something out side of range so we can check cases
        double angle = 400;
        //Find how far off our robot is from point by subtracting our goal from our current position
        double changeInX = goalxy[0]-xPosition;
        double changeInY = goalxy[1]-yPosition;
        //Checks to see if we may be at a 0, 90, 180, or 270 as using atan doesn't work as they include a 0
        if((changeInX==0)&&(changeInY>0)){
          angle = 90;
        }
        if((changeInX==0)&&(changeInY<0)){
          angle = 270;
        }
        if((changeInX>0)&&(changeInY==0)){
          angle = 180;
        }
        if((changeInX>0)&&(changeInY==0)){
          angle = 0;
        }
        //Calculates the angle that we want to get to if not found above
        if(angle==400){
          angle = Math.toDegrees(Math.atan(Math.abs(changeInY)/Math.abs(changeInX)));
          if(changeInX<0&&changeInY>0){
            angle = 180-angle;
          }
          if(changeInX<0&&changeInY<0){
            angle = 180+angle;
          }
          if(changeInX>0&&changeInY<0){
            angle = 360-angle;
          }
        }
        //This checks to see if our target is in front of us or behind us
        double direction = 1;
        if(angle - c_angle>90||angle - c_angle<-90){
          direction = -1;
        }
        //Calulates the angle that we need to turn to get to our point
        angle = angle - c_angle;
          if(angle<=-180){
            angle+=180;
          }
          if(angle<90){
            angle=180+angle;
          }
          if(angle>90){
            angle=angle-180;
          }
          //Calculates the distance we need to drive to get to our point
          double distance = direction*Math.sqrt(Math.pow(goalxy[0]-xPosition, 2)+Math.pow(goalxy[1]-yPosition, 2));  
          double [] distAndAngle = {distance,angle};
          System.out.println("x:" + xPosition + " y:" + yPosition);
          return distAndAngle;
    }

    public double getYPosition()
    {
        return yPosition;
    }
    public double getXPosition()
    {
        System.out.println("ROBOTMATH X POSTION GET: " + xPosition);
        return xPosition;
    }
}