package frc;

import java.util.Arrays;

import com.sun.jdi.Value;

import frc.robot.Const;


/**
 * RoboMath
 */
public class RoboMath {
  //Zeros out all values

    private double oldRight = 0;
    private double oldLeft = 0;
    private double distance = 0;
    
    
    /**
     * converts the giving gyro heading to unit circle degrees
     * 
     * The NavX gyro returns angle in a range from left to right as
     * -180:0 and 0:180
     * 
     * this will convert the range into unit circle degrees
     * where forward is 90 degrees
     * 
     * @param heading
     * @return
     */
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
     * calculates our change in position given our previous position, current, and the angle changed.
     * this method calculates the delta but will return the new absolute position.
     * 
     * @param encoderLeftCurrent
     * @param encoderRightCurrent
     * @param angle
     */
    public double[] calculateCoordinatePosition(double x, double y, double encoderLeftCurrent, double encoderRightCurrent, double angle) {
      //Changes angle to unit circle
      double gyroHeading = toUnitCircleDegrees(angle);
      double leftChange = -encoderLeftCurrent + oldLeft;
      double rightChange = -encoderRightCurrent + oldRight;
      //Finds average change of encoders from last cycle multiplied by encoder to inches so unit are in inches
      double position = (leftChange - rightChange)/2 * Const.encoderToIn;
      distance = position;
      //Calulates x and y 
      x += Math.cos(Math.toRadians(gyroHeading)) * distance;
      y += Math.sin(Math.toRadians(gyroHeading)) * distance;
      //Set old value to current
      oldLeft = encoderLeftCurrent;
      oldRight = encoderRightCurrent;

      return new double[] {x, y};
    }

    /**
     * will calculate 
     * 
     * 
     * @param x
     * @param y
     * @param c_angle
     * @param goalxy
     * @return
     */
    public double[] calculateRelativeAngle(double x, double y, double c_angle,double[] goalxy){ 
        //Sets angle to something out side of range so we can check cases
        double angle = 400;
        //Find how far off our robot is from point by subtracting our goal from our current position
        double changeInX = goalxy[0]-x;
        double changeInY = goalxy[1]-y;
        //Checks to see if we may be at a 0, 90, 180, or 270 as using atan doesn't work as they include a 0
        if((changeInX==0)&&(changeInY>=0)){
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
          double distance = direction*Math.sqrt(Math.pow(goalxy[0]-x, 2)+Math.pow(goalxy[1]-y, 2));  
          double [] distAndAngle = {distance,angle};
          // System.out.println("x:" + x + " y:" + y);
          return distAndAngle;
    }
}