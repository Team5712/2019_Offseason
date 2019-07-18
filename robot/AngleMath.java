package frc.robot;

public class AngleMath
{
  
  Values values = new Values();

  public double[] calculateRelativeAngle(double c_angle,double[] goalxy){ 
      //Sets angle to something out side of range so we can check cases
      double angle=400;
      double changeInX = goalxy[0]-values.getXPosition();
      double changeInY = goalxy[1]-values.getYPosition();
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
  
      double direction = 1;
      if(angle - c_angle>90||angle - c_angle<-90){
        direction = -1;
      }
      System.out.println(angle - c_angle);
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
        double distance = direction*Math.sqrt(Math.pow(goalxy[0]-values.getXPosition(), 2)+Math.pow(goalxy[1]-values.getYPosition(), 2));
        System.out.println("Distance :"+distance);
        System.out.println("Goal Y: "+ goalxy[1]);
        double [] distAndAngle = {distance,angle};
        return distAndAngle;
  }
}