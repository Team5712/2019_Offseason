package frc.robot;

/**
 * Values
 */
public class Values {

    public static double oldLeft = 0;
    public static double oldRight = 0;
    public static double distance = 0;

    public static double lx = 0;
    public static double ly = 0;

    public static double rx = 0;
    public static double ry = 0;

    public static double robotPositionX;
    public static double robotPositionY;

    public static double[] xPointsLeft = new double[25];
    public static double[] yPointsLeft = new double[25];

    public static double[] xPointsRight = new double[25];
    public static double[] yPointsRight = new double[25];

    public double getXPosition()
    {
        return robotPositionX;
    }
    public double getYPosition()
    {
        return robotPositionY;
    }

}