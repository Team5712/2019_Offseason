package frc.robot;

/**
 * Const
 */
public class Const {

    public static final double encoderToIn = 12.57f / 517;
    public static final double robotRadius = 13.5;

    public static final double THRESH_TURN = 3.5;
    public static final double THRESH_DRIVE = 5;

    // this is the amount the field will be scaled up in inches
    // eg. 12 will be 1 foot, 1 will be 1 inch, (1/2) will be half an inch
    public static final int FIELD_SCALE = 24;

    public static final int WIDTH = 10;
    public static final int HEIGHT = 10;

    public static double PATHFINDING_SPEED = 0.75;

}