package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * VisionProcessing
 */
public class VisionProcessing {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-one");;

    public static double getWidth() {
        return table.getEntry("thor").getDouble(0.0);
    }

    public static double getXOffset() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getDistanceToTarget() {
        double width = getWidth();

        if(width == 0) {
            System.out.println("TARGET NOT FOUND");
            return 0.0;
        }

        return (4873.8 / width) - 1.46526;
    }

    public static double getAngleToTarget() {
        return 90 - getXOffset();
    }


    public static double[] getCoordinateOffsetFromTarget() {
        double distance = getDistanceToTarget();

        double x = (Math.cos(Math.toRadians(getAngleToTarget())) * distance);
        double y = (Math.sin(Math.toRadians(getAngleToTarget())) * distance);

        return new double[] {x, y};
    }
}