package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
//import frc.robot.subsystems.Drivetrain;

public class TalonFXUtil {

    private static final int kCountsPerRev = 2048;
    private static final int k100msPerSecond = 10;

    // Helper methods to convert between meters and native units
    public static int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/DriveConstants.kWheelCircumference;
        double motorRotations = wheelRotations * DriveConstants.kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }

    public static int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/DriveConstants.kWheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond * DriveConstants.kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }

    public static double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / DriveConstants.kGearRatio;
        double positionMeters = wheelRotations * DriveConstants.kWheelCircumference;
        return positionMeters;
    }
}
