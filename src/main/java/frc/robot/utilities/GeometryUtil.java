package frc.robot.utilities;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class GeometryUtil {
    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Red;
    }
    
    public static Translation2d mirrorTranslationForRedAlliance(Translation2d blueTranslation) {
        return new Translation2d(
                blueTranslation.getX() +
                        Constants.FieldConstants.kFieldLength -
                        (2 * blueTranslation.getX()),

                blueTranslation.getY());
    }
}
