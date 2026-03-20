package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static class IntakeConstants {
        public static final int kBottomIntakeMotor = 20;
        public static final int kTopIntakeMotor = 21;

        //public static final int kRightArmMotor = 22;
        public static final int kWinchMotor = 22;
    }
    public static class FeederConstants {
        public static final int kBottomFeederMotor = 24;
        public static final int kTopFeederMotor = 25;
    }
    public static class AgitatorConstants {
        public static final int kLeftAgitatorMotor = 40;
        public static final int kRightAgitatorMotor = 41;
    }
    public static class ShooterConstants {
        public static final int kRightShooterMotor = 30;
        public static final int kLeftShooterMotor = 31;

        public static final int kRotateMotor = 32;
        public static final int kHoodMotor = 33;

        public static final int kShooterCANdi = 34;
        public static final double kRotateMotorOffset = 0.223;
        public static final double kRotatateMin = 90;
        public static final double kRotatateMax = 270;
    }
    public static class FieldConstants {
        //These are all for blue side. We use a Util function to convert to red
        public static final Translation2d kHubPosition = new Translation2d(4.61, 4.02);
        public static final Translation2d kPassTestSpotLeft = new Translation2d(.02,7); 
        public static final Translation2d kPassTestSpotRight = new Translation2d(0.02,1);

    }
    public static class ShooterInterpolationConstants{
        public static final InterpolatingDoubleTreeMap rpmMAP = new InterpolatingDoubleTreeMap();
        static{ //KEY: DISTANCE METERS,  VALUE: SHOOTER RPM
            rpmMAP.put(5.10, 5400.0);
            rpmMAP.put(4.94, 4800.0);
            rpmMAP.put(4.4, 4500.0);
            rpmMAP.put(3.9, 4200.0);
            rpmMAP.put(2.9, 3500.0);
            rpmMAP.put(2.41, 3200.0);
            rpmMAP.put(2.08, 3200.0);
        }
    }
}
