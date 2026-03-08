package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {
    public static class IntakeConstants {
        public static final int kBottomIntakeMotor = 20;
        public static final int kTopIntakeMotor = 21;

        public static final int kRightArmMotor = 22;
        public static final int kLeftArmMotor = 23;
    }
    public static class FeederConstants {
        public static final int kBottomFeederMotor = 24;
        public static final int kTopFeederMotor = 25;
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
        public static class Red{
            public static final Translation2d kGoalPosition = new Translation2d(11.90, 4.02);
            public static final Translation2d kRedPassTestSPot = new Translation2d(16.50,5.00); //this is mainly for passing, not accurate!
            public static final Translation2d kRedPassTestSPot2 = new Translation2d(16.50,1); //this is mainly for passing, not accurate!
        }
    }
    public static class ShooterInterpolationConstants{
        public static final InterpolatingDoubleTreeMap rpmMAP = new InterpolatingDoubleTreeMap();
        static{ //KEY: DISTANCE METERS,  VALUE: SHOOTER RPM
            rpmMAP.put(4.94, 4800.0);
            rpmMAP.put(3.9, 4200.0);
            rpmMAP.put(2.9, 3500.0);
            rpmMAP.put(2.41, 3200.0);
            rpmMAP.put(2.08, 3200.0);
        }
    }
}
