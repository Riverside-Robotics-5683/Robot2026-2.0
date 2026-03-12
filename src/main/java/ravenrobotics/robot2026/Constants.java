package ravenrobotics.robot2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static class AutoConstants {
        public static RobotConfig PATHPLANNER_CONFIG;

        static {
            try {
                PATHPLANNER_CONFIG = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    public static class FeederAndIntakeConstants {
        public static final int PIVOT_MOTOR = 20;
        public static final int INTAKE_MOTOR = 21;

        public static final int FEEDER_MOTOR = 4;

        public static final double PIVOT_KP = 1.0;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.5;

        public static final double PIVOT_IN = 2;
        public static final double PIVOT_OUT = 15.8;
    }

    public static class FlywheelAndHoodConstants {
        public static final int LEFT_FLYWHEEL_MOTOR = 5;
        public static final int CENTER_FLYWHEEL_MOTOR = 6;
        public static final int RIGHT_FLYWHEEL_MOTOR = 7;

        public static final int COLUMN_MOTOR = 8;

        public static final double FLYWHEEL_KP = 0.025;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 4.75;

        public static final double FLYWHEEL_IDLE = 2500;
    }

    public static class VisionConstants {
        public static final String FLYWHEEL_CAMERA = "flywheelCamera";

        public static final Transform3d FLYWHEEL_CAMERA_OFFSET = new Transform3d(
            Inches.of(-1.210522),
            Inches.of(-0.026610),
            Inches.of(22.136655),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(18.1),
                Degrees.of(0)
            )
        );

        public static final Matrix<N3, N1> singleTagDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
