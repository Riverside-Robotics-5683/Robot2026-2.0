package ravenrobotics.robot2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class Constants {
    public static class AutoConstants {
        public static RobotConfig PATHPLANNER_CONFIG;

        public static final double AUTO_TRANSLATION_SPEED = 2.0;
        public static final double AUTO_ROTATION_SPEED = 1.0;

        static {
            try {
                PATHPLANNER_CONFIG = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    public static class IntakeConstants {
        public static final int INTAKE_MOTOR = 21;
    }

    public static class PivotConstants {
        public static final int PIVOT_MOTOR = 20;

        public static final double PIVOT_KP = 2.8;
        public static final double PIVOT_KI = 0.0;
        public static final double PIVOT_KD = 0.5;

        public static final double PIVOT_IN = 0;
        public static final double PIVOT_OUT = 15.8;

        public static final double PIVOT_SHOOT_HIGH = 5;
        public static final double PIVOT_SHOOT_LOW = 14;
    }

    public static class FeederConstants {
        public static final int FEEDER_MOTOR = 4;
    }

    public static class FlywheelConstants {
        public static final int LEFT_FLYWHEEL_MOTOR = 5;
        public static final int CENTER_FLYWHEEL_MOTOR = 6;
        public static final int RIGHT_FLYWHEEL_MOTOR = 7;

        public static final int COLUMN_MOTOR = 8;

        public static final double FLYWHEEL_KP = 0.0012;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.015;

        public static final double FLYWHEEL_KS = 1.24;
        public static final double FLYWHEEL_KV = 0.0015;
        public static final double FLYWHEEL_KA = 0.0;

        public static final double FLYWHEEL_HUB_IDLE = 1500;
        public static final double FLYWHEEL_PASS_IDLE = 3000;

        public static final double FLYWHEEL_SETPOINT_TOLERANCE = 400;

        public static final Translation2d BLUE_HUB_POS = new Translation2d(Meters.of(4.625), Meters.of(4.03));
        public static final Translation2d RED_HUB_POS = new Translation2d(Meters.of(11.91), Meters.of(4.03));

        public static final Translation2d BLUE_HIGH_POS = new Translation2d(Meters.of(2.401), Meters.of(6));
        public static final Translation2d BLUE_LOW_POS = new Translation2d(Meters.of(2.401), Meters.of(2.5));

        public static final Translation2d RED_HIGH_POS = new Translation2d(Meters.of(14.1), Meters.of(6));
        public static final Translation2d RED_LOW_POS = new Translation2d(Meters.of(14.1), Meters.of(2.5));

        public static final InterpolatingMatrixTreeMap<Double, N2, N1> HUB_SHOT_TREE = new InterpolatingMatrixTreeMap<>();
        public static final InterpolatingMatrixTreeMap<Double, N2, N1> PASS_SHOT_TREE = new InterpolatingMatrixTreeMap<>();

        static {
            HUB_SHOT_TREE.put(4.789, VecBuilder.fill(3935, 0.039));
            HUB_SHOT_TREE.put(0.83, VecBuilder.fill(2985, 0));
            HUB_SHOT_TREE.put(2.53, VecBuilder.fill(3285, 0.037));
            HUB_SHOT_TREE.put(1.945, VecBuilder.fill(2785, 0.036));
            HUB_SHOT_TREE.put(3.07, VecBuilder.fill(3285, 0.047));

            PASS_SHOT_TREE.put(5.4, VecBuilder.fill(4250, 0.04));
            PASS_SHOT_TREE.put(7.96, VecBuilder.fill(4250, 0.07));
            PASS_SHOT_TREE.put(11.76, VecBuilder.fill(5250, 0.08));
            PASS_SHOT_TREE.put(5.5, VecBuilder.fill(3750, 0.065));
        }
    }

    public static class HoodConstants {
        public static final int LEFT_ACTUATOR = 9;
        public static final int RIGHT_ACTUATOR = 10;
    }

    public static class VisionConstants {
        public static final String FLYWHEEL_CAMERA = "flywheelCamera";
        public static final String HOPPER_CAMERA = "hopperCamera";

        public static final Transform3d FLYWHEEL_CAMERA_OFFSET = new Transform3d(
            Inches.of(-1.210522),
            Inches.of(1.72339),
            Inches.of(22.136655),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(18.1),
                Degrees.of(0)
            )
        );

        public static final Transform3d HOPPER_CAMERA_OFFSET = new Transform3d(
            Inches.of(-1.210522),
            Inches.of(-1.77661),
            Inches.of(22.136655),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(2),
                Degrees.of(0)
            )
        );

        public static final Matrix<N3, N1> singleTagDevs = VecBuilder.fill(4.5, 4.5, 8.5);
        public static final Matrix<N3, N1> multiTagDevs = VecBuilder.fill(3, 3, 2.5);
    }
}
