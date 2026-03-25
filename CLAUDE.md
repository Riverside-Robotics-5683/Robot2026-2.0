# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Deploy Commands

```bash
./gradlew -Dorg.gradle.java.home=$HOME/wpilib/2026/jdk build              # Compile the robot code
./gradlew -Dorg.gradle.java.home=$HOME/wpilib/2026/jdk deploy             # Deploy to RoboRIO (requires network connection to robot)
./gradlew -Dorg.gradle.java.home=$HOME/wpilib/2026/jdk simulationDebug    # Run simulation with GUI
./gradlew -Dorg.gradle.java.home=$HOME/wpilib/2026/jdk test               # Run unit tests (JUnit 5)
```

> **Note:** All `./gradlew` commands must include `-Dorg.gradle.java.home=$HOME/wpilib/2026/jdk` to use the WPILib JDK.

Team number: **5683**. Main class: `ravenrobotics.robot2026.Main`.

## Architecture Overview

This is a **command-based FRC robot** (WPILib 2026, Java 17) with a swerve drivetrain and a shooting mechanism.

### Entry Point Flow

```
Main.java → Robot.java (TimedRobot, 50 Hz) → RobotContainer (bindings + auto) → Subsystems + Superstructure
```

### Subsystems

| Subsystem | Hardware | Purpose |
|---|---|---|
| `CommandSwerveDrivetrain` | 4× TalonFX (drive/steer) + CANcoder | Swerve drive; PathPlanner integration |
| `FlywheelSubsystem` | 3× SparkFlex (left/center/right) | Shooting; center motor drives left/right as followers |
| `IntakeSubsystem` | TalonFX (ID 21) | Game piece intake |
| `FeederSubsystem` | SparkFlex (ID 4) | Moves pieces from intake to shooter |
| `PivotSubsystem` | TalonFX (ID 20) | Intake arm deployment with PID |
| `HoodSubsystem` | 2× SparkMax (#9/#10) | Shot angle adjustment; right leads, left follows |
| `VisionSubsystem` | 2× PhotonVision cameras | AprilTag localization; flywheel cam (17° pitch), hopper cam (25°) |

### Superstructure State Machine

`Superstructure.java` is the core high-level controller. States:

- `STOP` → all systems off
- `IDLE` → flywheel at idle RPM, intake retracted
- `INTAKE` → intake down, pulling in pieces, feeder spinning
- `IDLE_INTAKE_OUT` → idle with intake deployed
- `OUTTAKE` → pushing pieces out
- `SHOOT` → smart shooting (hub or pass based on field position)

### Shooting Logic

The robot uses **interpolating lookup tables** for distance-based shot parameters (RPM + hood angle). Shot mode is selected by field X position:
- Blue side (X < 6m): hub shooting
- Center (6m–11m): pass shooting
- The pivot oscillates between `SHOOT_HIGH`/`SHOOT_LOW` every 50 cycles ("pivot shake") to help feed pieces through

### Key Files

- `RobotContainer.java` — controller bindings and PathPlanner auto registration
- `Superstructure.java` — state machine driving intake/shooter coordination
- `Constants.java` — all tunable values (PIDs, RPMs, positions, lookup tables)
- `MotorConfigs.java` — centralized motor configuration
- `generated/TunerConstants.java` — auto-generated swerve tuner output (do not edit manually)
- `util/HubShiftUtil.java` — match phase tracking (used for strategy adjustments)

### Autonomous

PathPlanner autos live in `src/main/deploy/pathplanner/autos/`. Named commands registered in `RobotContainer`:
- `deployIntake`, `ssIdle`, `ssIdleIntakeOut`, `ssShoot`

Alliance-aware: paths are automatically flipped for Red alliance.

### Vendor Libraries

- **CTRE Phoenix 6 (v26.1.1)** — TalonFX motors (drive, pivot, intake), CANcoder
- **REVLib (2026.0.3)** — SparkFlex/SparkMax (flywheel, feeder, hood)
- **PathPlanner (2026.1.2)** — autonomous path following
- **PhotonVision** — AprilTag detection
- **DogLog** — structured logging with tunable NetworkTables parameters; logs written to USB on RoboRIO
- **Lombok** — annotation processing (used for boilerplate reduction)
