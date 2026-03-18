// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ravenrobotics.robot2026;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autoCommand;

    private final RobotContainer robotContainer;

    private String currentAuto = "";

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay hootTimeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        robotContainer = new RobotContainer();

        DogLog.setOptions(new DogLogOptions()
            .withCaptureConsole(true)
            .withCaptureDs(true)
            .withCaptureNt(true));
            
        DogLog.setEnabled(true);
    }

    @Override
    public void robotPeriodic() {
        hootTimeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Get the currently selected auto.
        String selectedAuto = robotContainer.getAutonomousCommand().getName();

        if (selectedAuto != currentAuto) {
            // Set the current auto to the selected auto.
            currentAuto = selectedAuto;

            // Clear the Field2d widget and return if the AutoBuilder isn't configured.
            if (!AutoBuilder.isConfigured()) {
                robotContainer.drivetrain
                    .getFieldWidget()
                    .getObject("autoPath")
                    .setPose(new Pose2d());
            }

            // Clear the Field2d widget and return if the selected auto somehow isn't in the list of autos.
            if (!AutoBuilder.getAllAutoNames().contains(currentAuto)) {
                robotContainer.drivetrain
                    .getFieldWidget()
                    .getObject("autoPath")
                    .setPose(new Pose2d());
            }

            List<PathPlannerPath> autoPaths = new ArrayList<>();

            try {
                // Load the paths from the auto.
                autoPaths = PathPlannerAuto.getPathGroupFromAutoFile(currentAuto);
            } catch (Exception e) {
                // Print the error and return, don't continue.
                e.printStackTrace();
                return;
            }

            // Flip the paths if the alliance is Red.
            var currentAlliance = DriverStation.getAlliance();

            if (currentAlliance.isPresent()) {
                if (currentAlliance.get() == Alliance.Red) {
                    List<PathPlannerPath> flippedPaths = new ArrayList<>();

                    // Flip each path in the auto paths.
                    for (var path: autoPaths) {
                        var flippedPath = path.flipPath();
                        flippedPaths.add(flippedPath);
                    }

                    // Replace the auto paths with the flipped paths.
                    autoPaths = flippedPaths;
                }
            }

            List<Pose2d> autoPathPoses = new ArrayList<>();

            // Convert all the path points to Pose2d objects for the Field2d widget.
            for (var path : autoPaths) {
                autoPathPoses.addAll(
                    path
                        .getAllPathPoints()
                        .stream()
                        .map(point -> 
                            new Pose2d(
                                point.position.getX(),
                                point.position.getY(),
                                new Rotation2d()
                            ))
                        .collect(Collectors.toList())
                );
            }

            // Put the path visualization onto the Field2d widget.
            robotContainer
                .drivetrain
                .getFieldWidget()
                .getObject("autoPath")
                .setPoses(autoPathPoses);
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand();

        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
        // System.out.println("Give me more time!!!");
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autoCommand != null) {
            CommandScheduler.getInstance().cancel(autoCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
