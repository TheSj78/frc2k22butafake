// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonPath6 extends SequentialCommandGroup {
  /** Creates a new AutonPath6. */

  private Drivetrain drivetrain;
  public AutonPath6(Drivetrain dr) 
  {
    int delay = 5;
    drivetrain = dr;
    
    addCommands(
      moveFront().withTimeout(delay),
      moveBack().withTimeout(delay),
      moveFrontWithPathPlanner().withTimeout(delay),
      moveBackWithPathPlanner().withTimeout(delay),
      rotateAndMove().withTimeout(delay)
    );
  }
  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putString("ABTAN", "abbyton");
  }

  @Override
  public void end(boolean interrupted) {
      super.end(interrupted);
      SmartDashboard.putString("Auton", "autpoonaononaoanoaoa");
  }

  public Command moveBack()
  {
    Trajectory tr = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(0, -1),
        new Translation2d(0, -2),
        new Translation2d(0, -3)
      ),
      new Pose2d(0, -4, new Rotation2d(0)),
      new TrajectoryConfig(8, 2)
    );

    return drivetrain.getRamseteCommand(tr);
  }

  public Command moveFront()
  {
    Trajectory tr = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(0, 1),
        new Translation2d(0, 2),
        new Translation2d(0, 3)
      ), 
      new Pose2d(0, 4, new Rotation2d(0)), 
      new TrajectoryConfig(8, 2)
    );

    return drivetrain.getRamseteCommand(tr);
  }

  public Command moveFrontWithPathPlanner()
  {
    Trajectory tr = PathPlanner.loadPath("MoveFrontPath", 8, 2);
    return drivetrain.getRamseteCommand(tr);
  }

  public Command moveBackWithPathPlanner()
  {
    Trajectory tr = PathPlanner.loadPath("MoveBackPath", 8, 2);
    return drivetrain.getRamseteCommand(tr);
  }

  public Command rotateAndMove()
  {
    Trajectory tr = PathPlanner.loadPath("RotateAndMove", 8, 2);
    return drivetrain.getRamseteCommand(tr);
  }
}
