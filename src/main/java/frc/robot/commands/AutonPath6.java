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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutonPath6 extends SequentialCommandGroup {
  /** Creates a new AutonPath6. */

  private Drivetrain drivetrain;
  public AutonPath6(Drivetrain dr) 
  {
    drivetrain = dr;
    addCommands(
      new MoveBack().withTimeout(5)
    );
  }
  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    SmartDashboard.putString("ABTAN", "abbyton");
  }
  @Override
  public void end(boolean interrupted) {
      // TODO Auto-generated method stub
      super.end(interrupted);

      SmartDashboard.putString("Auton", "autpoonaononaoanoaoa");
  }

  public Command MoveBack()
  {
    Trajectory tr = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(new Translation2d(0, 1), new Translation2d(0,2)), new Pose2d(0, 3, new Rotation2d()), new TrajectoryConfig(10, 2));//PathPlanner.loadPath("TryPath", 4, 4);
    return drivetrain.getRamseteCommand(tr);
  }
}
