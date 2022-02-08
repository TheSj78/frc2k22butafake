// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTrajectory extends CommandBase {
  Drivetrain drivetrain;
  Trajectory traj;
  RamseteController ramseteController = new RamseteController();
  Timer timer = new Timer();
  /** Creates a new DriveTrajectory. */
  public DriveTrajectory(Drivetrain dt, Trajectory _traj) {
    drivetrain = dt;
    traj = _traj;
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /** Creates a new DriveTrajectory. */
  public DriveTrajectory(Drivetrain dt) {
   
    this(dt, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
              List.of(new Translation2d(0,0.5)), 
              new Pose2d(0, 1, new Rotation2d(0)), 
              new TrajectoryConfig(5, 2).setKinematics(dt.getKinematics()))
        );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setOdometry(traj.getInitialPose());
    drivetrain.getField().getObject("Traj").setTrajectory(traj);
    
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State setpoint = traj.sample(timer.get());

    
    ChassisSpeeds chassisSpeeds =
        ramseteController.calculate(drivetrain.getPose(), setpoint);
    DifferentialDriveWheelSpeeds wheelSpeeds =
        drivetrain.getKinematics().toWheelSpeeds(chassisSpeeds);
      
    
    drivetrain.setDrivetrainVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

