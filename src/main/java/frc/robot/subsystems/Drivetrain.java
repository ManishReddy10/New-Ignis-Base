// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final Field2d m_field = new Field2d();

  /** Creates a new Drivetrain. */
  
  // Master is the front motor
  // Follower is the back motor
  TalonFX leftMaster = new TalonFX(1, "rio");
  TalonFX leftFollower = new TalonFX(2, "rio");
  TalonFX rightMaster = new TalonFX(3, "rio");
  TalonFX rightFollower = new TalonFX(4, "rio");


  // IF MOTORS GRINDING SET OpposeMasterDirection true
  Follower leftFollowerConfig = new Follower(1, false);
  Follower rightFollowerConfig = new Follower(3, false);


  DifferentialDrive m_DifferentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  





  public Drivetrain() {
    SmartDashboard.putData("Field", m_field);

    leftFollower.setControl(leftFollowerConfig);
    leftFollower.setControl(rightFollowerConfig);
    
    
  }

  private final double driveSpeed = 0.6;
  private final double turnSpeed = 0.6;

  public void arcadeDrive(final double forward, final double turn) {

    m_DifferentialDrive.arcadeDrive(forward*driveSpeed, turn*turnSpeed);

  }

  public void hardCodedDrive(final double forward, final double turn) {
    double speed = forward*driveSpeed;
    double turning = turn*turnSpeed;

    double left = speed + turning;
    double right = speed - turning;


    // if not all motors moving uncomment followers
    leftMaster.set(left);
    // leftFollower.set(left);
    rightMaster.set(-right);
    // rightFollower.set(right);

    

  }

  // public double getLeftEncoderPosition() {
  //   // get rotations of encoder by dividing encoder counts by counts per rotation
  //   return ((double) (leftMaster.getSelectedSensorPosition() / 2048) / 8.4) * (Math.PI * .1524);
  //   /*
  //   double encoderRotations = m_leftMaster.getSelectedSensorPosition() / 2048;

  //   // get rotations of wheel by diving rotations of encoder by gear ratio
  //   double wheelRotations = encoderRotations / 8.4;

  //   // get distance by multiplying rotations of wheel by circumference of wheel (pi * diameter in meters)
  //   double distance = wheelRotations * (Math.PI * (.1524));

	// 	return distance;
  //   */
	// }


  // private final DifferentialDrivePoseEstimator m_poseEstimator =
  // new DifferentialDrivePoseEstimator(



  // public Pose2d getPose(){
  //   return m_DifferentialDrive.
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_field.setRobotPose(getPose());



  }
}
