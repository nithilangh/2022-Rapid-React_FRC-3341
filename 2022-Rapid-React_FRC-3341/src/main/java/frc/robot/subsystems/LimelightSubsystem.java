// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  NetworkTable _limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight-3341");
  NetworkTableEntry _tx, _ty, _ta, _tl;

  public LimelightSubsystem() {    
    _tx = _limelightNetworkTable.getEntry("tx");
    _ty = _limelightNetworkTable.getEntry("ty");
    _ta = _limelightNetworkTable.getEntry("ta");
    _tl = _limelightNetworkTable.getEntry("tl");

    _limelightNetworkTable.getEntry("pipeline").setNumber(0);
  }

  public double getHorizontalAngleOffset() {
    return _tx.getDouble(29.8);
  }

  public double getVerticalAngleOffset() {
    return _ty.getDouble(24.85);
  }

  public void setCameraMode(boolean cameraOn) {
    _limelightNetworkTable.getEntry("ledMode").setNumber(cameraOn ? 1 : 0);
    _limelightNetworkTable.getEntry("camMode").setNumber(cameraOn ? 1 : 0);
  }

  public double getDistanceFromTarget(double mountingHeight, double mountingAngle) {
    return ((Constants.Field.HubHeightMeters - mountingHeight) / Math.tan(_ty.getDouble(24.85) + mountingAngle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
