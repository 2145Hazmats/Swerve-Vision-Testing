// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Constants {
  /* Constants for the swerve chassis */
  public static class SwerveConstants {
    public static final double MAX_SPEED  = 5.4; // maximum m/s for the robot
    public static final double PATHPLANNER_TRANS_KP = 1;
    public static final double LOOP_TIME  = 0.13; // in seconds, 20ms + 110ms spark max velocity lag

//PASTED FROM OTEHR CODE IT WORKS
    public static final double P_Angle = 0.02;
    public static final double I_Angle = 0;
    public static final double D_Angle = 0.001;
    public static final double MaxPIDAngle = 1.25;//rad per sec 1 worked fine before testing new stuff
    public static final double FF_Angle = 0.33;
    public static final double FF_NoteAngle=.3;
    //VALUES FOR FACE POSE2D METHOD IN SWERVE SUBSYSTEM
    public static final double ROT_P = 0.025;
    public static final double ROT_D = 0.001;
    public static final double ROT_FF = 0.325;
    public static final double MAX_ROT_SPEED = 1;

    // The field is 16.5 meters x 7.2 meters y
    public static final Pose2d kBlueSubwooferPose = new Pose2d(0.0, 5.55, new Rotation2d(0));
    public static final Pose2d kRedSubwooferPose = new Pose2d(16.5, 5.55, new Rotation2d(0)); //16.5
   
    public static final Pose2d kBluePassPose = new Pose2d(1.0, 7.2, new Rotation2d(0));
    public static final Pose2d kRedPassPose = new Pose2d(15.5, 7.2, new Rotation2d(0));
  }

  /* Constants for the controllers */
  public static class OperatorConstants {
    public static final int kDriverControllerPort   = 0;
    public static final int kOperatorControllerPort = 1;
    /* Deadbands */
    public static final double LEFT_X_DEADBAND  = 0.04;
    public static final double LEFT_Y_DEADBAND  = 0.04;
    public static final double RIGHT_X_DEADBAND = 0.02;
    public static final double RIGHT_Y_DEADBAND = 0.02;
    /* Speed Modes */
    public static final double kFastModeSpeed = 1;
    public static final double kMidModeSpeed  = 0.6;
    public static final double kSlowModeSpeed = 0.3;
  }

  /* Constants for the arm subsystem */
  public static class ArmConstants{
    // All of our PID Postions for the arm
    public static enum ArmState {IDLE, FLOOR, SOURCE, AMP, SHOOT_SUB, SHOOT_N2, SHOOT_HORIZONTAL, CLIMB_1, CLIMB_2, TRAP, PASS, MANUAL};
    // Motor IDs
    public static final int kElbowMotorLeaderID   = 20;
    public static final int kElbowMotorFollowerID = 21;
    public static final int kWristMotorID         = 22;
    // NominalVoltage
    public static final double kElbowMotorNominalVoltage = 10.5;
    public static final double kWristMotorNominalVoltage = 10.5;
    // Encoder Conversion Factor
    public static final double kElbowEncoderFactor = 180;
    // Elbow and wrist PID + PID max speed
    public static final double kElbowP        = 0.025;
    public static final double kElbowI        = 0.0;
    public static final double kElbowD        = 0.01;
    public static final double kElbowMinSpeed = -0.6;
    public static final double kElbowMaxSpeed = 0.6;
    public static final double kWristP        = 0.1;
    public static final double kWristI        = 0.0;
    public static final double kWristD        = 0.05;
    public static final double kWristMinSpeed = -0.6;
    public static final double kWristMaxSpeed = 0.6;
    // Elbow FF gravity constant
    //public static final double kElbowS = 0;
    public static final double kElbowG = 0;
    // Elbow offset. The angle should be 0 degrees when parallel
    public static final double kElbowAngleOffset = 0.0;

    // Setpoints for the arm subsystem
    // {Elbow Angle, Wrist Angle} SP = SetPoint
    public static final double[] kIdleAngleSP             = {-0.25, 0};
    public static final double[] kFloorAngleSP            = {0, -34.5};
    public static final double[] kSourceAngleSP           = {-48.95, -20.88 }; //-60.51, 21.57 -33.31, 9.285
    public static final double[] kAmpAngleSP              = {-115.44, -37.93}; //-113.7, 31.64
    public static final double[] kSpeakerSubwooferAngleSP = {-9, -38}; //31.64
    public static final double[] kSpeakerN2AngleSP        = {9, -26}; // we need to set these values to be not false
    public static final double[] kHorizontalAngleSP       = {-33.4, -28.5};
    public static final double[] kClimb1AngleSP           = {-98, -34};
    public static final double[] kClimb2AngleSP           = {-43.6, -71.1};
    public static final double[] kTrapAngleSP             = {-85, -51}; // Shoot Subwoofer with intake within bumper bounderies
    //TODO: FILL OUT THIS ANGLE
    public static final double[] kPassAngleSP             = {-13.33, -8.31};

    public static final double SPEAKER_VISION_ELBOW_SP = 0; //-9
    public static final double SPEAKER_1_METER_WRIST_SP = -30.25;
    public static final double SPEAKER_2_METER_WRIST_SP = -28;
    public static final double SPEAKER_3_METER_WRIST_SP = -26.75;
    //TODO: SPEAKER_4_METER_WRIST_SP and SPEAKER_5_METER_WRIST_SP
    public static final double SPEAKER_4_METER_WRIST_SP = -25.75;
    public static final double SPEAKER_5_METER_WRIST_SP = -25;

    // This should be the same number as the highest SPEAKER_?_METER_WRIST_SP
    public static final double MAX_SPEAKER_VISION_METERS = 5;

    public static final double kManualSpeed = 0.8;
  }

  /* Constants for the box subsystem */
  public static class BoxConstants{
    // Motor IDs
    public static final int kTopShooterMotorID = 30;
    public static final int kBottomShooterMotorID = 31;
    public static final int kIntakeMotorID  = 32;
    // Sensor digital input channel
    public static final int kNoteSensorChannel = 0;
    //Max RPM of shooter motors.
    //public static final int kMaxRPM = 4960;
    // Nominal Voltage
    public static final double kIntakeMotorNominalVoltage  = 10.5;
    public static final double kShooterMotorNominalVoltage = 10.5;
    // Shooter motor PFF constants
    public static final double kTopShooterP = 0.0002;
    public static final double kBottomShooterP = 0.0002;
    public static final double kTopShooterFF = 0.000207; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
    public static final double kBottomShooterFF = 0.000237; //(1/kMaxRPM); // same as kV but in percentage instead of volts?
    // Shooter motor speeds
    public static final double kTopDefaultRPM    = 3500;
    public static final double kTopSpeakerRPM    = 3500;
    public static final double kTopAmpRPM        = 1200;
    public static final double kTopHorizontalRPM = 4000;
    public static final double kTopN2RPM         = 3700;
    //TODO: FILL OUT THIS RPM
    public static final double kTopPassRPM       = 3500;
    //public static final double kTopYeetRPM    = 2500;
    //public static final double kBottomYeetRPM = 2500;
    // RPM error range
    public static final double kRPMErrorRange = 200; // We are using Math.abs so its +/- kRPMErrorRange
    // Intake motor speeds
    public static final double kIntakeSpeed      = 1;
    public static final double kSourceIntakeSpeed = 0.8;
    public static final double kFeedSpeed        = 0.8;
    public static final double kRegurgitateSpeed = -0.5;
    public static final double kYeetSpeedIntake  = 1;
    // Shooter delay
    public static final double kShooterDelay     = 1.25;
    // Regurgitate time
    public static final double kRegurgitateTime  = 0.25;
  }

  /*
  public static class LimelightConstants {
    // Wrist offsets
    public static final double wristAngleOffset = 0.0;  // in degrees
    public static final double wristPivotOffsetX = 0.00; // in Meters
    public static final double wristPivotOffsetY = 0.00; // in Meters
    // Speaker height and offset
    public static final double speakerHeight = 0.00;     // height to the middle of the speaker opening
    public static final double speakerOffsetX = 0.22;    // distance to center of speaker opening
  }
  */

  // Constants used for PhotonVision
  public static class PhotonVisionConstants {
    //Transform3d from the center of the robot to the camera mount position (ie, robot ➔ camera) in the Robot Coordinate System
    //The Cameras are mounter on the back of the value so all transform signs are flipped (not rotations). + ➔ -
    public static final Transform3d ROBOT_TO_RIGHT_CAMERA =
        new Transform3d(-0.2545401, 0.1467405, 0.1934088, new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(12)));
    public static final Transform3d ROBOT_TO_LEFT_CAMERA =
        new Transform3d(-0.2545401, -0.1467405,  0.1934088, new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(-12)));
    public static final Transform3d ROBOT_TO_NOTE_CAMERA =
        new Transform3d(-0.2545401, 0, 0.1934088, new Rotation3d(0,Units.degreesToRadians(0),Units.degreesToRadians(0)));
  }

  
}
