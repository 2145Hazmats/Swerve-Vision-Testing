// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhotonVisionConstants;
import frc.robot.Constants.SwerveConstants;


public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  // PhotonVision objects
  private PhotonCamera rightCamera = new PhotonCamera("Right_Arducam_OV9281_USB_Camera");
  private PhotonCamera leftCamera = new PhotonCamera("Left_Arducam_OV9281_USB_Camera");
  private PhotonCamera noteCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  private PIDController TurnToAnglePIDController = new PIDController(Constants.SwerveConstants.P_Angle, Constants.SwerveConstants.I_Angle, Constants.SwerveConstants.D_Angle);

  private PhotonPipelineResult rightResult = null;
  private PhotonPipelineResult leftResult = null;
  private PhotonPipelineResult middleResult = null;

  private PhotonTrackedTarget rightTarget = null;
  private PhotonTrackedTarget leftTarget = null;

   private double Aedyn = 1;

  // PhotonVision objects used in vision localization
  private PhotonPoseEstimator rightPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
      //Calculates a new robot position estimate by combining all visible tag corners.
      //If using MULTI_TAG_PNP_ON_COPROCESSOR, must configure the AprilTagFieldLayout properly in the UI.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      rightCamera,
      PhotonVisionConstants.ROBOT_TO_RIGHT_CAMERA);

  private PhotonPoseEstimator leftPoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
      //Calculates a new robot position estimate by combining all visible tag corners.
      //If using MULTI_TAG_PNP_ON_COPROCESSOR, must configure the AprilTagFieldLayout properly in the UI.
      //https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html#multitag-localization
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      leftCamera,
      PhotonVisionConstants.ROBOT_TO_LEFT_CAMERA);

  private PhotonPoseEstimator middlePoseEstimator = new PhotonPoseEstimator(
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      noteCamera, 
      PhotonVisionConstants.ROBOT_TO_NOTE_CAMERA);

  // latest EstimatedRobotPose from PhotonPoseEstimator
  private EstimatedRobotPose rightLatestRobotPose = null;
  private EstimatedRobotPose leftLatestRobotPose = null;
  private EstimatedRobotPose middleLatestRobotPose = null;

  // Rotation PIDcontrollers
  private double visionRotP = Constants.SwerveConstants.ROT_P;
  private double visionRotI = 0.0; 
  private double visionRotD = Constants.SwerveConstants.ROT_D;

  private PIDController RotationalPIDController = new PIDController(
      visionRotP,
      visionRotI,
      visionRotD
  );
private PIDController RotationalNotePIDController = new PIDController(
      .05,
      0,
      0
  );

  

  // True = we add vision Pose2ds to the robot's existing odometry
  // False = vision Pose2ds and robot odometry are separate (not as good)
  private boolean visionPlusOdometryLocalization = false;
  private Pose2d visionPose2d = new Pose2d();

  // Field to display on Shuffleboard
  private final Field2d m_field = new Field2d();

  public static double allianceInverse = 1;

  public static boolean teleopMode = false;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_SPEED);
    } catch (Exception e) { throw new RuntimeException(e); }

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle
  
    SmartDashboard.putNumber("visionRotP", Constants.SwerveConstants.ROT_P);
    SmartDashboard.putNumber("visionRotD", Constants.SwerveConstants.ROT_D);
  }


  /* Setup AutoBuilder for PathPlanner */
  public void setupPathPlannerRobot() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            // Translation PID constants
            new PIDConstants(SwerveConstants.PATHPLANNER_TRANS_KP, 0.0, 0.0),
            // Rotation PID constants
            new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                swerveDrive.swerveController.config.headingPIDF.i,
                swerveDrive.swerveController.config.headingPIDF.d),
            // Max module speed, in m/s
            SwerveConstants.MAX_SPEED,
            // Drive base radius in meters. Distance from robot center to furthest module.
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Default path replanning config. See the API for the options here
            new ReplanningConfig()
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
    this); // Reference to this subsystem to set requirements
  }


  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // Open loop is disabled since it shouldn't be used most of the time.
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }


  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }


  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }
  

  /* Lock the swerve drive to prevent it from moving */
  public void lock() {
    swerveDrive.lockPose();
  }


  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 3.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(360)
    );

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.5 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }


  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param path Target {@link PathPlannerPath} to go to and then follow.
   * @return PathFinding command
   */
  public Command driveToPathThenFollowPath(PathPlannerPath path) {
    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumVelocity(), 3.0,
        swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(360)
    );

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        0.5 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );
  }


  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @param nerfChooser      A speed multiplier.
   * @param isFieldCentric   True if the robot should be field centric. False if the robot should be robot centric.
   * @return Drive command.
   */
  public Command driveCommandAngularVelocity(DoubleSupplier translationX,
                                             DoubleSupplier translationY,
                                             DoubleSupplier angularRotationX,
                                             double nerfChooser,
                                             boolean isFieldCentric
                                             ) {
    //swerveDrive.setHeadingCorrection(false);

    return run(() -> {
      // Make the robot move
      swerveDrive.drive(
          new Translation2d(
              MathUtil.applyDeadband(translationX.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND) * swerveDrive.getMaximumVelocity() * nerfChooser,
              MathUtil.applyDeadband(translationY.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND) * swerveDrive.getMaximumVelocity() * nerfChooser
          ).times(allianceInverse),
          Math.pow(MathUtil.applyDeadband(angularRotationX.getAsDouble(), OperatorConstants.RIGHT_X_DEADBAND), 3) * swerveDrive.getMaximumAngularVelocity() * nerfChooser,
          isFieldCentric, 
          false);
    });
  }


  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommandPoint(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    swerveDrive.setHeadingCorrection(true, 0.01); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      double xInput = MathUtil.applyDeadband(translationX.getAsDouble(), OperatorConstants.LEFT_X_DEADBAND); // Smooth controll out
      double yInput = MathUtil.applyDeadband(translationY.getAsDouble(), OperatorConstants.LEFT_Y_DEADBAND); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
          headingX.getAsDouble() * allianceInverse,
          headingY.getAsDouble() * allianceInverse,
          swerveDrive.getYaw().getRadians(),
          swerveDrive.getMaximumVelocity()));
    });
  }


  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }


  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }


  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }


  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }


  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }


  /**
   * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public double PIDturnToAngle(double RotationSetpointDegrees) {
    double AngleSpeedCalculated = TurnToAnglePIDController.calculate(
        getPose().getRotation().getDegrees(), RotationSetpointDegrees);//-180 - 180 degrees

    double charzardSigmaDefinite = Math.signum(AngleSpeedCalculated);

    //Charzard is = to -1 or 1 based on if the calculation is neg
    AngleSpeedCalculated = AngleSpeedCalculated + (Constants.SwerveConstants.FF_Angle * charzardSigmaDefinite);
    
    if (charzardSigmaDefinite == 1) {
      AngleSpeedCalculated = Math.min(AngleSpeedCalculated, Constants.SwerveConstants.MaxPIDAngle);
    } else {
      AngleSpeedCalculated = Math.max(AngleSpeedCalculated, -Constants.SwerveConstants.MaxPIDAngle);
    }

    return AngleSpeedCalculated;
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY, getHeading().getRadians(), SwerveConstants.MAX_SPEED);
  }


  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians(), SwerveConstants.MAX_SPEED);
  }


  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }


  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }


  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }


  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }


  /* Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0 */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }


  // Resets the gyro angle
  public void resetGyro(){
    swerveDrive.setGyro(new Rotation3d(0, 0, 0) );
  }


  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }


  public double facePassPose2d() {
    Pose2d CordsToFace;
    
    // Change subwoofer coordinates depending on blue/red alliance
    CordsToFace = (allianceInverse == 1)
        ? Constants.SwerveConstants.kBluePassPose
        : Constants.SwerveConstants.kRedPassPose;

    double robotX;
    double robotY;
    if (visionPlusOdometryLocalization == true) {
      robotX = getPose().getX();
      robotY = getPose().getY();
    } else {
      robotX = visionPose2d.getX();
      robotY = visionPose2d.getY();
    }

    // Calculate the x and y difference between the robot and subwoofer 
    // x is abs() because I use atan2() and it makes it simpler
    double xDifference = Math.abs(robotX - CordsToFace.getX());
    double yDifference = robotY - CordsToFace.getY();

    double targetAngleDeg;
    // atan2() my beloved. [-180, 180]
    // targetAngleDeg's sign is reversed depending on the alliance
    targetAngleDeg = (allianceInverse == 1)
      ? Units.radiansToDegrees(Math.atan2(yDifference, xDifference))
      : -Units.radiansToDegrees(Math.atan2(yDifference, xDifference));

    // Clalculate rotspeed in radians using a PID controller
    double rotSpeed = -RotationalPIDController.calculate(
        getPose().getRotation().getDegrees(), targetAngleDeg
    );
    // Speed to overcome friction
    rotSpeed += Math.signum(rotSpeed) * SwerveConstants.ROT_FF;
    // Limit max speed in radians. Java 21 has Math.clamp(), but we are using Java 17
    rotSpeed = Math.max(
        -SwerveConstants.MAX_ROT_SPEED,
        Math.min(SwerveConstants.MAX_ROT_SPEED, rotSpeed)
    );

    // Put variables on SmartDashboard
    SmartDashboard.putNumber("xDifference", xDifference);
    SmartDashboard.putNumber("yDifference", yDifference);
    SmartDashboard.putNumber("targetAngleDeg", targetAngleDeg);
    SmartDashboard.putNumber("rotSpeed", rotSpeed);
    
    return rotSpeed;
  }


  /* Add a vision measurement for localization */
  public void addVisionPose2d(Pose2d pose2d, double timestampSeconds) {
    if ((teleopMode == true) && (visionPlusOdometryLocalization == true)) {
      swerveDrive.addVisionMeasurement(pose2d, timestampSeconds);
    }
    else {
      // speed of the robot from [0.05, 0.9]
      double robotSpeed = (Math.abs((getRobotVelocity().vxMetersPerSecond/2)) + Math.abs(getRobotVelocity().vyMetersPerSecond/2));
      robotSpeed = Math.max(
          0.05,
          Math.min(0.9, robotSpeed)
      );

      visionPose2d = visionPose2d.interpolate(pose2d, robotSpeed);
    }
  }


  // Method to turn towards the subwoofer using a vision pose2d and the gyro
  // Turns towards the blue subwoofer on the blue alliance and vice versa
  // Gyro reset has to match the alliance you are playing as during testing. Otherwise it will be 180 degrees off!
  public double faceSubwooferPose2d() {
    Pose2d CordsToFace;
    
    // Change subwoofer coordinates depending on blue/red alliance
    CordsToFace = (allianceInverse == 1)
        ? Constants.SwerveConstants.kBlueSubwooferPose
        : Constants.SwerveConstants.kRedSubwooferPose;

    double robotX;
    double robotY;
    if (visionPlusOdometryLocalization == true) {
      robotX = getPose().getX();
      robotY = getPose().getY();
    } else {
      robotX = visionPose2d.getX();
      robotY = visionPose2d.getY();
    }

    // Calculate the x and y difference between the robot and subwoofer 
    // x is abs() because I use atan2() and it makes it simpler
    double xDifference = Math.abs(robotX - CordsToFace.getX());
    double yDifference = robotY - CordsToFace.getY();

    double targetAngleDeg;
    // atan2() my beloved. [-180, 180]
    // targetAngleDeg's sign is reversed depending on the alliance
    targetAngleDeg = (allianceInverse == 1)
      ? Units.radiansToDegrees(Math.atan2(yDifference, xDifference))
      : -Units.radiansToDegrees(Math.atan2(yDifference, xDifference));

    // Clalculate rotspeed in radians using a PID controller
    double rotSpeed = -RotationalPIDController.calculate(
        getPose().getRotation().getDegrees(), targetAngleDeg
    );
    // Speed to overcome friction
    rotSpeed += Math.signum(rotSpeed) * SwerveConstants.ROT_FF;
    // Limit max speed in radians. Java 21 has Math.clamp(), but we are using Java 17
    rotSpeed = Math.max(
        -SwerveConstants.MAX_ROT_SPEED,
        Math.min(SwerveConstants.MAX_ROT_SPEED, rotSpeed)
    );

    return rotSpeed;
  }
public void setAmpCam() {
 
  if(Aedyn == 1) {
noteCamera.setPipelineIndex(1);
    Aedyn=0;
  } else {
    noteCamera.setPipelineIndex(0);
    Aedyn=1;
  }
  

}
public double faceNote() {
  double NoteStuffReal=0;
  PhotonPipelineResult result = noteCamera.getLatestResult();
  if (result.hasTargets()) {
    NoteStuffReal = result.getBestTarget().getYaw();
  }
    double AngleSpeedCalculated = TurnToAnglePIDController.calculate(NoteStuffReal, 0);//-180 - 180 degrees

    double charzardSigmaDefinite = Math.signum(AngleSpeedCalculated);

    //Charzard is = to -1 or 1 based on if the calculation is neg
    AngleSpeedCalculated = AngleSpeedCalculated + (Constants.SwerveConstants.FF_NoteAngle * charzardSigmaDefinite);
    
    if (charzardSigmaDefinite == 1) {
      AngleSpeedCalculated = Math.min(AngleSpeedCalculated, Constants.SwerveConstants.MaxPIDAngle);
    } else {
      AngleSpeedCalculated = Math.max(AngleSpeedCalculated, -Constants.SwerveConstants.MaxPIDAngle);
    }

    return AngleSpeedCalculated;
  }

  // I couldn't find the built-in lerp method in java. So here it is
  public double lerp(double a, double b, double f) {
    return a * (1.0 - f) + (b * f);
  }
  

  // Returns the distance to the red/blue subwoofer
  public double distanceToSubwoofer() {
    Pose2d CordsToFace;
    
    // Change subwoofer coordinates depending on blue/red alliance
    CordsToFace = (allianceInverse == 1)
        ? Constants.SwerveConstants.kBlueSubwooferPose
        : Constants.SwerveConstants.kRedSubwooferPose;

    double robotX;
    double robotY;
    if (visionPlusOdometryLocalization == true) {
      robotX = getPose().getX();
      robotY = getPose().getY();
    } else {
      robotX = visionPose2d.getX();
      robotY = visionPose2d.getY();
    }

    // Calculate the x and y difference between the robot and subwoofer 
    double xDifference = robotX - CordsToFace.getX();
    double yDifference = robotY - CordsToFace.getY();

    // c^2 = a^2 + b^2
    double distanceToSpeaker = Math.pow(xDifference, 2) + Math.pow(yDifference, 2);
    distanceToSpeaker = Math.pow(distanceToSpeaker, 0.5);

    return distanceToSpeaker;
  }


  // This method calculates the wrist angle to the speaker.
  // Should never return 0. If it does, there is a bug in this method
  public double calculateWristAngleToSpeaker() {
    double distance = distanceToSubwoofer();
    SmartDashboard.putNumber("distanceToSubwoofer", distance);

    // Important for drivers to know if they are within vision range for the speaker!!!
    if (distance <= ArmConstants.MAX_SPEAKER_VISION_METERS) {
      SmartDashboard.putBoolean("Speaker Vision", true);
    } else {
      SmartDashboard.putBoolean("Speaker Vision", false);
    }

    // You can have more or less of these. This should be easy to setup assuming the math is correct.
    // The wrist/arm encoders might not be in degrees and I don't know if we fire at an arc.
    // This is similar to what you wanted with preset angles, it just interpolates between preset angles.
    double visionWristAngle = 0.0;
    if (distance <= 1) { // I don't know if being closer than 1 meter to the subwoofer is possible gven how we measure
      visionWristAngle = ArmConstants.SPEAKER_1_METER_WRIST_SP;
    }
    else if (distance <= 2) {
      visionWristAngle = lerp(
          ArmConstants.SPEAKER_1_METER_WRIST_SP,
          ArmConstants.SPEAKER_2_METER_WRIST_SP,
          distance - 1
      );
    }
    else if (distance <= 3) {
      visionWristAngle = lerp(
          ArmConstants.SPEAKER_2_METER_WRIST_SP,
          ArmConstants.SPEAKER_3_METER_WRIST_SP,
          distance - 2
      );
    }
    else if (distance <= 4) {
      visionWristAngle = lerp(
          ArmConstants.SPEAKER_3_METER_WRIST_SP,
          ArmConstants.SPEAKER_4_METER_WRIST_SP,
          distance - 3
      );
    }
    else if (distance <= 5) {
      visionWristAngle = lerp(
          ArmConstants.SPEAKER_4_METER_WRIST_SP,
          ArmConstants.SPEAKER_5_METER_WRIST_SP,
          distance - 4
      );
    }
    else {
      visionWristAngle = ArmConstants.SPEAKER_5_METER_WRIST_SP;
    }

    SmartDashboard.putNumber("visionWristAngle", visionWristAngle);
    return visionWristAngle;
  }


  @Override
  public void periodic() {
    // Get the latest camera results
    rightResult = rightCamera.getLatestResult();
    leftResult = leftCamera.getLatestResult();
    middleResult = noteCamera.getLatestResult();

    SmartDashboard.putBoolean("CameraTrue", noteCamera.getLatestResult().hasTargets());

    if (noteCamera.getLatestResult().hasTargets()) {
      SmartDashboard.putNumber("yaw", noteCamera.getLatestResult().getBestTarget().getYaw());
    };

    // Try to update "latestRobotPose" with a new "EstimatedRobotPose" using a "PhotonPoseEstimator"
    // If "latestRobotPose" is updated, call addVisionPose2d() and pass the updated "latestRobotPose" as an argument
    try {
      rightLatestRobotPose = rightPoseEstimator.update(rightResult).get();
      addVisionPose2d(rightLatestRobotPose.estimatedPose.toPose2d(), rightLatestRobotPose.timestampSeconds);
      SmartDashboard.putBoolean("rightLatestRobotPose Update", true);
    } catch (Exception e) { // catch = catching an exception, java.util.Optional.get() throws NoSuchElementException if no value is present
      rightLatestRobotPose = null; // If there is no updated "EstimatedRobotPose", update "latestRobotPose" to null
      SmartDashboard.putBoolean("rightLatestRobotPose Update", false);
    }
    try {
      middleLatestRobotPose = middlePoseEstimator.update(middleResult).get();
      addVisionPose2d(middleLatestRobotPose.estimatedPose.toPose2d(), middleLatestRobotPose.timestampSeconds);
      SmartDashboard.putBoolean("middleLatestRobotPose Update", true);
    } catch (Exception e) {
      middleLatestRobotPose = null;
      SmartDashboard.putBoolean("middleLatestRobotPose Update", false);
    }

    try {
      leftLatestRobotPose = leftPoseEstimator.update(leftResult).get();
      addVisionPose2d(leftLatestRobotPose.estimatedPose.toPose2d(), leftLatestRobotPose.timestampSeconds);
      SmartDashboard.putBoolean("leftLatestRobotPose Update", true);
    } catch (Exception e) { // catch = catching an exception, java.util.Optional.get() throws NoSuchElementException if no value is present
      leftLatestRobotPose = null; // If there is no updated "EstimatedRobotPose", update "latestRobotPose" to null
      SmartDashboard.putBoolean("leftLatestRobotPose Update", false);
    }

    // Update field for Shuffleboard
    if (visionPlusOdometryLocalization == true) {
      m_field.setRobotPose(getPose());
    } else {
      m_field.setRobotPose(visionPose2d);
      SmartDashboard.putNumber("visionPose X", visionPose2d.getX());
      SmartDashboard.putNumber("visionPose Y", visionPose2d.getY());
      SmartDashboard.putNumber("visionPose Rot", visionPose2d.getRotation().getDegrees());
    }
    SmartDashboard.putBoolean("Localization", visionPlusOdometryLocalization);

    SmartDashboard.putNumber("AllianceInverse", allianceInverse);

    SmartDashboard.putNumber("RobotChasisSpeed X", getRobotVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("RobotChasisSpeed Y", getRobotVelocity().vyMetersPerSecond);
    SmartDashboard.putNumber("RobotChasisSpeed Rotation", getRobotVelocity().omegaRadiansPerSecond);

    SmartDashboard.putNumber("Vision X", getPose().getX());
    SmartDashboard.putNumber("Vision Y", getPose().getY());

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());

    // PID values in SmartDashboard for easy tuning
    if (visionRotP != SmartDashboard.getNumber("visionRotP", Constants.SwerveConstants.ROT_P)) {
      visionRotP = SmartDashboard.getNumber("visionRotP", 0);
      RotationalPIDController.setP(visionRotP);
    }
    if (visionRotD != SmartDashboard.getNumber("visionRotD", Constants.SwerveConstants.ROT_D)) {
      visionRotD = SmartDashboard.getNumber("visionRotD", 0);
      RotationalPIDController.setD(visionRotD);
    }
  }


  @Override
  public void simulationPeriodic() {}

}