package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.LimelightHelpers;


public class LimelightOdom {

    private final SwerveDrivePoseEstimator odometry;
    private final String LIMELIGHT_NAME = "limelight-front";  // Adjust if necessary
    private double llUpdateThreshold = 0.75;
    private boolean ignoreLimelight = false;

    /**
     * Constructor for LimelightOdom.
     *
     * @param initialModulePositions Initial positions of the swerve modules.
     * @param initialHeading Initial heading of the robot.
     * @param initialPose Initial pose of the robot.
     */
    public LimelightOdom(SwerveModulePosition[] initialModulePositions, Rotation2d initialHeading, Pose2d initialPose) {
        // Initialize the odometry with the drivetrain data
        odometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_DRIVE_KINEMATICS,
            initialHeading,
            initialModulePositions,
            initialPose
        );
    }

    /**
     * Updates the odometry with the latest module positions, current pose, and heading.
     *
     * @param modulePositions Current positions of the swerve modules.
     * @param currentPose Current pose of the robot.
     * @param currentHeading Current heading of the robot.
     */
    public void update(SwerveModulePosition[] modulePositions, Pose2d currentPose, Rotation2d currentHeading) {
        // Update odometry with swerve drivetrain data
        odometry.update(currentHeading, modulePositions);

        // Update with vision data from Limelight
        updateLimelightPose(currentPose);
    }

    /**
     * Retrieves the current estimated pose of the robot.
     *
     * @return The current estimated pose as a Pose2d object.
     */
    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Retrieves the current estimated heading of the robot.
     *
     * @return The current estimated heading as a Rotation2d object.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Updates the pose estimate using data from the Limelight.
     *
     * @param currentPose Current pose of the robot.
     */
    private void updateLimelightPose(Pose2d currentPose) {
        // Set robot orientation for the Limelight camera
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, currentPose.getRotation().getDegrees(), 35.0, 0, 0, 0, 0);

        // Retrieve the pose estimate from the Limelight
        LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LIMELIGHT_NAME);

        if (poseEstimate != null && poseEstimate.pose != null) {
            // Evaluate whether the pose from Limelight is valid
            ignoreLimelight = poseEstimate.tagCount == 0 ||
                !validPose(poseEstimate.pose) ||
                LimelightHelpers.getTA(LIMELIGHT_NAME) < 0.1 ||
                (getLLposesDist(poseEstimate.pose, currentPose) > llUpdateThreshold);

            if (!ignoreLimelight) {
                // Add vision measurement to the pose estimator
                odometry.addVisionMeasurement(new Pose2d(poseEstimate.pose.getX(), poseEstimate.pose.getY(), currentPose.getRotation()), poseEstimate.timestampSeconds);
            }
        }
    }

    /**
     * Calculates the distance between two poses.
     *
     * @param curr Current pose.
     * @param prev Previous pose.
     * @return The Euclidean distance between the two poses.
     */
    private double getLLposesDist(Pose2d curr, Pose2d prev) {
        return Math.sqrt(
            Math.pow((prev.getX() - curr.getX()), 2) +
            Math.pow((prev.getY() - curr.getY()), 2)
        );
    }

    /**
     * Checks if a pose is within valid field boundaries.
     *
     * @param pose The pose to validate.
     * @return True if the pose is valid, false otherwise.
     */
    private boolean validPose(Pose2d pose) {
        return pose.getX() > 0 && pose.getX() < 16 && pose.getY() > 0 && pose.getY() < 8;
    }

    /**
     * Updates the SmartDashboard with relevant Limelight data.
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("Limelight Area", LimelightHelpers.getTA(LIMELIGHT_NAME));
        SmartDashboard.putBoolean("Ignore Limelight", ignoreLimelight);
        SmartDashboard.putNumber("Limelight TX", LimelightHelpers.getTX(LIMELIGHT_NAME));
        SmartDashboard.putNumber("Limelight TY", LimelightHelpers.getTY(LIMELIGHT_NAME));
        SmartDashboard.putNumber("Limelight Target Count", LimelightHelpers.getTargetCount(LIMELIGHT_NAME));
    }
}
