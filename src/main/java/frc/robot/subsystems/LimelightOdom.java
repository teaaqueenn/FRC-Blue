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

    public LimelightOdom(SwerveModulePosition[] initialModulePositions, Rotation2d initialHeading, Pose2d initialPose) {
        // Initialize the odometry with the drivetrain data
        odometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_DRIVE_KINEMATICS,
            initialHeading,
            initialModulePositions,
            initialPose
        );
    }

    public void update(SwerveModulePosition[] modulePositions, Pose2d currentPose, Rotation2d currentHeading) {
        // Update odometry with swerve drivetrain data
        odometry.update(currentHeading, modulePositions);

        // Update with vision data from Limelight
        updateLimelightPose(currentPose);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

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

    private double getLLposesDist(Pose2d curr, Pose2d prev) {
        return Math.sqrt(
            Math.pow((prev.getX() - curr.getX()), 2) +
            Math.pow((prev.getY() - curr.getY()), 2)
        );
    }

    private boolean validPose(Pose2d pose) {
        return pose.getX() > 0 && pose.getX() < 16 && pose.getY() > 0 && pose.getY() < 8;
    }

    public void updateDashboard() {
       SmartDashboard.putNumber("Limelight Area", LimelightHelpers.getTA(LIMELIGHT_NAME));
    }
}
