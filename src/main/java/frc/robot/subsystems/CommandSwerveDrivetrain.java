package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightOdom;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private LimelightOdom limelightOdom;

    private Field2d field = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    /**
     * Constructs a CommandSwerveDrivetrain.
     *
     * @param driveTrainConstants Constants for the swerve drivetrain.
     * @param OdometryUpdateFrequency Frequency for updating odometry.
     * @param modules Array of swerve module constants.
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        limelightOdom = new LimelightOdom(getModulePositions(), getHeading(), getPose());
    }

    /**
     * Constructs a CommandSwerveDrivetrain.
     *
     * @param driveTrainConstants Constants for the swerve drivetrain.
     * @param modules Array of swerve module constants.
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        limelightOdom = new LimelightOdom(getModulePositions(), getHeading(), getPose());
    }

    /**
     * Creates a command to apply a swerve request.
     *
     * @param requestSupplier A supplier for the swerve request.
     * @return A command that applies the given swerve request.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Starts a simulation thread to update the simulation state periodically.
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

             /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Periodically updates the drivetrain state and applies operator perspective.
     */
    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/

        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
        // Update the limelight odometry
        limelightOdom.update(getModulePositions(), getPose(), getHeading());

        // Update the field
        field.setRobotPose(limelightOdom.getPose());

        // Update the SmartDashboard
        SmartDashboard.putData("Field", field);
        limelightOdom.updateDashboard();
    }

    /**
     * Gets the pose estimator.
     *
     * @return The swerve drive pose estimator.
     */
    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.m_odometry;
    }

    /**
     * Gets the current pose.
     *
     * @return The current pose of the robot.
     */
    public Pose2d getPose() {
        return limelightOdom.getPose();
    }

    /**
     * Gets the current heading.
     *
     * @return The current heading of the robot.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Gets the positions of the swerve modules.
     *
     * @return An array of swerve module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            SwerveModule module = getModule(i);
            positions[i] = module.getPosition(true);
        }
        return positions;
    }
}
