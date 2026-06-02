package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Stream;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain implements Subsystem{
    public List<SparkMax> motors;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    public AHRS gyro;

    public List<SparkBaseConfig> configs;

    public DifferentialDrivePoseEstimator PoseEstimator;

    private static Drivetrain inst;

    public Supplier<ChassisSpeeds> getChassisSpeeds;
    public Supplier<Pose2d> getPose;
    public Consumer<Pose2d> resetPose;

    private Drivetrain(){
        motors = Constants.MoorIDs.stream().<SparkMax>map(id -> new SparkMax(id, MotorType.kBrushless)).toList();
        LeftEncoder = motors.get(0).getEncoder();
        RightEncoder = motors.get(2).getEncoder();
        LeftPID = motors.get(0).getClosedLoopController();
        RightPID = motors.get(2).getClosedLoopController();
        gyro = new AHRS(NavXComType.kMXP_SPI);

        configs = List.of(
            Constants.LeadingMotorConfig.inverted(false),
            new SparkMaxConfig().follow(motors.get(0)),
            Constants.LeadingMotorConfig.inverted(true),
            new SparkMaxConfig().follow(motors.get(2))
        );

        motors.forEach(m -> configs.forEach(c -> m.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)));

        PoseEstimator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, new Pose2d());

        getChassisSpeeds = () -> Constants.kinematics.toChassisSpeeds(getState());
        getPose = PoseEstimator::getEstimatedPosition;
        resetPose = PoseEstimator::resetPose;

        register();
    }

    public DifferentialDriveWheelPositions getPosition(){
        return new DifferentialDriveWheelPositions(
            Constants.WheelCirc.times(LeftEncoder.getPosition()),
            Constants.WheelCirc.times(RightEncoder.getPosition()));
    }

    public DifferentialDriveWheelSpeeds getState(){
        return new DifferentialDriveWheelSpeeds(
            Constants.WheelCirc.times(LeftEncoder.getVelocity()).per(Seconds),
            Constants.WheelCirc.times(RightEncoder.getVelocity()).per(Seconds)
        );
    }

    public Command drive(Supplier<ChassisSpeeds> speeds){
        return run(() -> setChassisSpeeds(speeds.get()));
    }

    public void setChassisSpeeds(ChassisSpeeds spds){
        ChassisSpeeds.discretize(spds, 0.02);
        setSpeed(Constants.kinematics.toWheelSpeeds(spds));
    }
    
    private void setSpeed(DifferentialDriveWheelSpeeds speeds){
        speeds.desaturate(Constants.MaxVelocity);
        LeftPID.setSetpoint(speeds.leftMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
        RightPID.setSetpoint(speeds.rightMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void periodic(){
        PoseEstimator.update(gyro.getRotation2d(), getPosition());
        DogLog.log("Drivtrain/CurrentPose", getPose.get());
        DogLog.log("Drivetrain/CurrentSpeeds", getChassisSpeeds.get());
        DogLog.log("Drivetrain/WheelSpeeds", 
            List.of(
                new SwerveModuleState(getState().leftMetersPerSecond,Rotation2d.kZero), //FL
                new SwerveModuleState(getState().rightMetersPerSecond,Rotation2d.kZero), //FR
                new SwerveModuleState(getState().leftMetersPerSecond,Rotation2d.kZero), // BL
                new SwerveModuleState(getState().rightMetersPerSecond,Rotation2d.kZero) //BR
            ).toArray(SwerveModuleState[]::new));
        
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
