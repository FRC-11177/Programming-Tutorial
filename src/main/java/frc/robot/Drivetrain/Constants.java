package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final List<Integer> MoorIDs = List.of(11,12,13,14); //FL,BL,FR,BR
    public static final Distance WheelCirc = Inches.of(6).times(Math.PI);
    public static final double GearRatio = 10.71;
    public static final Distance WheelDistance = Centimeters.of(60);
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(WheelCirc);
    public static final LinearVelocity MaxVelocity = WheelCirc.times(5676/GearRatio/60).per(Seconds);
    public static final AngularVelocity MaxOmega = RotationsPerSecond.of(1.5);
    public static final ClosedLoopConfig DrivePID = new ClosedLoopConfig()
        .pid(0, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    public static final FeedForwardConfig DriveFF = new FeedForwardConfig()
        .sva(0, 0, 0);
    public static final MAXMotionConfig DriveMotion = new MAXMotionConfig()
        .maxAcceleration(0);
    public static final SparkBaseConfig LeadingMotorConfig = new SparkMaxConfig()
        .apply(
            new SparkMaxConfig()
                .idleMode(IdleMode.kBrake)
                .voltageCompensation(12)
                .smartCurrentLimit(40))
        .apply(
            new EncoderConfig()
                .positionConversionFactor(1/GearRatio)
                .velocityConversionFactor(1/GearRatio/60)
        ).apply(DrivePID.apply(DriveFF).apply(DriveMotion));

}
