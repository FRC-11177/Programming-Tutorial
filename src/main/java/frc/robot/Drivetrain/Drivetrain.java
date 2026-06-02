package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Drivetrain implements Subsystem{
    public List<SparkMax> motors;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    public AHRS gyro;

    // 這裡把每一顆馬達的設定集中放成一個清單，方便一次套用到多顆馬達。
    public List<SparkBaseConfig> configs;

    public DifferentialDrivePoseEstimator PoseEstimator;

    private static Drivetrain inst;

    public Supplier<ChassisSpeeds> getChassisSpeeds;
    public Supplier<Pose2d> getPose;
    public Consumer<Pose2d> resetPose;

    private Drivetrain(){
        // stream() + map()：把馬達 ID 轉成 SparkMax 物件，最後收集成 List。
        motors = Constants.MoorIDs.stream().<SparkMax>map(id -> new SparkMax(id, MotorType.kBrushless)).toList();
        // 這兩個 Encoder 代表左右側輪子的回授資料，用來讀位置與速度。
        LeftEncoder = motors.get(0).getEncoder();
        RightEncoder = motors.get(2).getEncoder();
        // Closed-loop controller 會依照目標值自動調整輸出，抓內建 PID 控制器。
        LeftPID = motors.get(0).getClosedLoopController();
        RightPID = motors.get(2).getClosedLoopController();
        // NavX 陀螺儀提供機器人的朝向角度，供里程計與姿態估算使用。
        gyro = new AHRS(NavXComType.kMXP_SPI);

        // List.of() 會建立一個固定內容的清單，這裡存四種馬達設定。
        configs = List.of(
            Constants.LeadingMotorConfig.inverted(false),
            new SparkMaxConfig().follow(motors.get(0)),
            Constants.LeadingMotorConfig.inverted(true),
            new SparkMaxConfig().follow(motors.get(2))
        );

        // 將每一種設定套用到每一顆馬達。
        motors.forEach(m -> configs.forEach(c -> m.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)));

        // 姿態估算器會結合馬達里程與陀螺儀角度，估出機器人目前位置。
        PoseEstimator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPosition().leftMeters, getPosition().rightMeters, new Pose2d());

        // Supplier / Consumer 是函式型別，讓其他地方可以直接取用目前速度、位置，或重設位置。
        getChassisSpeeds = () -> Constants.kinematics.toChassisSpeeds(getState());
        getPose = PoseEstimator::getEstimatedPosition;
        resetPose = PoseEstimator::resetPose;

        register();
        autoInit();
    }

    /**
     * 讀取左右輪目前累積的距離。
     *
     * <p>這個值由編碼器換算而來，通常用於里程計與位置估算。
     *
     * @return 左右輪目前的距離位置。
     */
    public DifferentialDriveWheelPositions getPosition(){
        return new DifferentialDriveWheelPositions(
            Constants.WheelCirc.times(LeftEncoder.getPosition()),
            Constants.WheelCirc.times(RightEncoder.getPosition()));
    }

    /**
     * 讀取左右輪目前的速度。
     *
     * <p>這個值由編碼器轉速換算成實際線速度，供控制與記錄使用。
     *
     * @return 左右輪目前的線速度。
     */
    public DifferentialDriveWheelSpeeds getState(){
        return new DifferentialDriveWheelSpeeds(
            Constants.WheelCirc.times(LeftEncoder.getVelocity()).per(Seconds),
            Constants.WheelCirc.times(RightEncoder.getVelocity()).per(Seconds)
        );
    }

    /**
     * 建立一個持續驅動底盤的命令。
     *
     * @param speeds 由外部提供的底盤速度。
     * @return 可持續執行的驅動命令。
     */
    public Command drive(Supplier<ChassisSpeeds> speeds){
        return run(() -> setChassisSpeeds(speeds.get()));
    }

    /**
     * 設定底盤的目標速度。
     *
     * <p>先把速度離散化，再轉成左右輪速度交給馬達控制器。
     *
     * @param spds 目標底盤速度。
     */
    public void setChassisSpeeds(ChassisSpeeds spds){
        // 0.02 秒通常代表 20ms 的控制迴圈週期。
        ChassisSpeeds.discretize(spds, 0.02);
        setSpeed(Constants.kinematics.toWheelSpeeds(spds));
    }
    
    /**
     * 將左右輪速度送進馬達閉迴路控制器。
     *
     * @param speeds 左右輪目標速度。
     */
    private void setSpeed(DifferentialDriveWheelSpeeds speeds){
        // desaturate() 會把速度限制在安全範圍內，避免指令超出馬達能力。
        speeds.desaturate(Constants.MaxVelocity);
        // setSetpoint() 會把目標速度送給控制器，由馬達控制器自行閉迴路調整輸出。
        LeftPID.setSetpoint(speeds.leftMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
        RightPID.setSetpoint(speeds.rightMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void periodic(){
        // 每個週期更新里程計，讓位置估算保持最新。
        PoseEstimator.update(gyro.getRotation2d(), getPosition());
        DogLog.log("Drivtrain/CurrentPose", getPose.get());
        DogLog.log("Drivetrain/CurrentSpeeds", getChassisSpeeds.get());
        // 下面把左右輪速度轉成陣列格式，方便記錄到 DogLog。
        DogLog.log("Drivetrain/WheelSpeeds", 
            List.of(
                new SwerveModuleState(getState().leftMetersPerSecond,Rotation2d.kZero), // 前左
                new SwerveModuleState(getState().rightMetersPerSecond,Rotation2d.kZero), // 前右
                new SwerveModuleState(getState().leftMetersPerSecond,Rotation2d.kZero), // 後左
                new SwerveModuleState(getState().rightMetersPerSecond,Rotation2d.kZero) // 後右
            ).toArray(SwerveModuleState[]::new));
    }

    /**
     * 自動模式初始化保留點。
     */
    private void autoInit(){
        try{
            AutoBuilder.configure(
                getPose, 
                resetPose, 
                getChassisSpeeds, 
                (speeds, ff) -> setChassisSpeeds(speeds), 
                new PPLTVController(0.02, Constants.MaxVelocity.in(MetersPerSecond
                )), 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red, 
                this);
        }catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }
    }

    /**
     * 取得 Drivetrain 的唯一實例。
     *
     * @return Drivetrain 的單例實例。
     */
    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
