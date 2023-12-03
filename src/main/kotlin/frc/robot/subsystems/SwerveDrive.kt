package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.utils.SwerveDriveModule
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


/** Creates a new ExampleSubsystem.  */
class SwerveDrive(private val gyro: Pigeon2) : SubsystemBase() {
    val field = Field2d();

    val frontLeft = SwerveDriveModule(1,2,3,CANSparkMaxLowLevel.MotorType.kBrushless,true,"front left");
    val frontRight = SwerveDriveModule(4,5,6,CANSparkMaxLowLevel.MotorType.kBrushless,true,"front right");
    val backLeft = SwerveDriveModule(7,8,9,CANSparkMaxLowLevel.MotorType.kBrushless,true,"back left");
    val backRight = SwerveDriveModule(10,11,12,CANSparkMaxLowLevel.MotorType.kBrushless,true,"back right");

    val swervekinematics = SwerveDriveKinematics(Translation2d(-0.5,0.5),Translation2d(0.5,0.5),Translation2d(0.5,-0.5),Translation2d(-0.5,0.5));

    var x = 0.0;
    var y = 0.0;

    enum class DrivingStyle {
        RobotRelative,
        FieldRelative,
    }

    var relativeTo = DrivingStyle.RobotRelative;

    var vx = 0.0;
    var vy = 0.0;
    var omega = 0.0;

    var lastTime = 0.0;
    
    init {
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    fun exampleCondition(): Boolean {
        // Query some boolean state, such as a digital sensor.
        return false
    }

    /** This method will be called once per scheduler run  */
    override fun periodic() {
        var vx2 = vx;
        var vy2 = vy;
        var omega2 = omega;

        if (relativeTo == DrivingStyle.FieldRelative) {
            val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega2, Rotation2d.fromDegrees(gyro.yaw)); // lets hope it is degrees
            vx2 = speeds.vxMetersPerSecond;
            vy2 = speeds.vyMetersPerSecond;
            omega2 = speeds.omegaRadiansPerSecond;
        }
        val swerveStates = swervekinematics.toSwerveModuleStates(ChassisSpeeds(vx2,vy2,omega2));
        frontLeft.setAngle(swerveStates[0].angle);
        frontRight.setAngle(swerveStates[1].angle);
        backRight.setAngle(swerveStates[2].angle);
        backLeft.setAngle(swerveStates[2].angle);

        frontLeft.setSpeed(swerveStates[0].speedMetersPerSecond);
        frontRight.setSpeed(swerveStates[1].speedMetersPerSecond);
        backRight.setSpeed(swerveStates[2].speedMetersPerSecond);
        backLeft.setSpeed(swerveStates[2].speedMetersPerSecond);


        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();

        val chasisSpeeds = 

        SwerveDriveKinematics.desaturateWheelSpeeds([frontLeft.state,frontRight.state,backRight.state,backLeft.state], Constants.kMaxSpeed);

        swervekinematics.toChassisSpeeds(frontLeft.state,
                                     frontRight.state,
                                     backRight.state,
                                     backLeft.state);
    }

    public fun setvx(vx:Double) {
        this.vx = vx;
    }

    public fun setvy(vy:Double) {
        this.vy = vy;
    }

    public fun setRotationSpeed(omega: Double) {
        this.omega = omega;
    }


    /** This method will be called once per scheduler run during simulation  */
    override fun simulationPeriodic() {
        var currentTime = Timer.getFPGATimestamp();
        var delta = (currentTime - lastTime);
            frontLeft.simulate(delta);
            frontRight.simulate(delta);
            backLeft.simulate(delta);
            backRight.simulate(delta);


        lastTime = currentTime;

        val chasisSpeeds = 
            swervekinematics.toChassisSpeeds(frontLeft.state,
                                         frontRight.state,
                                         backRight.state,
                                         backLeft.state);
        gyro.simCollection.addHeading(chasisSpeeds.omegaRadiansPerSecond * 180/Math.PI * delta);

        val fieldRel = 

        x += chasisSpeeds.vxMetersPerSecond * delta;
        y += chasisSpeeds.vyMetersPerSecond * delta;
    }
}
