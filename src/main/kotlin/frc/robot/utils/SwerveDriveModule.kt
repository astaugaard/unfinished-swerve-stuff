package frc.robot.utils

import com.ctre.phoenix.sensors.CANCoder
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import edu.wpi.first.wpilibj.simulation.PWMSim
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand
import kotlin.math.abs
import kotlin.math.sign
import frc.robot.Constants
import javax.sound.sampled.Control

class SwerveDriveModule(turningId: Int, driveId: Int, cancoderId: Int, motorType: CANSparkMaxLowLevel.MotorType, public val rotateInverted: Boolean, private val name:String) {
    val turningMotor = CANSparkMax(turningId,motorType);
    val driveMotor = CANSparkMax(driveId,motorType);

    val turningPid = PIDController(0.087500,0.0,0.0);
    val drivePid = PIDController(0.0, 0.0, 0.0);

    var turningMotorSpeed = 0.0;
    var turningMotorPosition = 0.0;

    var driveMotorSpeed = 0.0;

    val turningMotorT = DCMotor.getNEO(1).withReduction(7.0/150);


    val driveMotorT = DCMotor.getNEO(1).withReduction(1/6.21);

    var goalAngle = 1.0;

    var goalSpeed = 1.0;

    // var turningMotionCurve = TrapezoidProfile.Constraints(Math.PI*32,Math.PI*32);

    val cancoder = CANCoder(cancoderId);

    // var currentState = TrapezoidProfile.State(0.0,0.0);

    init {
        // todo set up can coder initialization of angles
        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();
        
        driveMotor.encoder.velocityConversionFactor = 1.0/6.12 * 2.0 * Math.PI * 0.1016/60;
        turningMotor.encoder.positionConversionFactor = 7.0/150.0 * 2.0 * Math.PI;
        turningMotor.encoder.velocityConversionFactor = 7.0/150.0 * 2.0 * Math.PI/60;

        turningPid.enableContinuousInput(0.0, 2.0 * Math.PI);

        val sparkMaxPid = turningMotor.getPIDController();
        sparkMaxPid.setP(turningPid.getP());
        sparkMaxPid.setI(turningPid.getI());
        sparkMaxPid.setD(turningPid.getD());

        val sparkMaxPidDrive = driveMotor.getPIDController();
        sparkMaxPidDrive.setP(drivePid.getP());
        sparkMaxPidDrive.setI(drivePid.getI());
        sparkMaxPidDrive.setD(drivePid.getD());

        SmartDashboard.putData(name + " turning pid", turningPid);
        SmartDashboard.putData(name + " drive pid", drivePid);

        turningMotor.set(0.0);
        SmartDashboard.putNumber(name + " goal angle", 1.0);
        SmartDashboard.putNumber(name + " goal speed", 1.0);
    }

    fun periodic() {
        goalAngle = SmartDashboard.getNumber(name + " goal angle", 1.0);
        goalSpeed = SmartDashboard.getNumber(name + " goal speed", 1.0);

        // val endState = TrapezoidProfile.State(goalAngle, 0.0);

        // val profile = TrapezoidProfile(turningMotionCurve, endState, currentState);

        // currentState = profile.calculate(0.02);

        // SmartDashboard.putNumber(name + " reference angle", currentState.position);

        if (!RobotBase.isReal()) {
             // using the spark max pid controllers on a real robot
             turningMotor.set(turningPid.calculate(turningMotor.encoder.position, goalAngle)); // simulating spark max pid controllers
             driveMotor.set(drivePid.calculate(driveMotorSpeed, goalSpeed))
        }
    }

    public fun setAngle(angle: Rotation2d) {
        goalAngle = angle.radians;
        turningMotor.getPIDController().setReference(goalAngle, CANSparkMax.ControlType.kPosition);
    }

    public fun setSpeed(speed: Double) {
        driveMotor.getPIDController().setReference(speed,CANSparkMax.ControlType.kVelocity);
        goalSpeed = speed/Constants.Robot.wheelRadius;
    }

    fun simulate(timestep: Double) {

        val rotateVoltage = turningMotor.get() * RobotController.getBatteryVoltage();
        val driveVoltage = driveMotor.get() * RobotController.getBatteryVoltage();

        var driveTorque = driveVoltage.sign.toDouble() * driveMotorT.getCurrent(Math.min(Math.max(driveMotorSpeed,1.0),-1.0) * 6.12, abs(driveVoltage)) * driveMotorT.KtNMPerAmp;
        SmartDashboard.putNumber(name + " drive torque", driveTorque);

        val R = Constants.Robot.wheelRadius;
        val I = Constants.Robot.driveWheelMomentOfInertia;

        var forwardForce = driveTorque * R;

        val driveAngleAcceleration = forwardForce/(I/R + R * Constants.Robot.robotMass);

        driveMotorSpeed += driveAngleAcceleration * timestep;
        
        var inversionSign = 1.0;
        if (rotateInverted) {
            inversionSign = -1.0;
        }
        
        var torque = inversionSign.toDouble() * rotateVoltage.sign.toDouble() * turningMotorT.getCurrent(turningMotorSpeed * 150/7, abs(rotateVoltage)) * turningMotorT.KtNMPerAmp;
        // if (abs(turningMotorSpeed * 7/150/turningMotorT.KvRadPerSecPerVolt) > abs(rotateVoltage)) {
        //     torque = 0.0;
        // }
        var frictionForce = 0.0;

        if (abs(turningMotorSpeed) > 0) {
            frictionForce = -turningMotorSpeed.sign * Constants.Robot.turningFrictionForce;
        }

        SmartDashboard.putNumber(name + "friction force", frictionForce);

        // torque += frictionForce;
        val angularAcceleration = torque/Constants.Robot.turningWheelMomentOfInertia;
        // turningMotorSpeed += angularAcceleration * timestep;

        val frictionAcceleration = frictionForce/Constants.Robot.turningWheelMomentOfInertia
        if (turningMotorSpeed == 0.0 && Constants.Robot.turningFrictionForce >= abs(angularAcceleration)) {
            // noop 
        } else if (abs(frictionAcceleration * timestep) > abs(angularAcceleration * timestep + turningMotorSpeed)) {
            turningMotorPosition += turningMotorSpeed*turningMotorSpeed/2   *(frictionAcceleration + angularAcceleration);
            turningMotorSpeed = 0.0;
        } else {            
            turningMotorPosition += turningMotorSpeed * timestep + 1/2 * (angularAcceleration + frictionAcceleration) * timestep * timestep;
            turningMotorSpeed += (angularAcceleration + frictionAcceleration) * timestep;
        }

        turningMotorPosition %= 2 * Math.PI;
        turningMotorPosition += 2*Math.PI;
        turningMotorPosition %= 2 * Math.PI;

        turningMotor.encoder.position = turningMotorPosition;

        SmartDashboard.putNumber(name + " angular position", turningMotorPosition % (2 * Math.PI));
        SmartDashboard.putNumber(name + " angular velocity", turningMotorSpeed);
        SmartDashboard.putNumber(name + " turning angular acceleration", angularAcceleration);
        SmartDashboard.putNumber(name + " friction", frictionForce);
        SmartDashboard.putNumber(name + " drive speed", driveMotorSpeed);
        SmartDashboard.putNumber(name + " drive speed goal", goalSpeed);
    }

    val state: SwerveModuleState
    get() {
        if (RobotBase.isReal()) {
            return SwerveModuleState(driveMotor.encoder.velocity,
                                     Rotation2d.fromRadians(turningMotor.encoder.position));
        }
        return SwerveModuleState(driveMotorSpeed, 
                  Rotation2d.fromRadians(turningMotorPosition));
    }
}