
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.SwerveDrive


class DriveCommand(private val swerve: SwerveDrive): CommandBase() {
    init {
        addRequirements(swerve)
    }


}