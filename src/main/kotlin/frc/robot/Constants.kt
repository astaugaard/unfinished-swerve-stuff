package frc.robot

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
    object OperatorConstants {
        const val kDriverControllerPort = 0
    }

    object Robot {
        const val robotMass = 56.0;
        const val robotWeight = robotMass * 9.81;
        const val cofficientOfFrictionOfRubber = 1.0;
        // Force of friction = µgMR^2/12
        const val turningFrictionForce = cofficientOfFrictionOfRubber * 9.81 * robotMass * 0.00185161/12;
        const val wheelRadius = 0.1016/2;
        const val turningWheelMomentOfInertia = 0.2494758 * wheelRadius * wheelRadius * 0.5;
        const val driveWheelMomentOfInertia = 0.2494758 * wheelRadius * wheelRadius;
    }


}
