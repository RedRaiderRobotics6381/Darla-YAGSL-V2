package frc.robot.commands.swervedrive.drivebase;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.ObjectVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class DriveToNoteCmd extends Command
{
  private final SwerveSubsystem swerveSubsystem;
  private PIDController   xController;
  private PIDController   zController;
  private boolean hasTargets;
  private double TX = 0;
  private double TZ = 0;
  boolean droveToNote = false;
  ObjectVision m_Vision;

  public DriveToNoteCmd(SwerveSubsystem swerveSubsystem)
  {
    this.swerveSubsystem = swerveSubsystem;
    
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(swerveSubsystem);
  }

  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    droveToNote = false;
    hasTargets = false;
    xController = new PIDController(0.125, 0.00, 0.0);
    //yController = new PIDController(0.0625, 0.00375, 0.0001);
    zController = new PIDController(0.09,0.0, 0.000);
    xController.setTolerance(1);
    //yController.setTolerance(3);
    zController.setTolerance(1);
  }

    /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   *
   * This method retrieves the latest result from PhotonVision and checks if there are any targets.
   * If targets are found, it calculates the translation values for the x and z axes using PID controllers.
   * It then drives the swerve subsystem based on the calculated translation values.
   * If the swerve subsystem reaches the x-axis setpoint, it initiates a drive distance command to approach the note.
   */
  @Override
  public void execute()
  {
    var result = ObjectVision.camObj.getLatestResult();  // Get the latest result from PhotonVision
    hasTargets = result.hasTargets(); // Check if the latest result has any targets.
    PhotonTrackedTarget target = result.getBestTarget(); // Get the best target from the latest result.
    if (hasTargets == true) { // If there are targets found
      TZ = target.getYaw(); // Get the yaw of the target
      
      //The code below is to track a single note instead of sometimes driving towards a note further ahead that it might see.
      if (TX == 0) { //If the target pitch is 0 (the first time the code runs), set the target pitch to the current pitch
        TX = target.getPitch();
      } else if (target.getPitch() <= TX) {
        TX = target.getPitch(); //If the target pitch is less than the current pitch, set the target pitch to the current pitch
      }

      double translationValx = MathUtil.clamp(xController.calculate(TX, -9), -4 , 4); //Tune the setpoint to be where the note is just barely found.
      double translationValz = MathUtil.clamp(zController.calculate(TZ, 0.0), -6.0 , 6.0); //* throttle, 2.5 * throttle);

      if(droveToNote != true) { // If the swerve subsystem has not driven to the note
        if (xController.atSetpoint() != true) { // If the swerve subsystem has not reached the x-axis setpoint
            swerveSubsystem.drive(new Translation2d(translationValx, 0.0), translationValz, false); // Drive the swerve subsystem based on the calculated translation values
          } else { // If the swerve subsystem has reached the x-axis setpoint
            //new DriveDistancePPID(.5, 0.0, 0.0, .1, swerveSubsystem); // Initiate a drive distance command to approach the note
            droveToNote = true; // Set droveToNote to true
          }
      }
    }    
  }


  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return droveToNote; // Return whether the swerve subsystem has driven to the note
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {
    //swerveSubsystem.lock();
    TX = 0;
    droveToNote = false;
    //Robot.LEDsSubSystem.fireEffect(); // Fire the LEDs effect
  }
}