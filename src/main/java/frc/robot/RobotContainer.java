// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvAim;
import frc.robot.subsystems.Vision.FiducialVision;
import frc.robot.subsystems.Vision.ObjectVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  ObjectVision objectVision;
  FiducialVision fiducialVision;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the rotational velocity 
    // buttons are quick rotation positions to different ways to face
    // and a button will yaw the robot towards a target.
    // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
    AbsoluteDriveAdvAim closedAbsoluteDriveAdvAim = new AbsoluteDriveAdvAim(drivebase,
                                                                      () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                    OperatorConstants.LEFT_Y_DEADBAND) *
                                                                                                    DrivebaseConstants.Max_Speed_Multiplier,
                                                                      () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                    OperatorConstants.LEFT_X_DEADBAND) *
                                                                                                    DrivebaseConstants.Max_Speed_Multiplier,
                                                                      () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                    OperatorConstants.RIGHT_X_DEADBAND) *
                                                                                                    .5,
                                                                      driverXbox.povUp(),
                                                                      driverXbox.povDown(),
                                                                      driverXbox.povRight(),
                                                                      driverXbox.povLeft(),
                                                                      driverXbox.a());

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? closedAbsoluteDriveAdvAim : closedAbsoluteDriveAdvAim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //     Commands.deferredProxy(() -> drivebase.driveToPose(
    //                                new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                           ));
    // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
    driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.sysIdAngleMotorCommand()));

    driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.sysIdDriveMotorCommand()));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        // driverXbox.x().whileTrue(new DriveToAprilTagPosCmd("Speaker",
    //                                                    1.7,
    //                                                    0.0,
    //                                                    0.1,
    //                                                    vision,
    //                                                    drivebase)); 
    
    driverXbox.b().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                             FiducialVision.getAprilTagPose(AprilTagConstants.speakerID,
                                                            new Transform2d(1.7, 0,
                                                            Rotation2d.fromDegrees(0))))));

    // driverXbox.b().whileTrue(new DriveToNoteCmd(drivebase).andThen
    //                         (new DriveDistancePPID(-.5, 0, 0, .1, drivebase)));

    // driverXbox.b().whileTrue(new DriveToNoteCmd(drivebase).andThen
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(0.5, 0.0, 0.0))));

    // driverXbox.a().whileTrue(Commands.deferredProxy(() ->
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(1.0, 1.0, 30.0)))));

    // driverXbox.y().whileTrue(Commands.deferredProxy(() ->
    //                         (drivebase.driveToPose(drivebase.getOffsetPose(-1.0, -1.0, -30.0)))));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

    /**
   * Sets the speed multiplier for the drivebase based on the state of the right and left bumpers on the driver's Xbox controller.
   * If both bumpers are pressed, the speed multiplier is set to 1 (HighSpd).
   * If either bumper is pressed, the speed multiplier is set to 0.75 (MedSpd).
   * If neither bumper is pressed, the speed multiplier is set to 0.50 (LowSpd).
   */
  public void spencerButtons(){

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      DrivebaseConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .75;
    }
    
  }

}
