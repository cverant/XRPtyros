// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Turn;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */       

public class Robot extends TimedRobot {
  XboxController controller;
  
  DriveTrain drive;
  Arm arm;

  SendableChooser<Command> autoChooser;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    controller = new XboxController(0);

    //init subsystems
    drive = new DriveTrain();
    arm = new Arm();

    SmartDashboard.putData(CommandScheduler.getInstance());

    double forward = controller.getRawAxis(1);
    // trying to set a deadband {
     // if(Math.abs(forward.GetValue() < 0.1)) return;
   // }

    double turn = controller.getRawAxis(0);
    

    if (forward < .05) {
      forward = 0;
    }
    if (turn < .05) {
      turn = 0;
    }

    //register commands
    drive.setDefaultCommand(drive.controllerDrive(() -> controller.getRawAxis(1), () -> controller.getRawAxis(4))); // originally: axis 3 - axis 2
    //arm.setDefaultCommand(arm.setPosition(() -> controller.getRawAxis(5)));
    new Trigger(controller::getAButton).whileTrue(arm.setAngle(Arm.CARRY));
    new Trigger(controller::getYButton).whileTrue(arm.setAngle(Arm.PICKUP));

    //setup Auto
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("None", new InstantCommand());
    autoChooser.addOption("Ring", drive.lineFollow());
    autoChooser.addOption("Map", autoSequence());
    SmartDashboard.putData(autoChooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().schedule(autoChooser.getSelected());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double percent = -controller.getLeftY();
    double turn = -controller.getRightX();
    //use the arcade drive class to factor in the turn commands
    drive.ArcadeDrive(percent, turn);
    
    //these were used if you want to control the motors directly instead
    //leftDrive.set(percent);
    //rightDrive.set(percent);

    //drive.TankDrive(-controller.getLeftY(),-controller.getRightY());

    /*
    double position;
    if (controller.getAButton() == true) {
      position = 0.25;
    } else if (controller.getYButton() == true) {
      position = 0.75;
    } else {
      //the example has this at 0-1, but I was running into the servo PID struggling to hit the ends, so to save the servos I clampped the output.  I didn't want to explain that in this simple demo. 
      position = MathUtil.clamp(controller.getRightTriggerAxis(), 0.1, 0.9);
    }
    arm.setPosition(position);
    */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private Command autoSequence() {
    var driveToCup = arm.setAngle(Arm.PICKUP)
      .alongWith(drive.lineFollow())
      .until(() ->  drive.distanceSensor() < 0.039);

    var turnAround = arm.setAngle(Arm.CARRY)
      .raceWith(
        //time to lift the cup
        new WaitCommand(0.4)
        //turn around
        .andThen(drive.turnToLine())
        //follow to right angle turn
        .andThen(drive.lineFollow())
        //drive a bit extra to get off the line
        .andThen(new DriveDistance(drive, 0.5))
        //turn left till we find our line again
        .andThen(drive.turnToLine())
        //drive forward up the triangle
        .andThen(drive.lineFollow())
        //drive to goal
        .andThen(new DriveDistance(drive, 24))
      )
    ;
    
    var score = 
    arm.setAngle(Arm.PICKUP)
      .raceWith(
        new WaitCommand(0.4)
        .andThen(new DriveDistance(drive, -4))
      );

    //var driveDistance = drive.lineFollow().andThen(new DriveDistance(drive, 24));

    return driveToCup.andThen(turnAround).andThen(score);
    //return turnAround.andThen(score);
  }
}