package com.spartronics4915.frc.commands;

import com.spartronics4915.frc.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static com.spartronics4915.frc.Constants.OI.*;

public class SwerveCommands {
    private final XboxController mController;

    private final Swerve mSwerve;

    public SwerveCommands(XboxController controller, Swerve swerve) {
        mController = controller;
        mSwerve = swerve;
    }

    public class TeleopCommand extends CommandBase {
        public TeleopCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            double x1 = mController.getLeftX();
            double y1 = mController.getLeftY();
            double x2 = mController.getRightX();

            x1 = applyTransformations(x1);
            y1 = applyTransformations(y1);
            x2 = applyTransformations(x2);

            Translation2d translation = new Translation2d(-y1, -x1);
            
            mSwerve.drive(translation, -x2, !mController.getRawButton(kRobotOrientedButton), true);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class TestingCommand extends CommandBase {
        private double mDesiredAngle;
        private SwerveModuleState[] mDesiredStates;
        
        public TestingCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {
            mSwerve.resetTestingAngle();
            mDesiredAngle = 0;
            mDesiredStates = new SwerveModuleState[4];
            for (SwerveModuleState s : mDesiredStates) {
                s = new SwerveModuleState(0, new Rotation2d(mDesiredAngle));
            }
            mSwerve.setModuleStates(mDesiredStates);
        }

        @Override
        public void execute() {
            if (mController.getRawButtonPressed(kTestingButton)) {
                mDesiredAngle = mSwerve.getTestingAngleSupplier().getAsDouble();
                for (SwerveModuleState s : mDesiredStates) {
                    s = new SwerveModuleState(0, new Rotation2d(mDesiredAngle));
                }
                mSwerve.setModuleStates(mDesiredStates);
            } // does this work?
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    private double applyTransformations(double c) {
        return applyResponseCurve(MathUtil.applyDeadband(c, kDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }
}
