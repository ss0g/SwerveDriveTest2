package com.spartronics4915.frc.commands;

import com.spartronics4915.frc.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static com.spartronics4915.frc.Constants.Swerve.*;
import static com.spartronics4915.frc.Constants.OI.*;

public class SwerveCommands {
    private final XboxController mController;

    private final Swerve mSwerve;
    private boolean mIsFieldRelative = true;
    private boolean mIsSlowMode = false;

    public SwerveCommands(XboxController controller, Swerve swerve) {
        mController = controller;
        mSwerve = swerve;
    }

    public class SetFieldRelative extends CommandBase {
        public SetFieldRelative(boolean fieldRelative) {
            mIsFieldRelative = fieldRelative;
        }

        @Override
        public void initialize() {
            mSwerve.setFieldRelative(mIsFieldRelative);
        }

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {
            mIsFieldRelative = mSwerve.getFieldRelative(); // just to make sure
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ToggleFieldRelative extends CommandBase {
        public ToggleFieldRelative() {}

        @Override
        public void initialize() {
            mSwerve.toggleFieldRelative();
        }

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {
            mIsFieldRelative = mSwerve.getFieldRelative();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ResetYaw extends CommandBase {
        public ResetYaw() {}

        @Override
        public void initialize() {
            mSwerve.zeroNavX();
        }

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ResetOdometry extends CommandBase {
        public ResetOdometry() {}

        @Override
        public void initialize() {
            mSwerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        }

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class TeleopInitCommand extends CommandBase {
        public TeleopInitCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {
            mSwerve.resetOdometry(new Pose2d()); // for odometry testing
            mSwerve.zeroModules();
        }

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
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

            Translation2d translation = new Translation2d(-y1, -x1).times(kMaxSpeed);
            double rotation = -x2 * kMaxAngularSpeed;

            if (Math.abs(mController.getRawAxis(kSlowModeAxis)) <= kTriggerDeadband) { // <= for slow mode default
                translation = translation.times(kSlowModeSpeedMultiplier);
                rotation *= kSlowModeAngularSpeedMultiplier;
            }
            
            mSwerve.drive(translation, rotation, true);
        }

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public class TestInitCommand extends CommandBase {
        public TestInitCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class TestCommand extends CommandBase {
        public TestCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private double applyTransformations(double c) {
        return applyResponseCurve(MathUtil.applyDeadband(c, kStickDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }
}
