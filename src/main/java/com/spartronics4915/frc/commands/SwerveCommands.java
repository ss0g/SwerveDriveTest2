package com.spartronics4915.frc.commands;

import com.spartronics4915.frc.subsystems.Swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static com.spartronics4915.frc.Constants.Swerve.*;
import static com.spartronics4915.frc.Constants.OI.*;

public class SwerveCommands {
    private final XboxController mController;

    private final Swerve mSwerve;
    private boolean mIsFieldRelative = true;

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
        public void end(boolean interrupted) {}

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class ToggleFieldRelative extends ConditionalCommand {
        public ToggleFieldRelative() {
            super(
                new SetFieldRelative(false),
                new SetFieldRelative(true),
                mSwerve::getFieldRelative
            );
        }
    }

    public class TeleopInitCommand extends CommandBase {
        public TeleopInitCommand() {
            addRequirements(mSwerve);
        }

        @Override
        public void initialize() {
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
            
            mSwerve.drive(translation, -x2, true);
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
        return applyResponseCurve(MathUtil.applyDeadband(c, kDeadband));
    }

    private double applyResponseCurve(double c) {
        return Math.signum(c) * Math.pow(Math.abs(c), kResponseCurveExponent);
    }
}
