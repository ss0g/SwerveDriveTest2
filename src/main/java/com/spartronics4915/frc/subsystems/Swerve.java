package com.spartronics4915.frc.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.spartronics4915.frc.Constants.Swerve.*;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

public class Swerve extends SubsystemBase {
    private SwerveDriveOdometry mOdometry;
    private SwerveModule[] mModules;
    private AHRS mNavX;

    private boolean mIsFieldRelative = true;

    public Swerve() {
        mNavX = new AHRS();
        mNavX.reset();

        mOdometry = new SwerveDriveOdometry(kKinematics, getYaw());

        mModules = new SwerveModule[] {
            new SwerveModule(0, Module0.kConstants),
            new SwerveModule(1, Module1.kConstants),
            new SwerveModule(2, Module2.kConstants),
            new SwerveModule(3, Module3.kConstants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds;
        SmartDashboard.putBoolean("field relative", mIsFieldRelative);
        if (mIsFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getYaw()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation
            );
        }

        SwerveModuleState[] moduleStates = kKinematics.toSwerveModuleStates(chassisSpeeds);
        
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(moduleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeed);

        for (SwerveModule mod : mModules) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public void setFieldRelative(boolean fieldRelative) {
        mIsFieldRelative = fieldRelative;
    }

    public boolean getFieldRelative() {
        return mIsFieldRelative;
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(-mNavX.getYaw());
    }

    public void resetOdometry(Pose2d pose) {
        mOdometry.resetPosition(pose, getYaw());
    }

    public void resetModuleZeroes() {
        for (SwerveModule mod : mModules) {
            mod.resetToAbsolute();
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mModules) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public void zeroNavX() {
        mNavX.reset();
    }

    public void zeroModules() {
        SwerveModuleState[] zeroedStates = new SwerveModuleState[4];
        Arrays.fill(zeroedStates, new SwerveModuleState(0, new Rotation2d(0)));
        // for (SwerveModuleState state : zeroedStates) {
        //     state = new SwerveModuleState(0, new Rotation2d(0));
        // }
        setModuleStates(zeroedStates);
    }

    @Override
    public void periodic() {
        mOdometry.update(getYaw(), getStates());
        for (SwerveModule mod : mModules) {
            mod.putSmartDashboardValues();
        }
    }
}
