// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;

/** Add your docs here. */
public class Utils {
    public static Rotation2d getAngleBetweenPointsForShooter(Translation2d robot, Translation2d target){
        //return new Rotation2d(Math.atan2(robot.getY()-target.getY(), robot.getX()-target.getX()));
        return new Rotation2d(Math.atan2(target.getY()-robot.getY(), target.getX()-robot.getX()));
    }
}
