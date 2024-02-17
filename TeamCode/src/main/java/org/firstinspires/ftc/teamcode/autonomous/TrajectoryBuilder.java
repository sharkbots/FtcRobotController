package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.autonomous.AutoBase.SLOWERANGULARVELOCITY;
import static org.firstinspires.ftc.teamcode.autonomous.AutoBase.SLOWERVELOCITY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class TrajectoryBuilder {
    public AutoBase.Coordinates c;
    public SampleMecanumDrive drive;

    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive){
        this.c = coordinates;
        this.drive = drive;

        trajectorySequenceLeft = computeTrajectory(propLocations.LEFT, drive, c);
        trajectorySequenceCenter = computeTrajectory(propLocations.CENTER, drive, c);
        trajectorySequenceRight = computeTrajectory(propLocations.RIGHT, drive, c);
    }

    public ArrayList<TrajectorySequence> computeTrajectory(propLocations propLoc, SampleMecanumDrive drive, AutoBase.Coordinates c) {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

        if (propLoc == propLocations.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropCoordinate = c.backdropLeft;
        }
        else if (propLoc == propLocations.RIGHT) {
            teamPropCoordinate = c.rightTeamProp;
            backdropCoordinate = c.backdropRight;
        }
        else {
            teamPropCoordinate = c.centerTeamProp;
            backdropCoordinate = c.backdropCenter;
        }

        assert drive != null;
        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.preStartPose)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPose)
                .lineToLinearHeading(teamPropCoordinate)
                .build();
        finalTrajectory.add(purpleDrop);

        TrajectorySequence setupForBackdrop = drive.trajectorySequenceBuilder(purpleDrop.end())
                .back(3.5)
                .forward(3)
                .lineTo(new Vector2d(51, 11))
                .lineTo(new Vector2d(-8.5, 11))
                //.lineTo(new Vector2d(-8.5, 35))
                .build();
        finalTrajectory.add(setupForBackdrop);
        if (c.CloseSide) {
            finalTrajectory.add(setupForBackdrop);
        }

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(setupForBackdrop.end())
                .lineToLinearHeading(c.backdropIntermediateCenter)
                .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        if (c.CloseSide) {
            finalTrajectory.add(goToBackdrop);
        }

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(8)
                .lineToLinearHeading(c.parkIntermediate)
                .lineToLinearHeading(c.parkFinal)
                .build();
        finalTrajectory.add(parkRight);
        if (c.CloseSide) {
            finalTrajectory.add(parkRight);
        }

        return finalTrajectory;
    }

    public enum propLocations{
        LEFT,
        CENTER,
        RIGHT
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;

}
