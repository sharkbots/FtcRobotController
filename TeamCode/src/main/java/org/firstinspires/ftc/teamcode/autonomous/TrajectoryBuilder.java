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

        Vector2d setUpForBackdropA = c.setUpForBackdropA;
        Vector2d setUpForBackdropB = c.setUpForBackdropB;
        Vector2d setUpForBackdropC = c.setUpForBackdropC;

        if (propLoc == propLocations.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropCoordinate = c.backdropLeft;
            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
        }
        else if (propLoc == propLocations.RIGHT) {
            teamPropCoordinate = c.rightTeamProp;
            backdropCoordinate = c.backdropRight;
            backdropIntermediateCoordinate = c.backdropIntermediateRight;
        }
        else {
            teamPropCoordinate = c.centerTeamProp;
            backdropCoordinate = c.backdropCenter;
            backdropIntermediateCoordinate = c.backdropIntermediateCenter;
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
                .lineTo(setUpForBackdropA)
                .lineTo(setUpForBackdropB)
                .lineTo(setUpForBackdropC)
                .build();
        if (c.CloseSide) {
            finalTrajectory.add(setupForBackdrop);
        }

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(setupForBackdrop.end())
                .lineToLinearHeading(backdropIntermediateCoordinate)
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
    Pose2d backdropIntermediateCoordinate;

}
