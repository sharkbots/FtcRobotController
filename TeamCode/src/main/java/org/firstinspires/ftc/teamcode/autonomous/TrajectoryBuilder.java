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

        Vector2d prepareFarDrop = c.prepareFarDrop;
        Vector2d parkFinalLeft = c.parkFinalLeft;
        Vector2d backdropIntermediateFar = c.backdropIntermediateFar;
        Vector2d intermediateDropFar = c.intermediateDropFar;

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

        TrajectorySequence setupForBackdropClose = drive.trajectorySequenceBuilder(purpleDrop.end())
                .back(3.5)
                .forward(3)
                .lineTo(setUpForBackdropA)
                .lineTo(setUpForBackdropB)
                .lineTo(setUpForBackdropC)
                .build();

        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(prepareFarDrop)
                .lineTo(backdropIntermediateFar)
                .build();

        if (c.CloseSide) {
            finalTrajectory.add(setupForBackdropClose);
        } else{
            finalTrajectory.add(setupForBackdropFar);
        }

        TrajectorySequence goToBackdropClose = drive.trajectorySequenceBuilder(setupForBackdropClose.end())
                .lineToLinearHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*TrajectorySequence gotoBackdropFar = drive.trajectorySequenceBuilder(setupForBackdropFar.end())
                .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();*/
        TrajectorySequence gotoBackdropFar = drive.trajectorySequenceBuilder(setupForBackdropFar.end())
                .lineTo(intermediateDropFar)
                .lineToLinearHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate)
                .build();

        if (c.CloseSide) {
            finalTrajectory.add(goToBackdropClose);
        }else{
            finalTrajectory.add(gotoBackdropFar);
        }

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdropClose.end())
                .forward(8)
                .lineToLinearHeading(c.parkIntermediate)
                .lineToLinearHeading(c.parkFinalRight)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(gotoBackdropFar.end())
                .lineTo(parkFinalLeft)
                .build();

        if (c.CloseSide) {
            finalTrajectory.add(parkRight);
        }else{
            finalTrajectory.add(parkLeft);
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
