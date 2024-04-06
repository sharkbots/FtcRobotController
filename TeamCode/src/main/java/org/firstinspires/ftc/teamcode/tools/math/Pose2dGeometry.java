package org.firstinspires.ftc.teamcode.tools.math;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Pose2dGeometry {

    public Pose2dGeometry(){

    }

    public static Pose2d getPointInTransformedCoordinateSystem(Pose2d firstCenter, Pose2d secondCenter, Pose2d point){
        Pose2d pointInRotatedCoordinateSystem = getPointInRotatedCoordinateSystem(firstCenter, secondCenter, point);
        return getPointInTranslatedCoordinateSystem(firstCenter, secondCenter, pointInRotatedCoordinateSystem);
    }
    
    public static Vector2d getPointInTranslatedCoordinateSystem(Vector2d firstCenter, Vector2d secondCenter, Vector2d point){

        double translationX = secondCenter.getX()-firstCenter.getX();
        double translationY = secondCenter.getY()-firstCenter.getY();
        Vector2d translationVector = new Vector2d(translationX, translationY);

        return point.minus(translationVector);

    }
    public static Pose2d getPointInTranslatedCoordinateSystem(Pose2d firstCenter, Pose2d secondCenter, Pose2d point){
        double x = getPointInTranslatedCoordinateSystem(firstCenter.vec(), secondCenter.vec(), point.vec()).getX();
        double y = getPointInTranslatedCoordinateSystem(firstCenter.vec(), secondCenter.vec(), point.vec()).getY();

        return new Pose2d(x, y, point.getHeading());
    }



    public static Pose2d getPointInRotatedCoordinateSystem(Pose2d firstCenter, Pose2d secondCenter, Pose2d point){
        double rotation = secondCenter.getHeading()-firstCenter.getHeading();

        double x = point.getX()*Math.cos(rotation)+ point.getY()*Math.sin(rotation);
        double y = -point.getX()*Math.sin(rotation)+ point.getY()*Math.cos(rotation);

        return new Pose2d(x, y, point.getHeading()-rotation);
    }






}
