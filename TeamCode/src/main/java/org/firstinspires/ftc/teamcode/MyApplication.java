package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


public class MyApplication extends Application {
    @Override
    public void onCreate() {
        super.onCreate();

        Log.d("PLAYGROUND", "Init SharkBots");
        AppUtil.onApplicationStart(this); // Necessary to make FTC Libraries work


    }

}