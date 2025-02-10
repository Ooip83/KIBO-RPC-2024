package jp.jaxa.iss.kibo.rpc.fullrun;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import org.opencv.android.OpenCVLoader;

public class MainActivity extends AppCompatActivity{

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        if (!OpenCVLoader.initDebug()) {
            Log.i("OpenCV", "Unable to load OpenCV!");
        } else {
            Log.i("OpenCV", "OpenCV loaded Successfully!");
        }
    }
}


