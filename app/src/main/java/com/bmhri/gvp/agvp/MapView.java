package com.bmhri.gvp.agvp;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.os.Environment;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.SurfaceHolder;

import java.io.File;

import static com.bmhri.gvp.agvp.Running_state.INITING;
import static com.bmhri.gvp.agvp.Running_state.MOVING;
import static com.bmhri.gvp.agvp.Running_state.PLANNING;
import static com.bmhri.gvp.agvp.Running_state.STOPED;

/**
 * Created by shydh on 11/27/17.
 * SurfaceView
 */

//子线程标志位，用来控制子线程
enum Running_state {
    INITING, PLANNING, MOVING, STOPED
}

public class MapView extends SurfaceView
        implements SurfaceHolder.Callback, Runnable {
    static {
        System.loadLibrary("robo-lib");
    }

    static final String TAG = "SurfaceView";

    private String sdDir;

    private TaskLab mTaskLab;
    private TaskItem mTaskItem;

    int scalX, scalY;

    //可以控制SurfaceView的大小，格式，可以监控或者改变SurfaceView
    SurfaceHolder mSurfaceHolder;

    //子线程标志位，用来控制子线程
    boolean isMoving;
    Running_state state;

    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {

    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        scalX = width;
        scalY = height;
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        freeMoveBase();
        isMoving = false;
    }

    @Override
    public void run() {
        switch (state) {
            case INITING:
                getMap();
                if (initRobo()) {
                    //drawing();
                    Log.i(TAG, "initing!!");
                }
                break;
            case PLANNING:
                Log.i(TAG, "planning!!");
                if(planMoveBase()) {
                    loadPath(mSurfaceHolder.getSurface(), scalX, scalY);
                }
                break;
            case MOVING:
                while (isMoving) {
                    Log.i(TAG, "moving!!");
                    try {
                        Thread.sleep(1000);//睡眠时间为1秒
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                break;
            case STOPED:
                Log.i(TAG, "stoped!!");
                break;
            default:
                break;
        }
    }

    private void init() {
        mSurfaceHolder = getHolder();//得到SurfaceHolder对象
        mSurfaceHolder.addCallback(this);//注册SurfaceHolder
        setFocusable(true);
        setFocusableInTouchMode(true);
        this.setKeepScreenOn(true);//保持屏幕长亮

        isMoving = false;
        state = INITING;
    }

    private boolean initRobo() {
        mTaskLab = TaskLab.get();
        mTaskItem = mTaskLab.getTask();
        Log.i(TAG, "Got a goal: " + mTaskItem.getId());

        double[] current = {60, 200, 0};
        double[] goal = {180, 128, 0};

        if (initMoveBase(current, goal)) {
            Log.i(TAG, "Move_base inited!! ");
            return true;
        } else {
            Log.i(TAG, "Move_base failed initial!!  ");
            return false;
        }
    }

    private void getMap() {
        getSDPath();

        PGMLoader pgm = new PGMLoader();
        pgm.readPGMHeader(sdDir);

        loadIMG(sdDir, mSurfaceHolder.getSurface(), scalX, scalY);
    }

    private void getSDPath() {
        boolean sdCardExist = Environment.getExternalStorageState()
                .equals(android.os.Environment.MEDIA_MOUNTED); //判断sd卡是否存在
        if (sdCardExist) {
            File file = Environment.getExternalStorageDirectory();//获取根目录
            sdDir = file.toString();
            sdDir += "/agvp_maps/map.pgm";
            Log.i(TAG, sdDir);
        }
    }

    public void initRobot() {
        isMoving = false;
        state = INITING;
        new Thread(this).start();
    }

    public void planning() {
        isMoving = false;
        state = PLANNING;
        new Thread(this).start();
    }

    public void start() {
        isMoving = true;
        state = MOVING;
        new Thread(this).start();
    }

    public void stoprobo() {
        isMoving = false;
        state = STOPED;
        new Thread(this).start();
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void loadIMG(String strDir, Surface surface, int w, int h);

    public native void loadPath(Surface surface, int w, int h);

    public native void freeMoveBase();

    public native boolean initMoveBase(double[] current, double[] goal);

    public native boolean planMoveBase();
}
