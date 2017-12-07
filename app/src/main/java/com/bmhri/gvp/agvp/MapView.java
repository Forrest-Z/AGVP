package com.bmhri.gvp.agvp;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Rect;
import android.os.Environment;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceView;
import android.view.SurfaceHolder;

import java.io.File;
import java.util.Timer;
import java.util.TimerTask;

import static com.bmhri.gvp.agvp.Running_state.INITING;
import static com.bmhri.gvp.agvp.Running_state.MOVING;
import static com.bmhri.gvp.agvp.Running_state.PLANNING;

/**
 * Created by shydh on 11/27/17.
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
    private int iw, ih;
    private int[] pix;
    private TaskLab mTaskLab;
    private TaskItem mTaskItem;
    private double m_x = 0.0, m_y = 0.0, m_angle = 0.0;

    int scalX, scalY;

    //init计数
    int count;
    //可以控制SurfaceView的大小，格式，可以监控或者改变SurfaceView
    SurfaceHolder mSurfaceHolder;
    //画布
    Canvas mCanvas;

    //创建画笔对象
    private Paint mPaint = new Paint();

    //子线程标志位，用来控制子线程
    boolean isDrawing;
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
        isDrawing = false;
    }

    @Override
    public void run() {
        Log.i(TAG, "running!!");
        while (isDrawing) {
            switch (state) {
                case INITING:
                    if (count == 1) {
                        getMap();
                        drawing();
                        Log.i(TAG, "initing!!");
                    }
                    count++;
                    state = PLANNING;
                    break;
                case PLANNING:
                    if (count == 2) {
                        Log.i(TAG, "planning!!");
                        initRobo();
                    }
                    count++;
                    state = MOVING;
                    break;
                case MOVING:
                    Log.i(TAG, "moving!!");
                    break;
                case STOPED:
                    Log.i(TAG, "stoped!!");
                    break;
                default:
                    break;
            }

            try {
                Thread.sleep(1000);//睡眠时间为1秒
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void init() {
        mSurfaceHolder = getHolder();//得到SurfaceHolder对象
        mSurfaceHolder.addCallback(this);//注册SurfaceHolder
        setFocusable(true);
        setFocusableInTouchMode(true);
        this.setKeepScreenOn(true);//保持屏幕长亮

        //画笔
        mPaint = new Paint(Paint.ANTI_ALIAS_FLAG | Paint.DITHER_FLAG);
        mPaint.setStrokeWidth(10f);
        mPaint.setColor(Color.parseColor("#FF4081"));
        mPaint.setStyle(Paint.Style.STROKE);
        mPaint.setStrokeJoin(Paint.Join.ROUND);
        mPaint.setStrokeCap(Paint.Cap.ROUND);

        count = 1;
        isDrawing = true;
        state = INITING;
    }

    public void drawing() {
        Log.i(TAG, "drawing!!");

        Bitmap srcBitmap =
                Bitmap.createBitmap(iw, ih, Bitmap.Config.ARGB_4444);
        srcBitmap.setPixels(pix, 0, iw, 0, 0, iw, ih);

        // 锁定画布
        mCanvas = mSurfaceHolder.lockCanvas();

        //这里进行内容的绘制
        mCanvas.save();
        Matrix matrix = new Matrix();

        int width = srcBitmap.getWidth();// 获取资源位图的宽
        int height = srcBitmap.getHeight();// 获取资源位图的高
        float w = (float) scalX / (float) width;
        float h = (float) scalY / (float) height;
        matrix.setScale(w, h);

        // 初始化画布
        mCanvas.drawColor(Color.TRANSPARENT);
        //绘制图形
        mCanvas.drawBitmap(srcBitmap, matrix, mPaint);
        mCanvas.restore();
        // 解锁画布
        getHolder().unlockCanvasAndPost(mCanvas);
    }

    private void startRobo() {

    }

    private void initRobo() {
        mTaskLab = TaskLab.get();
        mTaskItem = mTaskLab.getTask();
        Log.i(TAG, "Got a goal: " + mTaskItem.getId());

        double[] current = {0, 0, 0};
        double[] goal = {128, 128, 128};

        if(initMoveBase(current, goal))
            Log.i(TAG, "Move_base inited!! ");
        else
            Log.i(TAG, "Move_base failed initial!!  ");

        Log.i(TAG, "initRobo finished!");
    }

    private void getPose() {

    }

    private void getMap() {
        getSDPath();

        PGMLoader pgm = new PGMLoader();
        pgm.readPGMHeader(sdDir);
        iw = pgm.getWidth();
        ih = pgm.getHeight();

        pix = new int[iw * ih];
        loadIMG(sdDir, pix, iw, ih);
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

    public void start() {
        new Thread(this).start();
    }

    /**
     * A native method that is implemented by the 'native-lib' native library,
     * which is packaged with this application.
     */
    public native void loadIMG(String strDir, int[] src_image, int w, int h);

    public native void freeMoveBase();

    public native boolean initMoveBase(double[] current, double[] goal);
}
