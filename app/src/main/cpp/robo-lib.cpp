#include <jni.h>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <android/native_window_jni.h>
#include <android/bitmap.h>

#include "move_base.h"

move_base::MoveBase *pmove_base = NULL;

void map_show(JNIEnv *env, jobject surface,
              cv::Mat *input_map,
              int width, int heigth)
{
    ANativeWindow_Buffer nwBuffer;

    LOGI("ANativeWindow_fromSurface ");
    ANativeWindow *mANativeWindow = ANativeWindow_fromSurface(env, surface);

    if (mANativeWindow == NULL)
    {
        LOGI("ANativeWindow_fromSurface error");
        return;
    }

    LOGI("ANativeWindow_lock ");
    if (0 != ANativeWindow_lock(mANativeWindow, &nwBuffer, 0))
    {
        LOGI("ANativeWindow_lock error");
        return;
    }

    cv::Mat dst_map;
    cv::resize(*input_map, dst_map, cv::Size(width, heigth));

    __uint16_t *line = (__uint16_t *) nwBuffer.bits;

    u_char r, g, b;

    /* Process data */
    if (line != NULL)
    {
        for (int i = 0, choosehNum = 0; i < heigth; i++)
        {
            //获得一行
            uchar *modify = dst_map.ptr<uchar>(i);
            //根据缩放选取行
            if (choosehNum++ < nwBuffer.height)
            {
                //LOGI("nwBuffer->format == WINDOW_FORMAT_RGB_565");
                for (int j = 0, choosewNum = 0; j < width; j++)
                {
                    if (nwBuffer.format == WINDOW_FORMAT_RGB_565)
                    {
                        r = (u_char(modify[j])) >> 3;
                        g = (u_char(modify[j])) >> 2;
                        b = (u_char(modify[j])) >> 3;
                        line[choosewNum] = (r << 11) + (g << 5) + (b << 0);

                        choosewNum++;
                    }
                }
                line = line + nwBuffer.stride;
            }
        }
        /*memcpy(nwBuffer.bits, data, size);
        if (nwBuffer.width == nwBuffer.stride)
        {
            memcpy(nwBuffer.bits, data, size);
        } else
        {
            for (int i = 0; i < nwBuffer.height; i++)
            {
                u_char *srcPointer = data + nwBuffer.width * i * 2;
                u_char *dstPointer = ((u_char *) nwBuffer.bits) 
                                     + nwBuffer.stride * i * 2;

                memcpy(dstPointer, srcPointer, size * 2);
            }
        }*/
    }

    LOGI("ANativeWindow_unlockAndPost ");
    if (0 != ANativeWindow_unlockAndPost(mANativeWindow))
    {
        LOGI("ANativeWindow_unlockAndPost error");
        return;
    }

    ANativeWindow_release(mANativeWindow);
    LOGI("ANativeWindow_release ");

    return;
}

//jstring to char*
char *jstringTostring(JNIEnv *env, jstring jstr)
{
    char *rtn = NULL;
    jclass clsstring = env->FindClass("java/lang/String");
    jstring strencode = env->NewStringUTF("utf-8");
    jmethodID mid = env->GetMethodID(clsstring, "getBytes", "(Ljava/lang/String;)[B");
    jbyteArray barr = (jbyteArray) env->CallObjectMethod(jstr, mid, strencode);
    jsize alen = env->GetArrayLength(barr);
    jbyte *ba = env->GetByteArrayElements(barr, JNI_FALSE);
    if (alen > 0)
    {
        rtn = (char *) malloc((size_t) alen + 1);
        memcpy(rtn, ba, (size_t) alen);
        rtn[alen] = 0;
    }
    env->ReleaseByteArrayElements(barr, ba, 0);
    return rtn;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_bmhri_gvp_agvp_MapView_loadIMG(JNIEnv *env, jobject instance,
                                        jstring strDir_,
                                        jobject surface,
                                        jint w, jint h)
{
    // jstring 转 char*
    char *chardata = jstringTostring(env, strDir_);
    // char* 转 string
    std::string img_src = chardata;

    cv::Mat srcImage = cv::imread(img_src, 0);

    if (pmove_base == NULL)
        pmove_base = new move_base::MoveBase(srcImage);

    map_show(env, surface, &srcImage, w, h);

    return;
}

extern "C"
JNIEXPORT void JNICALL
Java_com_bmhri_gvp_agvp_MapView_freeMoveBase(JNIEnv *env, jobject instance)
{
    // TODO
    if (pmove_base != NULL)
    {
        delete pmove_base;
        pmove_base = NULL;
    }
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_bmhri_gvp_agvp_MapView_initMoveBase(JNIEnv *env,
                                             jobject /* this */,
                                             jdoubleArray current,
                                             jdoubleArray goal)
{
    jdouble *tmp_c = env->GetDoubleArrayElements(current, 0);
    jdouble *tmp_g = env->GetDoubleArrayElements(goal, 0);

    if (pmove_base != NULL)
    {
        pmove_base->initCb(tmp_c, tmp_g);
        return 1;
    } else
        return 0;
}

extern "C"
JNIEXPORT jboolean JNICALL
Java_com_bmhri_gvp_agvp_MapView_planMoveBase(JNIEnv *env, jobject instance)
{
    if (pmove_base != NULL)
    {
        if (pmove_base->planCb())
            return 1;
        else
            return 0;
    } else
        return 0;

}

extern "C"
JNIEXPORT void JNICALL
Java_com_bmhri_gvp_agvp_MapView_loadPath(JNIEnv *env, jobject instance,
                                         jobject surface,
                                         jint w, jint h)
{
    std::vector<std::pair<u_int, u_int>> path;

    if (pmove_base != NULL)
    {
        path = pmove_base->get_plan();
    } else
        return;

    cv::Mat dst_map;

    dst_map = pmove_base->src_map_.clone();

    for(u_long i = 0; i < path.size(); i++)
    {
        uchar *modify = dst_map.ptr<uchar>(path[i].first);
        modify[path[i].second] = 1;
    }

    map_show(env, surface, &dst_map, w, h);

    return;
}