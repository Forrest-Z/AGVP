#include <jni.h>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "move_base.h"

move_base::MoveBase *pmove_base = NULL;

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
                                           jintArray src_image_,
                                           jint w, jint h)
{
    // jstring 转 char*
    char *chardata = jstringTostring(env, strDir_);
    // char* 转 string
    std::string img_src = chardata;

    //test_cv testCv(img_src);
    int size = w * h;

    cv::Mat srcImage = cv::imread(img_src, 0);

    if (pmove_base == NULL)
        pmove_base = new move_base::MoveBase(srcImage);

    jint *temp = env->GetIntArrayElements(src_image_, 0);

    jint b;

    for (int i = 0; i < size; i++)
    {
        b = srcImage.data[i];
        if (b < 0)
            b = b + 256;
        temp[i] = (255 << 24) | (b << 16) | (b << 8) | b;
    }

    env->ReleaseIntArrayElements(src_image_, temp, JNI_OK);

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

    if (pmove_base)
    {
        pmove_base->initCb(tmp_c, tmp_g);
        return 1;
    } else
        return 0;
}