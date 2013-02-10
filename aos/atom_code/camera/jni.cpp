#include <setjmp.h>

#include "jni/aos_Natives.h"
#include "aos/atom_code/camera/Buffers.h"
#include "aos/externals/libjpeg/include/jpeglib.h"

using aos::camera::Buffers;

namespace {

jclass nativeError, bufferError, outOfMemoryError;
bool findClass(JNIEnv *env, const char *name, jclass *out) {
  jclass local = env->FindClass(name);
  if (out == NULL) return true;
  *out = static_cast<jclass>(env->NewGlobalRef(local));
  if (out == NULL) return true;
  env->DeleteLocalRef(local);
  return false;
}

// Checks that the size is correct and retrieves the address.
// An expected_size of 0 means don't check it.
// If this function returns NULL, a java exception will already have been
// thrown.
void *getBufferAddress(JNIEnv *env, jobject obj, jlong expected_size) {
  if (obj == NULL) {
    env->ThrowNew(nativeError, "null buffer");
    return NULL;
  }
  if (expected_size != 0 &&
      expected_size != env->GetDirectBufferCapacity(obj)) {
    char *str;
    if (asprintf(&str, "wrong size. expected %lld but got %lld",
                 expected_size, env->GetDirectBufferCapacity(obj)) < 0) {
      env->ThrowNew(bufferError, "creating message failed");
      return NULL;
    }
    env->ThrowNew(bufferError, str);
    free(str);
    return NULL;
  }
  void *const r = env->GetDirectBufferAddress(obj);
  if (r == NULL) {
    env->ThrowNew(bufferError, "couldn't get address");
  }
  return r;
}

const int kImagePixels = Buffers::kWidth * Buffers::kHeight;

void jpeg_log_message(jpeg_common_struct *cinfo, log_level level) {
  char buf[LOG_MESSAGE_LEN];
  cinfo->err->format_message(cinfo, buf);
  log_do(level, "libjpeg: %s\n", buf);
}
void jpeg_error_exit(jpeg_common_struct *cinfo) __attribute__((noreturn));
void jpeg_error_exit(jpeg_common_struct *cinfo) {
  jpeg_log_message(cinfo, ERROR);
  longjmp(*static_cast<jmp_buf *>(cinfo->client_data), 1);
}
void jpeg_emit_message(jpeg_common_struct *cinfo, int msg_level) {
  if (msg_level < 0) {
    jpeg_log_message(cinfo, WARNING);
    longjmp(*static_cast<jmp_buf *>(cinfo->client_data), 2);
  }
  // this spews a lot of messages out
  //jpeg_log_message(cinfo, DEBUG);
}

// The structure used to hold all of the state for the functions that deal with
// a Buffers. A pointer to this structure is stored java-side.
struct BuffersHolder {
  Buffers buffers;
  timeval timestamp;
  BuffersHolder() : buffers() {}
};

} // namespace

void Java_aos_Natives_nativeInit(JNIEnv *env, jclass, jint width, jint height) {
  if (findClass(env, "aos/NativeError", &nativeError)) return;
  if (findClass(env, "aos/NativeBufferError", &bufferError)) return;
  if (findClass(env, "java/lang/OutOfMemoryError", &outOfMemoryError)) return;

  aos::InitNRT();

  if (width != Buffers::kWidth || height != Buffers::kHeight) {
    env->ThrowNew(nativeError, "dimensions mismatch");
    return;
  }

  LOG(INFO, "nativeInit finished\n");
}

static_assert(sizeof(jlong) >= sizeof(void *),
              "can't stick pointers into jlongs");

jboolean Java_aos_Natives_decodeJPEG(JNIEnv *env, jclass, jlongArray stateArray,
                                     jobject inobj, jint inLength,
                                     jobject outobj) {
  unsigned char *const in = static_cast<unsigned char *>(
      getBufferAddress(env, inobj, 0));
  if (in == NULL) return false;
  if (env->GetDirectBufferCapacity(inobj) < inLength) {
    env->ThrowNew(bufferError, "in is too small");
    return false;
  }
  unsigned char *const out = static_cast<unsigned char *>(
      getBufferAddress(env, outobj, kImagePixels * 3));
  if (out == NULL) return false;

  jpeg_decompress_struct *volatile cinfo; // volatile because of the setjmp call

  jlong state;
  env->GetLongArrayRegion(stateArray, 0, 1, &state);
  if (env->ExceptionCheck()) return false;
  if (state == 0) {
    cinfo = static_cast<jpeg_decompress_struct *>(malloc(sizeof(*cinfo)));
    if (cinfo == NULL) {
      env->ThrowNew(outOfMemoryError, "malloc for jpeg_decompress_struct");
      return false;
    }
    cinfo->err = jpeg_std_error(static_cast<jpeg_error_mgr *>(
            malloc(sizeof(*cinfo->err))));
    cinfo->client_data = malloc(sizeof(jmp_buf));
    cinfo->err->error_exit = jpeg_error_exit;
    cinfo->err->emit_message = jpeg_emit_message;
    // if the error handler sees a failure, it needs to clean up
    // (jpeg_abort_decompress) and then return the failure
    // set cinfo->client_data to the jmp_buf
    jpeg_create_decompress(cinfo);
    state = reinterpret_cast<intptr_t>(cinfo);
    env->SetLongArrayRegion(stateArray, 0, 1, &state);
    if (env->ExceptionCheck()) return false;
  } else {
    cinfo = reinterpret_cast<jpeg_decompress_struct *>(state);
  }

  // set up the jump buffer
  // this has to happen each time
  if (setjmp(*static_cast<jmp_buf *>(cinfo->client_data))) {
    jpeg_abort_decompress(cinfo);
    return false;
  }

  jpeg_mem_src(cinfo, in, inLength);
  jpeg_read_header(cinfo, TRUE);
  if (cinfo->image_width != static_cast<unsigned int>(Buffers::kWidth) ||
      cinfo->image_height != static_cast<unsigned int>(Buffers::kHeight)) {
    LOG(WARNING, "got (%ux%u) image but expected (%dx%d)\n", cinfo->image_width,
        cinfo->image_height, Buffers::kWidth, Buffers::kHeight);
    jpeg_abort_decompress(cinfo);
    return false;
  }
  cinfo->out_color_space = JCS_RGB;
  jpeg_start_decompress(cinfo);
  if (cinfo->output_components != 3) {
    LOG(WARNING, "libjpeg wants to return %d color components instead of 3\n",
        cinfo->out_color_components);
    jpeg_abort_decompress(cinfo);
    return false;
  }
  if (cinfo->output_width != static_cast<unsigned int>(Buffers::kWidth) ||
      cinfo->output_height != static_cast<unsigned int>(Buffers::kHeight)) {
    LOG(WARNING, "libjpeg wants to return a (%ux%u) image but need (%dx%d)\n",
        cinfo->output_width, cinfo->output_height,
        Buffers::kWidth, Buffers::kHeight);
    jpeg_abort_decompress(cinfo);
    return false;
  }

  unsigned char *buffers[Buffers::kHeight];
  for (int i = 0; i < Buffers::kHeight; ++i) {
    buffers[i] = &out[i * Buffers::kWidth * 3];
  }
  while (cinfo->output_scanline < cinfo->output_height) {
    jpeg_read_scanlines(cinfo, &buffers[cinfo->output_scanline],
                        Buffers::kHeight - cinfo->output_scanline);
  }

  jpeg_finish_decompress(cinfo);
  return true;
}

void Java_aos_Natives_threshold(JNIEnv *env, jclass, jobject inobj,
                                jobject outobj, jshort hoffset, jchar hmin,
                                jchar hmax, jchar smin, jchar smax, jchar vmin,
                                jchar vmax) {
  const unsigned char *__restrict__ const in = static_cast<unsigned char *>(
      getBufferAddress(env, inobj, kImagePixels * 3));
  if (in == NULL) return;
  char *__restrict__ const out = static_cast<char *>(
      getBufferAddress(env, outobj, kImagePixels));
  if (out == NULL) return;

  for (int i = 0; i < kImagePixels; ++i) {
    const uint8_t h = in[i * 3] + static_cast<uint8_t>(hoffset);
    out[i] = h > hmin && h < hmax &&
        in[i * 3 + 1] > smin && in[i * 3 + 1] < smax &&
        in[i * 3 + 2] > vmin && in[i * 3 + 2] < vmax;
  }
}
void Java_aos_Natives_convertBGR2BMP(JNIEnv *env, jclass,
                                     jobject inobj, jobject outobj) {
  const char *__restrict__ const in = static_cast<char *>(
      getBufferAddress(env, inobj, kImagePixels * 3));
  if (in == NULL) return;
  char *__restrict__ const out = static_cast<char *>(
      getBufferAddress(env, outobj, kImagePixels * 3));
  if (out == NULL) return;

  for (int i = 0; i < kImagePixels; ++i) {
    out[i * 3 + 0] = in[i * 3 + 2];
    out[i * 3 + 1] = in[i * 3 + 1];
    out[i * 3 + 2] = in[i * 3 + 0];
  }
}

jlong Java_aos_Natives_queueInit(JNIEnv *, jclass) {
  return reinterpret_cast<intptr_t>(new BuffersHolder());
}
void Java_aos_Natives_queueReleaseJPEG(JNIEnv *, jclass, jlong ptr) {
  reinterpret_cast<BuffersHolder *>(ptr)->buffers.Release();
}
jobject Java_aos_Natives_queueGetJPEG(JNIEnv *env, jclass, jlong ptr) {
  uint32_t size;
  BuffersHolder *const holder = reinterpret_cast<BuffersHolder *>(ptr);
  const void *const r = holder->buffers.GetNext(true, &size,
                                                &holder->timestamp, NULL);
  if (r == NULL) return NULL;
  return env->NewDirectByteBuffer(const_cast<void *>(r), size);
}
jdouble Java_aos_Natives_queueGetTimestamp(JNIEnv *, jclass, jlong ptr) {
  const BuffersHolder *const holder = reinterpret_cast<BuffersHolder *>(ptr);
  return holder->timestamp.tv_sec + holder->timestamp.tv_usec / 1000000.0;
}

void Java_aos_Natives_LOG(JNIEnv *env, jclass, jstring message, jint jlevel) {
  log_level level;
  if (jlevel >= 1000) {
    // Don't want to use FATAL because the uncaught java exception that is
    // likely to come next will be useful.
    level = ERROR;
  } else if (jlevel >= 900) {
    level = WARNING;
  } else if (jlevel >= 800) {
    level = INFO;
  } else {
    level = DEBUG;
  }
  // can't use Get/ReleaseStringCritical because log_do might block waiting to
  // put its message into the queue
  const char *const message_chars = env->GetStringUTFChars(message, NULL);
  if (message_chars == NULL) return;
  log_do(level, "%s\n", message_chars);
  env->ReleaseStringUTFChars(message, message_chars);
}
