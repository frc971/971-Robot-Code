#include "yolov5.h"

#include <tensorflow/lite/c/common.h>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tflite/public/edgetpu.h>
#include <tflite/public/edgetpu_c.h>

#include <opencv2/dnn.hpp>

#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_double(conf_threshold, 0.9,
              "Threshold value for confidence scores. Detections with a "
              "confidence score below this value will be ignored.");

DEFINE_double(
    nms_threshold, 0.5,
    "Threshold value for non-maximum suppression. Detections with an "
    "intersection-over-union value below this value will be removed.");

DEFINE_int32(nthreads, 6, "Number of threads to use during inference.");

namespace y2023 {
namespace vision {

class YOLOV5Impl : public YOLOV5 {
 public:
  // Takes a model path as string and and loads a pre-trained
  // YOLOv5 model from the specified path.
  void LoadModel(const std::string path);

  // Takes an image and returns a Detection.
  std::vector<Detection> ProcessImage(cv::Mat image);

 private:
  // Convert an OpenCV Mat object to a tensor input
  // that can be fed to the TensorFlow Lite model.
  void ConvertCVMatToTensor(cv::Mat src, uint8_t *in);

  // Resizes, converts color space, and converts
  // image data type before inference.
  void Preprocess(cv::Mat image);

  // Converts a TensorFlow Lite tensor to a 2D vector.
  std::vector<std::vector<float>> TensorToVector2D(TfLiteTensor *src_tensor,
                                                   const int rows,
                                                   const int columns);

  // Performs non-maximum suppression to remove overlapping bounding boxes.
  void NonMaximumSupression(const std::vector<std::vector<float>> &orig_preds,
                            const int rows, const int columns,
                            std::vector<Detection> *detections,
                            std::vector<int> *indices);
  // Models
  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;
  tflite::StderrReporter error_reporter_;

  // Parameters of interpreter's input
  int input_;
  int in_height_;
  int in_width_;
  int in_channels_;
  int in_type_;

  // Parameters of original image
  int img_height_;
  int img_width_;

  // Input of the interpreter
  uint8_t *input_8_;

  // Subtract this offset from class labels to get the actual label.
  static constexpr int kClassIdOffset = 5;
};

std::unique_ptr<YOLOV5> MakeYOLOV5() { return std::make_unique<YOLOV5Impl>(); }

void YOLOV5Impl::LoadModel(const std::string path) {
  LOG(INFO) << "Load model: start";


  tflite::ops::builtin::BuiltinOpResolver resolver;

  model_ = tflite::FlatBufferModel::VerifyAndBuildFromFile(path.c_str());

  /*
  auto model_impl = model_->GetModel();
  model_impl->subgraphs();
  LOG(INFO) << model_impl;
  LOG(INFO) << model_impl->subgraphs();
  auto subgraphs = model_impl->subgraphs();
  LOG(INFO) << subgraphs->size();
  LOG(INFO) << subgraphs->Get(0)->inputs()->size();
  LOG(INFO) << subgraphs->Get(0)->inputs()->Get(0);
  (void)subgraphs;
  */

  LOG(INFO) << "Load model: Build Model from file";

  CHECK(model_);
  CHECK(model_->initialized());
  CHECK_EQ(tflite::InterpreterBuilder(*model_, resolver)(&interpreter_),
           kTfLiteOk);
  LOG(INFO) << "Load model: Interpreter builder done";
  /*
  LOG(INFO) << &interpreter_->primary_subgraph();
  LOG(INFO) << interpreter_->subgraph(0);
  LOG(INFO) << interpreter_->subgraphs_size();
  LOG(INFO) << interpreter_->subgraph(0)->inputs().size();
  LOG(INFO) << interpreter_->inputs().size();
  */

  //interpreter_->SetExternalContext(kTfLiteEdgeTpuContext, edgetpu_context.get());
  // LOG(INFO) << "After set external context";

  size_t num_devices;
  std::unique_ptr<edgetpu_device, decltype(&edgetpu_free_devices)> devices(
      edgetpu_list_devices(&num_devices), &edgetpu_free_devices);

  //const auto &available_tpus =
    // edgetpu::EdgeTpuManager::GetSingleton()->EnumerateEdgeTpu();
  //LOG(INFO) << "Available tpus: " << available_tpus.size();

  LOG(INFO) << "Load model: Getting devices";
  CHECK_EQ(num_devices, 1ul);
  const auto &device = devices.get()[0];
  (void )device;
  LOG(INFO) << "Load model: Got Device";

  auto *delegate = edgetpu_create_delegate(device.type, device.path, nullptr, 0);

  interpreter_->ModifyGraphWithDelegate(delegate);


  TfLiteStatus status = interpreter_->AllocateTensors();
  CHECK_EQ(status, kTfLiteOk);
  CHECK(interpreter_);

  LOG(INFO) << "Load model: Allocate tensors success";
  input_ = interpreter_->inputs()[0];
  LOG(INFO) << "After set inputs";
  LOG(INFO) << input_;
  TfLiteIntArray *dims = interpreter_->tensor(input_)->dims;
  in_height_ = dims->data[1];
  in_width_ = dims->data[2];
  in_channels_ = dims->data[3];
  in_type_ = interpreter_->tensor(input_)->type;
  input_8_ = interpreter_->typed_tensor<uint8_t>(input_);
  

  interpreter_->SetNumThreads(FLAGS_nthreads);

  LOG(INFO) << "End of load";
}

void YOLOV5Impl::ConvertCVMatToTensor(cv::Mat src, uint8_t *in) {
  CHECK(src.type() == CV_8UC3);
  int n = 0, nc = src.channels(), ne = src.elemSize();
  LOG(INFO) << "ConvertCVMatToTensor - Rows " << src.rows;
  LOG(INFO) << "ConvertCVMatToTensor - Cols " << src.cols;
  for (int y = 0; y < src.rows; ++y) {
    for (int x = 0; x < src.cols; ++x) {
      for (int c = 0; c < nc; ++c) {
	      (void)ne;
	      (void)n;
        in[n++] = src.data[y * src.step + x * ne + c];
      }
    }
  }
}

std::vector<std::vector<float>> YOLOV5Impl::TensorToVector2D(
    TfLiteTensor *src_tensor, const int rows, const int columns) {
  auto scale = src_tensor->params.scale;
  auto zero_point = src_tensor->params.zero_point;
  std::vector<std::vector<float>> result_vec;
  for (int32_t i = 0; i < rows; i++) {
    std::vector<float> row_values;
    for (int32_t j = 0; j < columns; j++) {
      float val_float =
          ((static_cast<int32_t>(src_tensor->data.uint8[i * columns + j])) -
           zero_point) *
          scale;
      row_values.push_back(val_float);
    }
    result_vec.push_back(row_values);
  }
  return result_vec;
}

void YOLOV5Impl::NonMaximumSupression(
    const std::vector<std::vector<float>> &orig_preds, const int rows,
    const int columns, std::vector<Detection> *detections,
    std::vector<int> *indices)

{
  std::vector<float> scores;
  double confidence;
  cv::Point class_id;

  for (int i = 0; i < rows; i++) {
    if (orig_preds[i][4] > FLAGS_conf_threshold) {
      int left = (orig_preds[i][0] - orig_preds[i][2] / 2) * img_width_;
      int top = (orig_preds[i][1] - orig_preds[i][3] / 2) * img_height_;
      int w = orig_preds[i][2] * img_width_;
      int h = orig_preds[i][3] * img_height_;

      for (int j = 5; j < columns; j++) {
        scores.push_back(orig_preds[i][j] * orig_preds[i][4]);
      }

      cv::minMaxLoc(scores, nullptr, &confidence, nullptr, &class_id);
      if (confidence > FLAGS_conf_threshold) {
        Detection detection{cv::Rect(left, top, w, h), confidence,
                            class_id.x - kClassIdOffset};
        detections->push_back(detection);
      }
    }
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;

  for (const Detection &d : *detections) {
    boxes.push_back(d.box);
    confidences.push_back(d.confidence);
  }

  (void)indices;
  // TODO(FILIP): Fix linker error.
  // cv::dnn::NMSBoxes(boxes, confidences, FLAGS_conf_threshold,
                  // FLAGS_nms_threshold, *indices);
}

std::vector<Detection> YOLOV5Impl::ProcessImage(cv::Mat frame) {
  img_height_ = frame.rows;
  img_width_ = frame.cols;

  //Preprocess;
  cv::resize(frame, frame, cv::Size(in_height_, in_width_), cv::INTER_CUBIC);
  cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
  frame.convertTo(frame, CV_8U);

  LOG(INFO) << "After preprocess - Before convert to tensor";
  ConvertCVMatToTensor(frame, input_8_);

  // Inference
  LOG(INFO) << "Before Invoke";
  TfLiteStatus status = interpreter_->Invoke();
  CHECK_EQ(status, kTfLiteOk);

  LOG(INFO) << "After invoke, status checked";

  int output_tensor_index = interpreter_->outputs()[0];
  TfLiteIntArray *out_dims = interpreter_->tensor(output_tensor_index)->dims;
  int num_rows = out_dims->data[1];
  int num_columns = out_dims->data[2];

  TfLiteTensor *src_tensor = interpreter_->tensor(interpreter_->outputs()[0]);

  std::vector<std::vector<float>> orig_preds =
      TensorToVector2D(src_tensor, num_rows, num_columns);
  LOG(INFO) << "After tensor to vector 2D";

  std::vector<int> indices;
  std::vector<Detection> detections;

  NonMaximumSupression(orig_preds, num_rows, num_columns, &detections,
                       &indices);
  LOG(INFO) << "After NMS";
  for (size_t i = 0; i < interpreter_->outputs().size(); i++) {
        LOG(INFO) << "Detection #" << i << " | " << interpreter_->outputs()[i];
  }
  if (detections.size() > 0) {
    LOG(INFO) << "Detection ID: " <<  detections[0].class_id;
    LOG(INFO) << "Confidence" << detections[0].confidence;
  }
  return detections;
};

}  // namespace vision
}  // namespace y2023
