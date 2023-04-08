#include "yolov5.h"

#include <opencv2/core.hpp>

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

void YOLOV5::LoadModel(const std::string path) {
  model_ = tflite::FlatBufferModel::BuildFromFile(path.c_str());
  CHECK(model_);
  size_t num_devices;
  std::unique_ptr<edgetpu_device, decltype(&edgetpu_free_devices)> devices(
      edgetpu_list_devices(&num_devices), &edgetpu_free_devices);
  const auto &device = devices.get()[0];
  CHECK_EQ(num_devices, 1ul);
  tflite::ops::builtin::BuiltinOpResolver resolver;
  CHECK_EQ(tflite::InterpreterBuilder(*model_, resolver)(&interpreter_),
           kTfLiteOk);

  auto *delegate =
      edgetpu_create_delegate(device.type, device.path, nullptr, 0);
  interpreter_->ModifyGraphWithDelegate(delegate);

  TfLiteStatus status = interpreter_->AllocateTensors();
  CHECK(status == kTfLiteOk);

  input_ = interpreter_->inputs()[0];
  TfLiteIntArray *dims = interpreter_->tensor(input_)->dims;
  in_height_ = dims->data[1];
  in_width_ = dims->data[2];
  in_channels_ = dims->data[3];
  in_type_ = interpreter_->tensor(input_)->type;
  input_8_ = interpreter_->typed_tensor<uint8_t>(input_);

  interpreter_->SetNumThreads(FLAGS_nthreads);
}

void YOLOV5::Preprocess(cv::Mat image) {
  cv::resize(image, image, cv::Size(in_height_, in_width_), cv::INTER_CUBIC);
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  image.convertTo(image, CV_8U);
}

void YOLOV5::ConvertCVMatToTensor(const cv::Mat &src, uint8_t *in) {
  CHECK(src.type() == CV_8UC3);
  int n = 0, nc = src.channels(), ne = src.elemSize();
  for (int y = 0; y < src.rows; ++y)
    for (int x = 0; x < src.cols; ++x)
      for (int c = 0; c < nc; ++c)
        in[n++] = src.data[y * src.step + x * ne + c];
}

std::vector<std::vector<float>> YOLOV5::TensorToVector2D(
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

void YOLOV5::NonMaximumSupression(
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

  cv::dnn::NMSBoxes(boxes, confidences, FLAGS_conf_threshold,
                    FLAGS_nms_threshold, *indices);
}

std::vector<Detection> YOLOV5::ProcessImage(cv::Mat frame) {
  img_height_ = frame.rows;
  img_width_ = frame.cols;

  Preprocess(frame);
  ConvertCVMatToTensor(frame, input_8_);

  // Inference
  TfLiteStatus status = interpreter_->Invoke();
  CHECK_EQ(status, kTfLiteOk);

  int output_tensor_index = interpreter_->outputs()[0];
  TfLiteIntArray *out_dims = interpreter_->tensor(output_tensor_index)->dims;
  int num_rows = out_dims->data[1];
  int num_columns = out_dims->data[2];

  TfLiteTensor *src_tensor = interpreter_->tensor(interpreter_->outputs()[0]);
  std::vector<std::vector<float>> orig_preds =
      TensorToVector2D(src_tensor, num_rows, num_columns);

  std::vector<int> indices;
  std::vector<Detection> detections;

  NonMaximumSupression(orig_preds, num_rows, num_columns, &detections,
                       &indices);

  return detections;
};

}  // namespace vision
}  // namespace y2023
