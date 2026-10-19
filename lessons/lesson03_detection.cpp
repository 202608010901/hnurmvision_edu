#include "lessons.hpp"

namespace training {
// 第三讲：装甲板检测
// 输入：不可修改的 CV_8UC3 BGR 图像。
// 输出：Armor 列表；角点顺序为左上、右上、右下、左下，单位为像素。
// 图像坐标原点在左上；无法识别编号时 class_id=-1；没有装甲板时返回空列表。
std::vector<Armor> lesson03_detection(const cv::Mat& image) {
  // TODO(第三讲)：实现装甲板检测，填写角点、大小板、编号与置信度。
  (void)image;
  return {};
}
}  // namespace training
