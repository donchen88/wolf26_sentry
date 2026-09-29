#include "rm_behavior_tree/key_point_loader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <regex>

namespace rm_behavior_tree
{

bool KeyPointLoader::loadFromFile(const std::string & yaml_path)
{
  keypoints_.clear();
  
  std::ifstream file(yaml_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open YAML file: " << yaml_path << std::endl;
    return false;
  }

  std::string line;
  KeyPoint current_point;
  bool in_keypoints_section = false;
  bool reading_point = false;
  
  while (std::getline(file, line)) {
    // 去除行首尾空白（保留原始行用于调试）
    std::string trimmed_line = line;
    trimmed_line.erase(0, trimmed_line.find_first_not_of(" \t"));
    trimmed_line.erase(trimmed_line.find_last_not_of(" \t") + 1);
    
    // 跳过空行
    if (trimmed_line.empty()) {
      continue;
    }
    
    // 检查是否进入 keypoints 部分
    if (trimmed_line == "keypoints:" || trimmed_line.find("keypoints:") == 0) {
      in_keypoints_section = true;
      continue;
    }
    
    if (!in_keypoints_section) {
      continue;
    }
    
    // 检查是否是新的关键点开始（包含 "- id:"）
    size_t id_pos = trimmed_line.find("- id:");
    if (id_pos != std::string::npos) {
      // 如果之前有点未完成，先保存
      if (reading_point) {
        keypoints_.push_back(current_point);
      }
      
      // 开始读取新点
      reading_point = true;
      current_point = KeyPoint();
      current_point.description = "";
      
      // 解析 id（从 "- id:" 之后提取数字）
      std::regex id_regex(R"(-\s*id:\s*(\d+))");
      std::smatch id_match;
      if (std::regex_search(trimmed_line, id_match, id_regex)) {
        current_point.id = std::stoi(id_match[1].str());
      }
      
      // 检查是否有注释
      size_t comment_pos = trimmed_line.find('#');
      if (comment_pos != std::string::npos) {
        current_point.description = trimmed_line.substr(comment_pos + 1);
        // 去除注释前后空白
        current_point.description.erase(0, current_point.description.find_first_not_of(" \t"));
        current_point.description.erase(current_point.description.find_last_not_of(" \t") + 1);
      }
      continue;
    }
    
    // 解析 x 坐标（查找 "x:" 关键字，可能在行首或行中）
    size_t x_pos = trimmed_line.find("x:");
    if (x_pos != std::string::npos) {
      std::regex x_regex(R"(x:\s*([+-]?\d+\.?\d*))");
      std::smatch x_match;
      if (std::regex_search(trimmed_line, x_match, x_regex)) {
        current_point.x = std::stod(x_match[1].str());
      }
      continue;
    }
    
    // 解析 y 坐标
    size_t y_pos = trimmed_line.find("y:");
    if (y_pos != std::string::npos) {
      std::regex y_regex(R"(y:\s*([+-]?\d+\.?\d*))");
      std::smatch y_match;
      if (std::regex_search(trimmed_line, y_match, y_regex)) {
        current_point.y = std::stod(y_match[1].str());
      }
      continue;
    }
  }
  
  // 保存最后一个点
  if (reading_point) {
    keypoints_.push_back(current_point);
  }
  
  file.close();
  
  std::cout << "Loaded " << keypoints_.size() << " keypoints from " << yaml_path << std::endl;
  return true;
}

const KeyPoint * KeyPointLoader::getKeyPointById(int id) const
{
  for (const auto & point : keypoints_) {
    if (point.id == id) {
      return &point;
    }
  }
  return nullptr;
}

}  // namespace rm_behavior_tree

