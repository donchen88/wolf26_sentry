#include "rm_behavior_tree/key_point_loader.hpp"
#include <iostream>

int main(int argc, char ** argv)
{
  rm_behavior_tree::KeyPointLoader loader;
  
  // 使用相对路径或绝对路径
  std::string yaml_path = "src/rm_behavior_tree/rm_behavior_tree/config/key_point.yaml";
  if (argc > 1) {
    yaml_path = argv[1];
  }
  
  if (!loader.loadFromFile(yaml_path)) {
    std::cerr << "Failed to load keypoints from " << yaml_path << std::endl;
    return 1;
  }
  
  std::cout << "\n=== Loaded KeyPoints ===" << std::endl;
  for (const auto & point : loader.getKeyPoints()) {
    std::cout << "ID: " << point.id 
              << ", X: " << point.x 
              << ", Y: " << point.y;
    if (!point.description.empty()) {
      std::cout << ", Description: " << point.description;
    }
    std::cout << std::endl;
  }
  
  std::cout << "\nTotal keypoints: " << loader.size() << std::endl;
  
  // 测试根据ID查找
  std::cout << "\n=== Test getKeyPointById ===" << std::endl;
  for (int id = 1; id <= 5; ++id) {
    const auto * point = loader.getKeyPointById(id);
    if (point) {
      std::cout << "Found point " << id << ": (" << point->x << ", " << point->y << ")" << std::endl;
    } else {
      std::cout << "Point " << id << " not found" << std::endl;
    }
  }
  
  return 0;
}

