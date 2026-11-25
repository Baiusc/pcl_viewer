#include <vector>
#include <memory>
#include "ys_lidar_type.h"

using namespace you_see;
#define IMPORT_DLL extern "C" _declspec(dllimport)//指定允许给其他外部调用
IMPORT_DLL void AlgoCal(std::shared_ptr<std::vector<CartesianPoint>> & points);
