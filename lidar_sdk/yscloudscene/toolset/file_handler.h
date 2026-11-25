#ifndef YS_SCENE_FILEHANDLER_H_
#define YS_SCENE_FILEHANDLER_H_

#include <qglobal.h>
#include <string>
#include <vector>
#include <fstream>

namespace ys_scene {
class FileHandler {
public:
	FileHandler(std::string &filepath);
	virtual ~FileHandler();
	bool Open();
	void Close();
	bool ReadData();
	std::vector<uint8_t>& GetData();

private:
	std::string filename_;
	std::ifstream filestream_;
	std::vector<uint8_t> data_;
};
} // namespace you_see

#endif // YS_SCENE_FILEHANDLER_H_
