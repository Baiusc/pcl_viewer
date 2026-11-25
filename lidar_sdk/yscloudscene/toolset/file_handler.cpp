#include "file_handler.h"
#include <iostream>

namespace ys_scene {
FileHandler::FileHandler(std::string &filepath) : 
	filename_(filepath)
{

}

FileHandler::~FileHandler()
{
}

bool FileHandler::Open()
{
	filestream_.open(filename_, std::ios::binary);
	return filestream_.is_open();
}

void FileHandler::Close()
{
	if (filestream_.is_open()) {
		filestream_.close();
	}
}

bool FileHandler::ReadData()
{
	if (!filestream_.is_open()) {
		std::cerr << "File is not open" << std::endl;
		return false;
	}
	// get file size
	filestream_.seekg(0, std::ios::end);
	std::streampos filesize = filestream_.tellg();
	filestream_.seekg(0, std::ios::beg);
	
	// create data and read data
	data_.resize(filesize);
	filestream_.read(reinterpret_cast<char*>(data_.data()), filesize);

	return !filestream_.fail();
}

std::vector<uint8_t>& FileHandler::GetData()
{
	return data_;
}

} // namespace you_see

