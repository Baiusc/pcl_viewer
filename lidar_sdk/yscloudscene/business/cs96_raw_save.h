#ifndef YS_SCENE_CS96_RAW_SAVE_H_
#define YS_SCENE_CS96_RAW_SAVE_H_

#include <cstdint>
#include <string>

namespace ys_scene {

bool SaveCS96RawFrameSync(const uint8_t* data, size_t data_size,
    size_t frame_number, uint64_t ms_timestamp,
    const std::string& file_path);

} // namespace ys_scene

#endif // YS_SCENE_CS96_RAW_SAVE_H_
