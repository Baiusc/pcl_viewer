#ifndef YS_USER_SETTINGS_H
#define YS_USER_SETTINGS_H

/* 只保存用户参数，不包含工作模式、握手目标、身份或系统网络配置。 */
typedef struct {
	int echo_mode;
	int echo_type;
	int coordinate_mode;
} ys_user_settings_t;

/* 返回 0 表示读取成功，1 表示首次使用尚无文件，-1 表示文件损坏或不可访问。 */
int ys_user_settings_load(const char *path, ys_user_settings_t *settings);
int ys_user_settings_store(const char *path, const ys_user_settings_t *settings);
#endif
