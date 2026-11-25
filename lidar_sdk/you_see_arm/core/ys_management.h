#ifndef YS_MANAGEMENT_H
#define YS_MANAGEMENT_H
int ys_management_init(void);
int ys_management_enabled(void);
int ys_shutdown_requested(void);
void ys_management_initialized(int work_mode);
void ys_management_progress(int work_mode, int valid);
void ys_management_close(void);
#endif
