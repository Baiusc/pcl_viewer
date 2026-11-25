#ifndef YS_FLASH_COMMIT_H
#define YS_FLASH_COMMIT_H

#include <errno.h>
#include <spawn.h>
#include <stdlib.h>
#include <sys/wait.h>

extern char **environ;

/* Beta 启动器配置此入口。其他部署继续使用自己的持久目录。 */
static inline int ys_flash_commit(const char *kind, const char *candidate)
{
	const char *command = getenv("LIDAR_PERSIST_COMMAND");
	if (!kind || !command)
		return 0;
	if (*command != '/')
		return -1;
	char *const args[] = { (char *)command, "save", (char *)kind, (char *)candidate, NULL };
	pid_t worker;
	if (posix_spawn(&worker, command, NULL, NULL, args, environ))
		return -1;
	int status;
	pid_t result;
	do {
		result = waitpid(worker, &status, 0);
	} while (result < 0 && errno == EINTR);
	return result == worker && WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}

#endif
