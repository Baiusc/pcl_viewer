#define _GNU_SOURCE
#include "mgr_support.h"
#include "../core/ys_flash_commit.h"
#include "../core/ys_user_settings.h"
#include "../third_party/iniparser/iniparser.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

int64_t mgr_now(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
const char *mgr_string(cJSON *o, const char *name)
{
	cJSON *v = cJSON_GetObjectItemCaseSensitive(o, name);
	return cJSON_IsString(v) ? v->valuestring : NULL;
}
int mgr_int(cJSON *o, const char *name, int fallback)
{
	cJSON *v = cJSON_GetObjectItemCaseSensitive(o, name);
	return cJSON_IsNumber(v) && v->valuedouble == v->valueint ? v->valueint : fallback;
}
cJSON *mgr_read_json(const char *path)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
	struct stat st;
	if (fd < 0)
		return NULL;
	if (fstat(fd, &st) || !S_ISREG(st.st_mode) || st.st_size < 2 || st.st_size > 65536) {
		close(fd);
		return NULL;
	}
	char *text = calloc(1, (size_t)st.st_size + 1);
	if (!text) {
		close(fd);
		return NULL;
	}
	size_t got = 0;
	while (got < (size_t)st.st_size) {
		ssize_t n = read(fd, text + got, (size_t)st.st_size - got);
		if (n <= 0) {
			free(text);
			close(fd);
			return NULL;
		}
		got += (size_t)n;
	}
	close(fd);
	cJSON *result = cJSON_ParseWithOpts(text, NULL, 1);
	free(text);
	return result;
}
static int finish_file(FILE *file, int dirfd, const char *tmp, const char *name, const char *flash_kind,
		       const char *candidate)
{
	int bad = fflush(file) || ferror(file) || fsync(fileno(file));
	if (fclose(file))
		bad = 1;
	if (!bad && ys_flash_commit(flash_kind, candidate))
		bad = 1;
	if (!bad && renameat(dirfd, tmp, dirfd, name))
		bad = 1;
	if (!bad && fsync(dirfd))
		bad = 1;
	if (bad)
		unlinkat(dirfd, tmp, 0);
	return bad ? -1 : 0;
}
int mgr_write_json(const char *directory, const char *name, cJSON *value, const char *flash_kind)
{
	char candidate[PATH_MAX];
	if (snprintf(candidate, sizeof(candidate), "%s/.state.tmp", directory) >= (int)sizeof(candidate))
		return -1;
	int dirfd = open(directory, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	if (dirfd < 0)
		return -1;
	int fd = openat(dirfd, ".state.tmp", O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
	/* 上次断电遗留的临时文件不作为有效状态。目录由管理服务独占。 */
	if (fd < 0 && errno == EEXIST) {
		unlinkat(dirfd, ".state.tmp", 0);
		fd = openat(dirfd, ".state.tmp", O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
	}
	FILE *file = fd >= 0 ? fdopen(fd, "w") : NULL;
	char *text = cJSON_PrintUnformatted(value);
	int ret = -1;
	if (file && text) {
		fputs(text, file);
		ret = finish_file(file, dirfd, ".state.tmp", name, flash_kind, candidate);
	} else if (file)
		fclose(file);
	else if (fd >= 0)
		close(fd);
	free(text);
	close(dirfd);
	return ret;
}
int mgr_runtime_config(const char *base, const char *directory, int mode, const char *user_settings, unsigned int flags)
{
	dictionary *ini = iniparser_load(base);
	if (!ini)
		return -1;
	if (flags & MGR_RUNTIME_NO_FILE_LOG)
		iniparser_unset(ini, "system:log_file_path");
	if (user_settings) {
		ys_user_settings_t saved;
		int ret = ys_user_settings_load(user_settings, &saved);
		if (ret < 0) {
			iniparser_freedict(ini);
			return -1;
		}
		if (!ret) {
			const char *keys[] = { "system:echo_mode", "system:echo_type", "system:coordinate_mode" };
			int values[] = { saved.echo_mode, saved.echo_type, saved.coordinate_mode };
			for (int i = 0; i < 3; ++i) {
				char text[8];
				snprintf(text, sizeof(text), "%d", values[i]);
				if (iniparser_set(ini, keys[i], text)) {
					iniparser_freedict(ini);
					return -1;
				}
			}
		}
	}
	char value[16];
	snprintf(value, sizeof(value), "%d", mode);
	if (iniparser_set(ini, "system:work_mode", value)) {
		iniparser_freedict(ini);
		return -1;
	}
	int dirfd = open(directory, O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
	int fd = dirfd < 0 ? -1 :
			     openat(dirfd, ".config.tmp", O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
	FILE *file = fd >= 0 ? fdopen(fd, "w") : NULL;
	int ret = -1;
	if (file) {
		iniparser_dump_ini(ini, file);
		ret = finish_file(file, dirfd, ".config.tmp", "ys_conf.ini", NULL, NULL);
	} else if (fd >= 0)
		close(fd);
	if (dirfd >= 0)
		close(dirfd);
	iniparser_freedict(ini);
	return ret;
}
/* 不经过 shell；限制验证工具运行时间，避免服务被辅助程序永久阻塞。 */
static int execute(char *const args[], char *output, size_t capacity)
{
	int pipes[2];
	if (pipe2(pipes, O_CLOEXEC))
		return -1;
	pid_t pid = fork();
	if (pid == 0) {
		dup2(pipes[1], STDOUT_FILENO);
		close(pipes[0]);
		close(pipes[1]);
		execv(args[0], args);
		_exit(127);
	}
	close(pipes[1]);
	if (pid < 0) {
		close(pipes[0]);
		return -1;
	}
	int status = 0;
	int64_t deadline = mgr_now() + 5000;
	while (waitpid(pid, &status, WNOHANG) == 0) {
		if (mgr_now() >= deadline) {
			kill(pid, SIGKILL);
			waitpid(pid, &status, 0);
			close(pipes[0]);
			return -1;
		}
		usleep(10000);
	}
	ssize_t n = output && capacity ? read(pipes[0], output, capacity - 1) : 0;
	if (output && capacity)
		output[n > 0 ? n : 0] = 0;
	close(pipes[0]);
	return WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : -1;
}
static int verify_tree(const char *root, const char *relative, cJSON *files, const char *openssl, unsigned *count)
{
	char path[PATH_MAX];
	if (snprintf(path, sizeof(path), "%s/%s", root, relative) >= (int)sizeof(path))
		return -1;
	struct stat st;
	if (lstat(path, &st))
		return -1;
	if (S_ISDIR(st.st_mode)) {
		DIR *dir = opendir(path);
		if (!dir)
			return -1;
		struct dirent *entry;
		int ret = 0;
		while ((entry = readdir(dir))) {
			if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, ".."))
				continue;
			char next[PATH_MAX];
			if (snprintf(next, sizeof(next), "%s%s%s", relative, *relative ? "/" : "", entry->d_name) >=
				    (int)sizeof(next) ||
			    verify_tree(root, next, files, openssl, count)) {
				ret = -1;
				break;
			}
		}
		closedir(dir);
		return ret;
	}
	if (!S_ISREG(st.st_mode))
		return -1;
	if (!strcmp(relative, "manifest.json") || !strcmp(relative, "manifest.sig"))
		return 0;
	cJSON *record = cJSON_GetObjectItemCaseSensitive(files, relative);
	const char *hash = mgr_string(record, "sha256");
	cJSON *size = cJSON_GetObjectItemCaseSensitive(record, "size");
	if (!hash || strlen(hash) != 64 || !cJSON_IsNumber(size) || size->valuedouble != (double)st.st_size)
		return -1;
	char output[256];
	char *args[] = { (char *)openssl, "dgst", "-sha256", "-r", path, NULL };
	if (execute(args, output, sizeof(output)) || strncmp(hash, output, 64))
		return -1;
	++*count;
	return 0;
}
int mgr_verify_package(const char *root, const char *key, const char *openssl, int simulation, cJSON **manifest)
{
	char path[PATH_MAX], signature[PATH_MAX];
	if (snprintf(path, sizeof(path), "%s/manifest.json", root) >= (int)sizeof(path) ||
	    snprintf(signature, sizeof(signature), "%s/manifest.sig", root) >= (int)sizeof(signature))
		return -1;
	if (!simulation) {
		char resolved[PATH_MAX];
		if (!key || !realpath(key, resolved) ||
		    (!strncmp(root, resolved, strlen(root)) && resolved[strlen(root)] == '/'))
			return -1;
		char *args[] = { (char *)openssl, "dgst",    "-sha256", "-verify", (char *)key,
				 "-signature",	  signature, path,	NULL };
		if (execute(args, NULL, 0))
			return -1;
	}
	cJSON *value = mgr_read_json(path);
	if (mgr_int(value, "schema_version", 0) != 1) {
		cJSON_Delete(value);
		return -1;
	}
	cJSON *files = cJSON_GetObjectItemCaseSensitive(value, "files");
	unsigned count = 0;
	if (!cJSON_IsObject(files) || verify_tree(root, "", files, openssl, &count) || count == 0 ||
	    count != (unsigned)cJSON_GetArraySize(files)) {
		cJSON_Delete(value);
		return -1;
	}
	*manifest = value;
	return 0;
}
