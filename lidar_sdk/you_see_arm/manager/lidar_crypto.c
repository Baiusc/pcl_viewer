#define _POSIX_C_SOURCE 200809L
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 只通过 OpenSSL 1.1/3 的公开不透明类型与 EVP 接口调用已安装的密码库。
 * 不访问结构体布局，不实现密码算法；避免把主机架构的 OpenSSL 头混入交叉编译。
 * 接口依据：https://docs.openssl.org/1.1.1/man3/EVP_DigestVerifyInit/
 */
typedef struct evp_md_ctx_st EVP_MD_CTX;
typedef struct evp_md_st EVP_MD;
typedef struct evp_pkey_st EVP_PKEY;
typedef struct evp_pkey_ctx_st EVP_PKEY_CTX;
typedef struct engine_st ENGINE;
typedef int pem_password_cb(char *, int, int, void *);
typedef struct {
	EVP_MD_CTX *(*context_new)(void);
	void (*context_free)(EVP_MD_CTX *);
	const EVP_MD *(*sha256)(void);
	int (*digest_init)(EVP_MD_CTX *, const EVP_MD *, ENGINE *);
	int (*update)(EVP_MD_CTX *, const void *, size_t);
	int (*digest_final)(EVP_MD_CTX *, unsigned char *, unsigned int *);
	int (*verify_init)(EVP_MD_CTX *, EVP_PKEY_CTX **, const EVP_MD *, ENGINE *, EVP_PKEY *);
	int (*verify_final)(EVP_MD_CTX *, const unsigned char *, size_t);
	EVP_PKEY *(*read_key)(FILE *, EVP_PKEY **, pem_password_cb *, void *);
	void (*key_free)(EVP_PKEY *);
} Crypto;

static int load(void *library, Crypto *api)
{
#define LOAD(field, symbol)                                       \
	do {                                                      \
		*(void **)(&api->field) = dlsym(library, symbol); \
		if (!api->field)                                  \
			return -1;                                \
	} while (0)
	LOAD(context_new, "EVP_MD_CTX_new");
	LOAD(context_free, "EVP_MD_CTX_free");
	LOAD(sha256, "EVP_sha256");
	LOAD(digest_init, "EVP_DigestInit_ex");
	LOAD(update, "EVP_DigestUpdate");
	LOAD(digest_final, "EVP_DigestFinal_ex");
	LOAD(verify_init, "EVP_DigestVerifyInit");
	LOAD(verify_final, "EVP_DigestVerifyFinal");
	LOAD(read_key, "PEM_read_PUBKEY");
	LOAD(key_free, "EVP_PKEY_free");
#undef LOAD
	return 0;
}
int main(int argc, char **argv)
{
	int verify = argc == 8 && !strcmp(argv[3], "-verify") && !strcmp(argv[5], "-signature");
	int digest = argc == 5 && !strcmp(argv[3], "-r");
	if ((!verify && !digest) || strcmp(argv[1], "dgst") || strcmp(argv[2], "-sha256")) {
		fprintf(stderr, "仅支持 dgst -sha256 -r 文件，或 dgst -sha256 -verify 公钥 -signature 签名 文件\n");
		return 2;
	}
	/* 验签模式的参数位置与 openssl dgst 一致。 */
	const char *path = argv[argc - 1];
	void *library = dlopen("libcrypto.so.1.1", RTLD_NOW | RTLD_LOCAL);
	if (!library)
		library = dlopen("libcrypto.so.3", RTLD_NOW | RTLD_LOCAL);
	if (!library) {
		fprintf(stderr, "无法加载 libcrypto\n");
		return 1;
	}
	Crypto api = { 0 };
	if (load(library, &api)) {
		fprintf(stderr, "libcrypto 接口不完整\n");
		dlclose(library);
		return 1;
	}
	EVP_MD_CTX *ctx = api.context_new();
	EVP_PKEY *key = NULL;
	FILE *file = NULL;
	int ret = 1;
	if (!ctx)
		goto done;
	if (verify) {
		file = fopen(argv[4], "rb");
		if (!file)
			goto done;
		key = api.read_key(file, NULL, NULL, NULL);
		fclose(file);
		file = NULL;
		if (!key || api.verify_init(ctx, NULL, api.sha256(), NULL, key) != 1)
			goto done;
	} else if (api.digest_init(ctx, api.sha256(), NULL) != 1)
		goto done;
	file = fopen(path, "rb");
	if (!file)
		goto done;
	unsigned char buffer[16384];
	size_t n;
	while ((n = fread(buffer, 1, sizeof(buffer), file)) > 0) {
		if (api.update(ctx, buffer, n) != 1)
			goto done;
	}
	if (ferror(file))
		goto done;
	fclose(file);
	file = NULL;
	if (verify) {
		file = fopen(argv[6], "rb");
		if (!file)
			goto done;
		n = fread(buffer, 1, sizeof(buffer), file);
		if (!n || ferror(file) || !feof(file))
			goto done;
		if (api.verify_final(ctx, buffer, n) != 1)
			goto done;
		puts("签名验证通过");
	} else {
		unsigned char hash[64];
		unsigned int size = sizeof(hash);
		if (api.digest_final(ctx, hash, &size) != 1 || size != 32)
			goto done;
		for (unsigned i = 0; i < size; ++i)
			printf("%02x", hash[i]);
		printf(" *%s\n", path);
	}
	ret = 0;
done:
	if (file)
		fclose(file);
	if (key)
		api.key_free(key);
	if (ctx)
		api.context_free(ctx);
	dlclose(library);
	if (ret)
		fprintf(stderr, "摘要或签名验证失败\n");
	return ret;
}
