#!/bin/sh
# 生成固件版本标识头文件(构建期由 Makefile / CMake 调用)
#
# 用法: gen_version.sh <输出头文件路径>
#
# 设计约定:
#   1. 所有版本信息均从 git 自动推导, 无需任何手工维护;
#      固件版本取 v1.2.3 形式的 tag, 协议版本取 proto-v1.2.3 形式的 tag。
#   2. 内容不变时不触碰目标文件, 从而不触发无谓的重新编译。
#   3. 任何失败分支都必须产出一个可编译的头文件并以 0 退出, 绝不中断构建。
set -u

out="${1:-}"
if [ -z "$out" ]; then
	echo "gen_version.sh: 用法: gen_version.sh <输出头文件路径>" >&2
	exit 0
fi

# ---- git 探测: 以当前目录为起点向上查找仓库根, 找不到即视为无 git(tarball 构建) ----
git_available=0
sha=""
dirty=0
fw_major=0
fw_minor=0
fw_patch=0
proto_major=1
proto_minor=0
proto_patch=0
tag_note=0

repo_root=$(git rev-parse --show-toplevel 2>/dev/null) || repo_root=""

if [ -n "$repo_root" ]; then
	sha=$(git -C "$repo_root" rev-parse --short=7 HEAD 2>/dev/null) || sha=""
fi

if [ -n "$sha" ]; then
	git_available=1

	# 短 sha 长度上限 12: git 在存在歧义时会自动加长, 超出部分截断
	sha_len=${#sha}
	if [ "$sha_len" -gt 12 ]; then
		sha=$(printf '%s' "$sha" | cut -c1-12)
	fi

	# dirty: 只统计本应用自身的改动。子模块各自带版本,
	# driver 等子模块脏不应把 you_see 标记为不可复现
	if [ -n "$(git -C "$repo_root" status --porcelain --ignore-submodules=all 2>/dev/null)" ]; then
		dirty=1
	fi

	# 固件版本: 取最近的 v1.2.3 形式 tag
	fw_tag=$(git -C "$repo_root" describe --tags --match 'v[0-9]*' --abbrev=0 2>/dev/null) || fw_tag=""
	if [ -n "$fw_tag" ]; then
		fw_ver=${fw_tag#v}
		fw_major=$(printf '%s' "$fw_ver" | cut -d. -f1)
		fw_minor=$(printf '%s' "$fw_ver" | cut -d. -f2)
		fw_patch=$(printf '%s' "$fw_ver" | cut -d. -f3)
		# tag 名必须是纯数字三段, 否则视为未打版本 tag
		for seg in "$fw_major" "$fw_minor" "$fw_patch"; do
			case "$seg" in
			'' | *[!0-9]*)
				fw_major=0
				fw_minor=0
				fw_patch=0
				fw_tag=""
				break
				;;
			esac
		done
	fi

	if [ -n "$fw_tag" ]; then
		tag_note=$(git -C "$repo_root" rev-list --count "${fw_tag}..HEAD" 2>/dev/null) || tag_note=0
	else
		echo "gen_version.sh: 警告: 未找到 v1.2.3 形式的固件版本 tag, 固件版本退化为 0.0.0(靠 git sha 定位)" >&2
		tag_note=0
	fi

	# 协议版本: 独立命名空间 proto-v1.2.3, 与固件发版解耦
	proto_tag=$(git -C "$repo_root" describe --tags --match 'proto-v[0-9]*' --abbrev=0 2>/dev/null) || proto_tag=""
	proto_ok=0
	if [ -n "$proto_tag" ]; then
		proto_ver=${proto_tag#proto-v}
		proto_major=$(printf '%s' "$proto_ver" | cut -d. -f1)
		proto_minor=$(printf '%s' "$proto_ver" | cut -d. -f2)
		proto_patch=$(printf '%s' "$proto_ver" | cut -d. -f3)
		proto_ok=1
		for seg in "$proto_major" "$proto_minor" "$proto_patch"; do
			case "$seg" in
			'' | *[!0-9]*)
				proto_ok=0
				break
				;;
			esac
		done
	fi

	if [ "$proto_ok" != "1" ]; then
		proto_major=1
		proto_minor=0
		proto_patch=0
		echo "gen_version.sh: 警告: 未找到 proto-v1.2.3 形式的协议版本 tag, 退化为 1.0.0。" >&2
		echo "gen_version.sh:       当前线上协议即为 v1, 该退化值不会影响广播与应答;" >&2
		echo "gen_version.sh:       建议尽快打 tag: git tag proto-v1.0.0" >&2
	fi
else
	echo "gen_version.sh: 警告: 未获取到 git 信息(非 git 仓库或 git 不可用), 版本标识记为未知" >&2
fi

# ---- 构建时间: 支持 SOURCE_DATE_EPOCH 以实现可复现构建, 否则取当前 UTC 时间 ----
build_time=0
if [ -n "${SOURCE_DATE_EPOCH:-}" ]; then
	case "$SOURCE_DATE_EPOCH" in
	'' | *[!0-9]*) build_time=0 ;;
	*) build_time=$SOURCE_DATE_EPOCH ;;
	esac
fi
if [ "$build_time" = "0" ]; then
	build_time=$(date -u +%s 2>/dev/null) || build_time=0
fi

# ---- 发布标识: CI 可用 YS_RELEASE_ID 覆盖, 否则派生为 YYYYMMDD-<距最近固件 tag 的提交数> ----
if [ -n "${YS_RELEASE_ID:-}" ]; then
	release_id=$YS_RELEASE_ID
else
	release_day=$(date -u -d "@${build_time}" +%Y%m%d 2>/dev/null) || release_day=$(date -u +%Y%m%d 2>/dev/null) ||
		release_day="00000000"
	release_id=$(printf '%s-%03d' "$release_day" "$tag_note" 2>/dev/null) || release_id="${release_day}-000"
fi
# 头文件中该字段为 16 字节(含结尾 NUL), 超出即截断
release_id=$(printf '%s' "$release_id" | cut -c1-15)

# ---- 生成头文件: 先写临时文件, 内容有变化才替换目标, 避免无谓触发重新编译 ----
tmp="${out}.tmp.$$"
trap 'rm -f "$tmp"' EXIT INT TERM

mkdir -p "$(dirname "$out")" 2>/dev/null

cat >"$tmp" <<EOF
/*
 * 本文件由 scripts/gen_version.sh 在构建时自动生成, 请勿手工修改, 也勿纳入版本控制。
 * 所有值均来自 git, 无需人工维护。
 */
#ifndef YS_VERSION_GEN_H
#define YS_VERSION_GEN_H

#define YS_BUILD_GIT_AVAILABLE ${git_available}
#define YS_BUILD_GIT_SHA "${sha}"
#define YS_BUILD_GIT_SHA_LEN ${#sha}
#define YS_BUILD_DIRTY ${dirty}
#define YS_BUILD_TIME ${build_time}ULL

#define YS_BUILD_FW_MAJOR ${fw_major}
#define YS_BUILD_FW_MINOR ${fw_minor}
#define YS_BUILD_FW_PATCH ${fw_patch}

#define YS_BUILD_PROTO_MAJOR ${proto_major}
#define YS_BUILD_PROTO_MINOR ${proto_minor}
#define YS_BUILD_PROTO_PATCH ${proto_patch}

#define YS_BUILD_RELEASE_ID "${release_id}"
#define YS_BUILD_TAG_NOTE ${tag_note}

#endif /* YS_VERSION_GEN_H */
EOF

if [ -f "$out" ] && cmp -s "$tmp" "$out"; then
	rm -f "$tmp"
else
	mv -f "$tmp" "$out" 2>/dev/null || {
		rm -f "$tmp"
		echo "gen_version.sh: 错误: 无法写入 $out" >&2
		exit 0
	}
fi

trap - EXIT INT TERM
exit 0
