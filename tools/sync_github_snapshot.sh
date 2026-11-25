#!/usr/bin/env bash
#
# 生成可独立 clone 的 GitHub 快照分支：把 lidar_sdk 下的子模块展开成普通源码。
#
# 背景：Git 索引里一条路径要么是 gitlink（子模块），要么是普通文件，无法共存。
#   - GitLab 侧（内网）：保留子模块，各机器 git submodule update --init 拉取；
#   - GitHub 侧（外网）：子模块地址是内网 10.42.0.10，外部拉不到，
#     所以把子模块内容内联成普通文件，生成一个自包含的快照分支。
#
# 用法：
#   tools/sync_github_snapshot.sh             # 生成快照分支，不推送
#   tools/sync_github_snapshot.sh --push      # 生成并推送到 github
#   tools/sync_github_snapshot.sh --full      # 连数据/PDF/压缩包一起内联（约 68M）
#
# 可用环境变量覆盖：SRC_BRANCH / DST_BRANCH / GH_REMOTE
#
set -euo pipefail

SRC_BRANCH="${SRC_BRANCH:-cloud_viewer}"
DST_BRANCH="${DST_BRANCH:-github-snapshot}"
GH_REMOTE="${GH_REMOTE:-github}"
SUBMODULES=(lidar_sdk/you_see_arm lidar_sdk/yscloudscene)

PUSH=0
FULL=0
for arg in "$@"; do
	case "$arg" in
	--push) PUSH=1 ;;
	--full) FULL=1 ;;
	-h | --help)
		echo "用法: $0 [--push] [--full]"
		echo "  --push  生成后推送到 \$GH_REMOTE/$DST_BRANCH（默认 github）"
		echo "  --full  内联全部入库文件（含数据/PDF/压缩包，约 68M）；默认只内联源码，约 25M"
		exit 0
		;;
	*)
		echo "未知参数: $arg" >&2
		exit 1
		;;
	esac
done

ROOT="$(git -C "$(dirname "$0")" rev-parse --show-toplevel)"
cd "$ROOT"
WT="$ROOT/.build-tmp/github-snapshot"

cleanup() {
	git worktree remove --force "$WT" >/dev/null 2>&1 || rm -rf "$WT"
	git worktree prune >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== 源分支 $SRC_BRANCH -> 快照分支 $DST_BRANCH（模式: $([ "$FULL" -eq 1 ] && echo 全量 || echo 仅源码)）=="
rm -rf "$WT"
git worktree add -B "$DST_BRANCH" "$WT" "$SRC_BRANCH" >/dev/null
cd "$WT"

# 1) 拆掉子模块元信息：快照分支里这两个目录必须是普通文件
git rm -q --cached -r "${SUBMODULES[@]}" 2>/dev/null || true
git rm -q -f .gitmodules 2>/dev/null || true

# 2) 按子仓库自己的入库清单展开（git ls-files = 与上游一致的集合）
#    注意 you_see_arm 里还有嵌套子模块（algo/calib_reflectivity、cc、driver …），
#    它们在 ls-files 里是 160000 的 gitlink：已初始化的递归展开，未初始化的跳过。
total=0
nested_skipped=()
for d in "${SUBMODULES[@]}"; do
	if [ ! -d "$ROOT/$d" ]; then
		echo "  跳过（本地不存在）: $d"
		continue
	fi
	declare -A nested=()
	# -z：NUL 分隔，避免中文路径被 core.quotepath 转义成 "\351\233\206..."
	while IFS= read -r -d '' rec; do
		[ "${rec%% *}" = "160000" ] || continue
		nested["${rec#*$'\t'}"]=1
	done < <(git -C "$ROOT/$d" ls-files -sz)

	count=0
	while IFS= read -r -d '' f; do
		if [ "$FULL" -eq 0 ]; then
			case "$f" in
			*.tar.* | *.tar | *.pdf | */raw_* | */gray_data | */raw_histogram_data) continue ;;
			esac
		fi
		src="$ROOT/$d/$f"
		if [ -n "${nested[$f]:-}" ]; then
			if [ -d "$src" ] && [ -n "$(ls -A "$src" 2>/dev/null)" ]; then
				mkdir -p "$d/$f"
				rsync -a --exclude=.git "$src/" "$d/$f/"
				count=$((count + 1))
			else
				nested_skipped+=("$d/$f")
			fi
			continue
		fi
		mkdir -p "$(dirname "$d/$f")"
		cp -p "$src" "$d/$f"
		count=$((count + 1))
	done < <(git -C "$ROOT/$d" ls-files -z)
	unset nested
	echo "  $d: $count 个文件"
	total=$((total + count))
done

if [ "${#nested_skipped[@]}" -gt 0 ]; then
	echo "  注意: 以下嵌套子模块本地未初始化，快照里会被跳过（需要时在 you_see_arm 里 git submodule update --init）"
	printf '    - %s\n' "${nested_skipped[@]}"
fi

# 3) 主仓库 .gitignore 有 *.jpg / *.pcd / data/ 这类规则，会误伤子项目源码，用 -f 强制入库
for d in "${SUBMODULES[@]}"; do
	if [ -d "$d" ]; then
		git add -f "$d" 2>/dev/null || true
	fi
done
git add -A

if git diff --cached --quiet && git diff --quiet; then
	echo "== 无变化，快照分支保持不变 =="
else
	git commit -q -m "chore(github): 内联子模块源码，GitHub 侧可独立 clone（$(date +%F)）"
	echo "== 已提交: $(git rev-parse --short HEAD) =="
fi

echo "== 快照概况 =="
git --no-pager log --oneline -1
echo "  本次展开 $total 个文件，快照分支共跟踪 $(git ls-files | wc -l) 个文件"

if [ "$PUSH" -eq 1 ]; then
	echo "== 推送到 $GH_REMOTE/$DST_BRANCH =="
	# 快照分支每次从源分支重建，历史会与远端分叉，必须 force
	git push -f "$GH_REMOTE" "HEAD:$DST_BRANCH"
fi
