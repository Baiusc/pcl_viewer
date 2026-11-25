# 兼容入口：所有构建逻辑统一交给 CMake，必须明确选择预设。
.DEFAULT_GOAL := all
.PHONY: all build_only you_see configure package list help clean clean_other driver_lib
all build_only you_see:
	@test -n "$(PRESET)" || { echo '请使用 make PRESET=beta；make list 查看组合'; exit 2; }
	python3 scripts/build.py build "$(PRESET)"
configure:
	python3 scripts/build.py configure "$(PRESET)"
package:
	python3 scripts/build.py package "$(PRESET)"
list:
	python3 scripts/build.py list
clean:
	@test -n "$(PRESET)" || { echo '清理时必须指定 PRESET'; exit 2; }
	cmake --build "build/$(PRESET)" --target clean
clean_other:
	@echo '独立构建目录保留中间产物用于增量编译，无需 clean_other'
driver_lib: configure
	cmake --build "build/$(PRESET)" --target ys_driver
help:
	@echo 'make list / make PRESET=beta / make package PRESET=beta'
	@echo 'Beta/Gamma 使用按机型预设，work_mode 在运行配置中选择'
