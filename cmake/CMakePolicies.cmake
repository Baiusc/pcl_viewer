# Eliminate warnings with newer versions of CMake
if ( WIN32 )
	# https://cmake.org/cmake/help/v3.0/policy/CMP0071.html
	cmake_policy(SET CMP0071 NEW)
	#策略表示在INCLUDE_DIRECTORIES属性时禁止使用相对路径
endif()
