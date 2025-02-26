# 对于 libs 目录中的每个共享库，应该调用一次 InstallSharedLibrary。
# 此函数将共享库安装在每个平台的正确位置。
#
# 如果 INSTALL_DESTINATIONS 不为空，它将安装到列表中的每个目标。
# 如果 INSTALL_DESTINATIONS 为空，则此函数不执行任何操作。
#
# 参数：
# 目标 库目标的名称
function(InstallSharedLibrary)
	if(NOT INSTALL_DESTINATIONS)
		return()
	endif()

#解析传入参数的 “TARGET”的值，并将其写入到INSTALL_SHARED_LIB 变量
	cmake_parse_arguments(
			INSTALL_SHARED_LIB
			""
			"TARGET"
			""
			${ARGN}
	)

	# For readability
	set(shared_lib_target "${INSTALL_SHARED_LIB_TARGET}")
	message(STATUS "Install shared library: ${shared_lib_target}")

	foreach (destination ${INSTALL_DESTINATIONS})
		if(UNIX AND NOT APPLE)
			set(destination ${LINUX_INSTALL_SHARED_DESTINATION})
		endif()
#调用_InstallSharedTarget 函数
		_InstallSharedTarget(
				TARGET ${shared_lib_target}
				DEST_PATH ${destination}
		)
	endforeach ()
endfunction()

# 应调用安装文件来安装不是目标的文件。
# 此函数将文件安装在每个平台的正确位置。
#
# 如果 INSTALL_DESTINATIONS 不为空，它将安装到列表中的每个目标。
# 如果 INSTALL_DESTINATIONS 为空，则此函数不执行任何操作。
#
# 参数：
# 文件 要安装的文件的名称
function( InstallFiles )
	if( NOT INSTALL_DESTINATIONS )
		return()
	endif()
	#解析传入参数
	cmake_parse_arguments(
			INSTALL_FILES
			""
			""
			"FILES"
			${ARGN}
	)

	# For readability
	set( files "${INSTALL_FILES_FILES}" )

	if( NOT files )
		message( WARNING "InstallFiles: no files specified" )
		return()
	endif()
	
	message( STATUS "Install files: ${files} to ${INSTALL_DESTINATIONS}")
	
	foreach( destination ${INSTALL_DESTINATIONS} )			
	#调用 _InstallFiles函数
		_InstallFiles(
			FILES ${files}
			DEST_PATH ${destination}
		)		
	endforeach()
endfunction()

# 每个应用应该调用一次安装插件。
# 此函数将 TYPES 中请求的插件类型安装到 DEST_FOLDER 指定的路径中。
# 如果是带有着色器的 gl 插件，请安装着色器以SHADER_DEST_FOLDER。
#
# 参数：
# DEST_FOLDER 安装插件的目录名称。
# DEST_PATH DEST_FOLDER路径 - 请注意，在 Windows 上，我们将根据配置对其进行修改
# SHADER_DEST_FOLDER 为插件安装着色器的目录的名称。
# SHADER_DEST_PATH 路径SHADER_DEST_FOLDER - 请注意，在 Windows 上，我们将根据配置对其进行修改
# TYPES 要安装的插件类型列表（有效：gl、io、standard）。如果未指定，请全部安装。
function( InstallPlugins )
	cmake_parse_arguments(
			INSTALL_PLUGINS
			""
			"DEST_FOLDER;DEST_PATH;SHADER_DEST_FOLDER;SHADER_DEST_PATH"
			"TYPES"
			${ARGN}
	)
	
	# Check the types we need to install
	set( VALID_TYPES "gl" "io" "standard" )
	
	# If TYPES was not specified, use all of them
	if( NOT INSTALL_PLUGINS_TYPES )
		set( INSTALL_PLUGINS_TYPES "${VALID_TYPES}" )
	else()
		foreach( type ${INSTALL_PLUGINS_TYPES} )
			if( NOT "${type}" IN_LIST VALID_TYPES )		   
				# In cmake 3.12:
				# list( JOIN VALID_TYPES ", " VALID_TYPES_STR )
				string( REPLACE ";" ", " VALID_TYPES_STR "${VALID_TYPES}" )
				
				message( FATAL_ERROR "InstallPlugins: Did not find proper TYPES. Valid values are: ${VALID_TYPES_STR}" )
			endif()
		endforeach()
	endif()
	
	message( STATUS "Install plugins" )
	message( STATUS " Types: ${INSTALL_PLUGINS_TYPES}" )
	
	# Check our destination path is valid
	if( NOT INSTALL_PLUGINS_DEST_PATH )
		message( FATAL_ERROR "InstallPlugins: DEST_PATH not specified" )
	endif()
	
	message( STATUS " Destination: ${INSTALL_PLUGINS_DEST_PATH}/${INSTALL_PLUGINS_DEST_FOLDER}" )
	
	# If we have gl plugins, check that our shader destination folder is valid
	if( "gl" IN_LIST VALID_TYPES )
		if( NOT INSTALL_PLUGINS_SHADER_DEST_PATH )
			message( FATAL_ERROR "InstallPlugins: SHADER_DEST_PATH not specified" )
		endif()
		
		message( STATUS " Shader Destination: ${INSTALL_PLUGINS_SHADER_DEST_PATH}/${INSTALL_PLUGINS_SHADER_DEST_FOLDER}" )
	endif()

	# Make CloudCompare depend on the plugins
	# so that when building CloudCompare the plugins also get built
	# instead of waiting for the `install` target to be ran for the plugins to get built
	message("=========================${CC_PLUGIN_TARGET_LIST}")
	add_dependencies(${PROJECT_NAME} ${CC_PLUGIN_TARGET_LIST})

	# Install the requested plugins in the DEST_FOLDER
	foreach( plugin_target ${CC_PLUGIN_TARGET_LIST} )
		get_target_property( plugin_type ${plugin_target} PLUGIN_TYPE )
		
		if( "${plugin_type}" IN_LIST INSTALL_PLUGINS_TYPES )
			message( STATUS " Install ${plugin_target} (${plugin_type})" )
			
			_InstallSharedTarget(
				TARGET ${plugin_target}
				DEST_PATH ${INSTALL_PLUGINS_DEST_PATH}
				DEST_FOLDER ${INSTALL_PLUGINS_DEST_FOLDER}
			)		
			
			if( "${plugin_type}" STREQUAL "gl" )
				get_target_property( SHADER_FOLDER_NAME ${plugin_target} SHADER_FOLDER_NAME )
				get_target_property( SHADER_FOLDER_PATH ${plugin_target} SHADER_FOLDER_PATH )
				
				if( EXISTS "${SHADER_FOLDER_PATH}" )
					message( STATUS "  + shader: ${SHADER_FOLDER_NAME} (${SHADER_FOLDER_PATH})" )
					
					get_target_property( shader_files ${plugin_target} SOURCES )
					list( FILTER shader_files INCLUDE REGEX ".*\.vert|frag" )					
					
					_InstallFiles(
						FILES ${shader_files}
						DEST_PATH ${INSTALL_PLUGINS_SHADER_DEST_PATH}
						DEST_FOLDER ${INSTALL_PLUGINS_SHADER_DEST_FOLDER}/${SHADER_FOLDER_NAME}
					)
				endif()
			endif()
		endif()
	endforeach()
endfunction()

# _InstallSharedTarget应该只由上面的一个函数调用。
# It was factored out to provide cmake < 3.13 a way to install shared libs.
#
# Arguments:
#	DEST_FOLDER lib库的安桩目录名The name of the directory to install the shared lib in.
#	DEST_PATH Path to DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	TARGET lib库的名称The name of the shared lib target
function( _InstallSharedTarget )
	cmake_parse_arguments(
			INSTALL_SHARED_TARGET
			""
			"DEST_FOLDER;DEST_PATH;TARGET"
			""
			${ARGN}
	)
	
	# For readability
	set( shared_target "${INSTALL_SHARED_TARGET_TARGET}" )
	set( full_path "${INSTALL_SHARED_TARGET_DEST_PATH}/${INSTALL_SHARED_TARGET_DEST_FOLDER}" )
	
	# Before CMake 3.13, install(TARGETS) would only accept targets created in the same directory scope
	# This makes it difficult to work with submodules.
	# This can be cleaned up when we move to a minimum CMake of 3.13 or higher
	# https://gitlab.kitware.com/cmake/cmake/-/merge_requests/2152
	if ( ${CMAKE_VERSION} VERSION_LESS "3.13.0" )
		# Basic hack: construct the name of the dynamic library ("target_shared_lib") and install using
		# install(FILES) instead of install(TARGETS)
		
		if ( APPLE OR UNIX )
			set( lib_prefix "lib" )
		endif()
		
		if ( CMAKE_BUILD_TYPE STREQUAL "Debug" )
			get_target_property( lib_postfix ${shared_target} DEBUG_POSTFIX)
		endif()
		
		get_target_property( target_bin_dir ${shared_target} BINARY_DIR )
		
		set( target_shared_lib "${target_bin_dir}/${lib_prefix}${shared_target}${lib_postfix}${CMAKE_SHARED_LIBRARY_SUFFIX}" )
				
		if ( WIN32 )
			copy_files( "${target_shared_lib}" "${full_path}" 1 )
		else()
			copy_files( "${target_shared_lib}" "${full_path}" )
		endif()
	else()	
		if( WIN32 )
			if( NOT CMAKE_CONFIGURATION_TYPES )
				install(
					TARGETS ${shared_target}
					RUNTIME DESTINATION ${full_path}
				)
			else()
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS Debug
					RUNTIME DESTINATION ${INSTALL_SHARED_TARGET_DEST_PATH}_debug/${INSTALL_SHARED_TARGET_DEST_FOLDER}
				)
			
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS Release
					RUNTIME DESTINATION ${full_path}
				)
			
				install(
					TARGETS ${shared_target}
					CONFIGURATIONS RelWithDebInfo
					RUNTIME DESTINATION ${INSTALL_SHARED_TARGET_DEST_PATH}_withDebInfo/${INSTALL_SHARED_TARGET_DEST_FOLDER}
				)
			endif()			
		else()
			install( TARGETS ${shared_target}
				LIBRARY DESTINATION ${full_path}
				COMPONENT Runtime
			)
		endif()
	endif()
endfunction()

# _InstallFiles should only be called by one of the functions above.
#
# Arguments:
#	DEST_FOLDER The name of the directory to install the files in.
#	DEST_PATH Path to DEST_FOLDER - note that on Windows we will modify this depending on CONFIGURATIONS
#	FILES The name of the files to install
function( _InstallFiles )
	cmake_parse_arguments(
			INSTALL_FILES
			""
			"DEST_FOLDER;DEST_PATH"
			"FILES"
			${ARGN}
	)

	# For readability
	set( files "${INSTALL_FILES_FILES}" )
	set( full_path "${INSTALL_FILES_DEST_PATH}/${INSTALL_FILES_DEST_FOLDER}" )
	
	if( WIN32 )
		if( NOT CMAKE_CONFIGURATION_TYPES )
			install(
				FILES ${files}
				DESTINATION "${full_path}"
			)
		else()
			install(
				FILES ${files}
				CONFIGURATIONS Debug
				DESTINATION "${INSTALL_FILES_DEST_PATH}_debug/${INSTALL_FILES_DEST_FOLDER}"
			)
		
			install(
				FILES ${files}
				CONFIGURATIONS Release
				RUNTIME DESTINATION ${full_path}
			)
		
			install(
				FILES ${files}
				CONFIGURATIONS RelWithDebInfo
				RUNTIME DESTINATION "${INSTALL_FILES_DEST_PATH}_withDebInfo/${INSTALL_FILES_DEST_FOLDER}"
			)
		endif()			
	else()
		install(
			FILES ${files}
			DESTINATION "${full_path}"
		)
	endif()
endfunction()
