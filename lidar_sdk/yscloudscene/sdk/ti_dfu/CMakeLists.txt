cmake_minimum_required(VERSION 3.20)
project(TiDfu LANGUAGES CXX)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Widgets)

add_library(ti_dfu STATIC flasher.cpp flasher.h)
add_library(TiDfu::Core ALIAS ti_dfu)
set_target_properties(ti_dfu PROPERTIES AUTOMOC ON)
target_compile_features(ti_dfu PUBLIC cxx_std_17)
target_include_directories(ti_dfu PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(ti_dfu PUBLIC Qt${QT_VERSION_MAJOR}::Core)

add_library(ti_dfu_widgets STATIC flash_dialog.cpp flash_dialog.h)
add_library(TiDfu::Widgets ALIAS ti_dfu_widgets)
target_link_libraries(ti_dfu_widgets PUBLIC TiDfu::Core Qt${QT_VERSION_MAJOR}::Widgets)
