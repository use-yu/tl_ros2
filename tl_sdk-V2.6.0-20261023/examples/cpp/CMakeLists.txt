# ============================================================
# TL Robot Arm SDK — C++ 示例
#
# 双模式支持:
#   1. 独立构建: cd examples/cpp && cmake --preset linux-native
#   2. 子目录模式: 从根 CMakeLists.txt add_subdirectory
# ============================================================

# ── 双模式检测 ────────────────────────────────────────────
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    # 模式1: 独立构建
    cmake_minimum_required(VERSION 3.16)
    project(TL_Robot_Arm_SDK_Examples LANGUAGES CXX)
    set(STANDALONE_BUILD TRUE)
else()
    # 模式2: 从根目录引入（tl_host + tl_extension 由父 CMake 提供）
    set(STANDALONE_BUILD FALSE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC /utf-8：防止 UTF-8 中文注释/字符串被误解析为 GBK（C4819 警告）
add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/utf-8>)

# 项目根目录
set(PROJ_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

# ---- 平台检测（两种模式都需要）----
if(WIN32)
    set(IS_WINDOWS TRUE)
else()
    set(IS_LINUX TRUE)
endif()

# ============================================================
# 目标定义（仅在独立构建模式下）
# ============================================================
if(STANDALONE_BUILD)

    # 分发包平台目录名（与 src/tl_host/CMakeLists.txt 保持一致）
    if(NOT DEFINED PLATFORM_DIR)
        if(WIN32)
            set(PLATFORM_DIR "windows")
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)")
            set(PLATFORM_DIR "linux/arm64")
        else()
            set(PLATFORM_DIR "linux/x86")
        endif()
    endif()

    if(EXISTS "${PROJ_ROOT}/src/tl_host/CMakeLists.txt")
        # 源码树模式: tl_host 从源码构建（封装层 + 厂商静态库，开发仓库内使用）
        set(HAS_TL_HOST_SOURCE TRUE)
        add_subdirectory(${PROJ_ROOT}/src/tl_host ${CMAKE_BINARY_DIR}/src/tl_host)
        if(WIN32)
            set(PLATFORM_LIBS "")
        else()
            set(PLATFORM_LIBS pthread)
        endif()
    else()
        # 分发包模式: install/ 分发包无 src/ 目录，链接 lib/<platform>/ 预编译动态库
        set(HAS_TL_HOST_SOURCE FALSE)
        if(WIN32)
            set(_TL_HOST_LIB_FILE "${PROJ_ROOT}/lib/windows/tl_host.dll")
            if(NOT EXISTS "${_TL_HOST_LIB_FILE}" OR NOT EXISTS "${PROJ_ROOT}/lib/windows/tl_host.lib")
                message(FATAL_ERROR "分发包缺少 Windows 动态库: lib/windows/tl_host.dll / tl_host.lib")
            endif()
            add_library(tl_host SHARED IMPORTED)
            set_target_properties(tl_host PROPERTIES
                IMPORTED_LOCATION "${_TL_HOST_LIB_FILE}"
                IMPORTED_IMPLIB   "${PROJ_ROOT}/lib/windows/tl_host.lib"
            )
        else()
            set(_TL_HOST_LIB_FILE "${PROJ_ROOT}/lib/${PLATFORM_DIR}/libtl_host.so")
            if(NOT EXISTS "${_TL_HOST_LIB_FILE}")
                message(FATAL_ERROR "分发包缺少动态库: ${_TL_HOST_LIB_FILE}")
            endif()
            add_library(tl_host SHARED IMPORTED)
            set_target_properties(tl_host PROPERTIES
                IMPORTED_LOCATION "${_TL_HOST_LIB_FILE}"
            )
        endif()
    endif()
endif()

# ---- 导纳控制示例（链接 tl_host 合并库）— 使用 omp 自主导纳，全平台可用 ----
# SDL2 依赖（F710 手柄遥操作示例）
# ============================================================
find_package(SDL2 QUIET)
set(HAS_SDL2 FALSE)
if(SDL2_FOUND)
    set(HAS_SDL2 TRUE)
elseif(IS_LINUX)
    if(CMAKE_CROSSCOMPILING)
        if(DEFINED ENV{PKG_CONFIG_LIBDIR})
            find_package(PkgConfig QUIET)
            if(PkgConfig_FOUND)
                pkg_check_modules(SDL2 sdl2)
                if(SDL2_FOUND)
                    set(HAS_SDL2 TRUE)
                endif()
            endif()
        endif()
    else()
        find_package(PkgConfig QUIET)
        if(PkgConfig_FOUND)
            pkg_check_modules(SDL2 sdl2)
            if(SDL2_FOUND)
                set(HAS_SDL2 TRUE)
            endif()
        endif()
    endif()
endif()

if(NOT HAS_SDL2)
    message(STATUS "SDL2 未找到，跳过遥操作示例 (sudo apt install libsdl2-dev)")
endif()

# ============================================================
# 辅助函数: 配置示例运行时动态库定位（三种模式）
#   子目录模式      : tl_host 由父 CMake 构建提供，RPATH 指向父构建树产物
#   源码模式独立构建: RPATH 指向构建树内 tl_host 产物目录
#   分发包模式      : 动态库拷贝到可执行文件目录（Linux $ORIGIN / Windows 同目录搜索），
#                     可执行文件移动任意位置均可运行，不依赖分发包绝对路径
# ============================================================
function(configure_example_runtime name)
    if(NOT STANDALONE_BUILD)
        # 子目录模式: tl_host 由父 CMake 构建提供
        if(IS_LINUX)
            set_target_properties(${name} PROPERTIES
                BUILD_RPATH "$<TARGET_FILE_DIR:tl_host>"
            )
        endif()
    elseif(HAS_TL_HOST_SOURCE)
        # 源码模式独立构建
        if(IS_LINUX)
            set_target_properties(${name} PROPERTIES
                BUILD_RPATH "$<TARGET_FILE_DIR:tl_host>"
            )
        endif()
    else()
        # 分发包模式
        if(IS_LINUX)
            set_target_properties(${name} PROPERTIES
                BUILD_RPATH "$ORIGIN"
            )
        endif()
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_TL_HOST_LIB_FILE}" "$<TARGET_FILE_DIR:${name}>/"
            COMMENT "Copy ${_TL_HOST_LIB_FILE} to $<TARGET_FILE_DIR:${name}>"
        )
    endif()

    # Windows: 拷贝 tl_host 动态库到可执行文件目录
    # （源码模式取构建产物，分发包模式取 IMPORTED 库路径）
    if(WIN32)
        if(STANDALONE_BUILD AND NOT HAS_TL_HOST_SOURCE)
            set(_TL_HOST_COPY_SRC "${_TL_HOST_LIB_FILE}")
        else()
            set(_TL_HOST_COPY_SRC "$<TARGET_FILE_DIR:tl_host>/$<TARGET_FILE_NAME:tl_host>")
        endif()
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_TL_HOST_COPY_SRC}" "$<TARGET_FILE_DIR:${name}>/"
            COMMENT "Copy DLLs to $<TARGET_FILE_DIR:${name}>"
        )
    endif()
endfunction()

# ============================================================
# 辅助函数: 创建 SDK 示例（链接 tl_host）
# ============================================================
function(add_sdk_example name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${PROJ_ROOT}/include)
    target_link_libraries(${name} PRIVATE tl_host)
    configure_example_runtime(${name})
endfunction()

# ============================================================
# 辅助函数: 创建 Extension 示例（链接 tl_host 合并库，tl_extension 已并入）
# ============================================================
function(add_extension_example name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${PROJ_ROOT}/include)
    target_link_libraries(${name} PRIVATE tl_host)
    configure_example_runtime(${name})
endfunction()

# ============================================================
# 辅助函数: 创建导纳控制示例（使用 omp 自主导纳，链接 tl_extension + tl_host）
# ============================================================
function(add_admittance_example name source)
    add_executable(${name} ${source})
    target_include_directories(${name} PRIVATE ${PROJ_ROOT}/include)
    target_link_libraries(${name} PRIVATE tl_host)
    configure_example_runtime(${name})
endfunction()

# =============================================================
# 辅助函数: SDL2 链接（给遥操作示例用）
# ============================================================
function(target_link_sdl2 name)
    if(TARGET SDL2::SDL2)
        target_link_libraries(${name} PRIVATE SDL2::SDL2)
    elseif(SDL2_FOUND)
        target_link_libraries(${name} PRIVATE ${SDL2_LIBRARIES})
        target_include_directories(${name} PRIVATE ${SDL2_INCLUDE_DIRS})
    else()
        target_link_libraries(${name} PRIVATE ${SDL2_LDFLAGS})
        target_include_directories(${name} PRIVATE ${SDL2_CFLAGS_OTHER})
    endif()
endfunction()

# ============================================================
# 源文件列表（编译 + 安装共用同一个列表）
# ============================================================
set(SDK_SOURCES
    ex_connect.cpp
    ex_coord_tool.cpp
    ex_coord_transform.cpp
    ex_collision_detect.cpp
    ex_digital_io.cpp
    ex_error_callback.cpp
    ex_error_zero.cpp
    ex_global_position.cpp
    ex_info_query.cpp
    ex_job_control.cpp
    ex_log_download.cpp
    ex_inspire_claw.cpp
    ex_omnipicker_claw.cpp
    ex_move_control.cpp
    ex_queue_motion.cpp
    ex_robot_state_callback.cpp
    ex_servo.cpp
    ex_servoj.cpp
    ex_six_axis_force.cpp
    ex_six_axis_force_drag.cpp
    ex_teach_control.cpp
    ex_tool_coord_ik.cpp
    ex_tool_coord_move.cpp
    ex_tool_hand_number.cpp
    ex_track_queue_drag.cpp
    ex_user_coord_compare.cpp
    ex_user_coord_ik.cpp
)

set(EXTENSION_SOURCES
    ex_servo_movej.cpp
    ex_servo_movel.cpp
)

set(ADMITTANCE_SOURCES
    ex_admittance_control.cpp
    ex_massage.cpp
    ex_constant_force.cpp
)

# ============================================================
# SDK 示例（仅链接 tl_host）
# ============================================================
foreach(src ${SDK_SOURCES})
    string(REPLACE ".cpp" "" name ${src})
    add_sdk_example(${name} ${src})
endforeach()

# ---- 遥操作（需要 SDL2）----
if(HAS_SDL2)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ex_teleop_f710_servoj.cpp")
        add_sdk_example(ex_teleop_f710_servoj ex_teleop_f710_servoj.cpp)
        target_link_sdl2(ex_teleop_f710_servoj)
    else()
        # 旧分发包未携带该源文件（打包机无 SDL2 时未随包安装），跳过而非报错
        message(STATUS "ex_teleop_f710_servoj.cpp 不在分发包内，跳过遥操作示例")
    endif()
endif()

# ============================================================
# Extension 示例（链接 tl_extension + tl_host）
# ============================================================
foreach(src ${EXTENSION_SOURCES})
    string(REPLACE ".cpp" "" name ${src})
    add_extension_example(${name} ${src})
endforeach()

# ============================================================
# 导纳控制示例（使用 omp 自主导纳，全平台可用）
# ============================================================
foreach(src ${ADMITTANCE_SOURCES})
    string(REPLACE ".cpp" "" name ${src})
    add_admittance_example(${name} ${src})
endforeach()

# ============================================================
# 安装规则 — 整目录拷贝（所有示例源码 + 构建配置无条件进分发包）
# 不使用按源文件列表安装：ex_teleop_f710_servoj.cpp 等编译依赖 SDL2 的示例
# 若随打包机环境条件性进包，目标机装/未装 SDL2 时会产生配置不一致
# ============================================================
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
    DESTINATION examples/cpp
    PATTERN "build" EXCLUDE        # 排除本机构建产物
)
