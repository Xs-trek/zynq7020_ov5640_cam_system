cmake_minimum_required(VERSION 3.10)

project(cam_system VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_SKIP_RPATH ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "Install prefix" FORCE)
endif()

include(GNUInstallDirs)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CAM_SYSTEM_ENABLE_WARNINGS "Enable project warning flags" ON)
option(CAM_SYSTEM_WERROR "Treat warnings in first-party code as errors" ON)
option(CAM_SYSTEM_ENABLE_CPU_DETECTORS
    "Build the CPU model detector candidates (ncnn, UltraFace, libfacedetection); the product uses the PL" OFF)
set(CAM_SYSTEM_DEPENDENCY_MODE "EXTERNAL" CACHE STRING
    "Dependency mode: EXTERNAL uses historical ARM install paths, YOCTO uses recipe sysroot packages")
set_property(CACHE CAM_SYSTEM_DEPENDENCY_MODE PROPERTY STRINGS EXTERNAL YOCTO)

set(CAM_SYSTEM_NCNN_ROOT "$ENV{HOME}/ncnn/build-arm/install"
    CACHE PATH "ncnn ARM install prefix")
set(CAM_SYSTEM_JPEGTURBO_ROOT "$ENV{HOME}/workdir/libjpeg-turbo/build-arm/install"
    CACHE PATH "libjpeg-turbo ARM install prefix")
set(CAM_SYSTEM_NCNN_LIBRARY "${CAM_SYSTEM_NCNN_ROOT}/lib/libncnn.a"
    CACHE FILEPATH "ncnn library")
set(CAM_SYSTEM_TURBOJPEG_LIBRARY "${CAM_SYSTEM_JPEGTURBO_ROOT}/lib/libturbojpeg.so"
    CACHE FILEPATH "TurboJPEG library")

function(cam_system_require_path path description)
    if(NOT EXISTS "${path}")
        message(FATAL_ERROR "${description} not found: ${path}")
    endif()
endfunction()

find_package(Threads REQUIRED)

# Build identity.  A source tree exported with git archive carries its commit in cmake/git_commit.txt
# (export-subst, see .gitattributes); a git checkout asks git; otherwise the caller may pass it.
if(NOT DEFINED CAM_SYSTEM_GIT_COMMIT)
    file(STRINGS "${CMAKE_SOURCE_DIR}/cmake/git_commit.txt" _cam_git_commit LIMIT_COUNT 1)
    if(NOT _cam_git_commit MATCHES "^[0-9a-f]+$")
        set(_cam_git_commit "unknown")
        find_package(Git QUIET)
        if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
            execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE _cam_git_head OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE _cam_git_rc ERROR_QUIET)
            if(_cam_git_rc EQUAL 0)
                set(_cam_git_commit "${_cam_git_head}")
                execute_process(COMMAND "${GIT_EXECUTABLE}" diff --quiet HEAD --
                    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                    RESULT_VARIABLE _cam_git_dirty ERROR_QUIET)
                if(NOT _cam_git_dirty EQUAL 0)
                    set(_cam_git_commit "${_cam_git_commit}-dirty")
                endif()
            endif()
        endif()
    endif()
    set(CAM_SYSTEM_GIT_COMMIT "${_cam_git_commit}")
endif()
message(STATUS "cam_system ${PROJECT_VERSION} (${CAM_SYSTEM_GIT_COMMIT})")
configure_file("${CMAKE_SOURCE_DIR}/include/cam_system/version.h.in"
    "${CMAKE_BINARY_DIR}/generated/cam_system/version.h" @ONLY)

if(CAM_SYSTEM_DEPENDENCY_MODE STREQUAL "EXTERNAL")
    cam_system_require_path("${CAM_SYSTEM_JPEGTURBO_ROOT}/include" "libjpeg-turbo include directory")
    cam_system_require_path("${CAM_SYSTEM_TURBOJPEG_LIBRARY}" "TurboJPEG library")
    add_library(TurboJPEG::turbojpeg UNKNOWN IMPORTED)
    set_target_properties(TurboJPEG::turbojpeg PROPERTIES
        IMPORTED_LOCATION "${CAM_SYSTEM_TURBOJPEG_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_JPEGTURBO_ROOT}/include"
    )
    set(CAM_SYSTEM_TURBOJPEG_TARGET TurboJPEG::turbojpeg)

    if(CAM_SYSTEM_ENABLE_CPU_DETECTORS)
        cam_system_require_path("${CAM_SYSTEM_NCNN_ROOT}/include" "ncnn include directory")
        cam_system_require_path("${CAM_SYSTEM_NCNN_LIBRARY}" "ncnn library")
        add_library(ncnn::ncnn STATIC IMPORTED)
        set_target_properties(ncnn::ncnn PROPERTIES
            IMPORTED_LOCATION "${CAM_SYSTEM_NCNN_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_NCNN_ROOT}/include"
        )
        set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
    endif()
elseif(CAM_SYSTEM_DEPENDENCY_MODE STREQUAL "YOCTO")
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(TURBOJPEG QUIET IMPORTED_TARGET libturbojpeg)
    endif()
    if(TARGET PkgConfig::TURBOJPEG)
        set(CAM_SYSTEM_TURBOJPEG_TARGET PkgConfig::TURBOJPEG)
    else()
        find_path(CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR
            NAMES turbojpeg.h
            PATH_SUFFIXES include)
        find_library(CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY
            NAMES turbojpeg)
        cam_system_require_path("${CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR}" "Yocto TurboJPEG include directory")
        cam_system_require_path("${CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY}" "Yocto TurboJPEG library")
        add_library(TurboJPEG::turbojpeg UNKNOWN IMPORTED)
        set_target_properties(TurboJPEG::turbojpeg PROPERTIES
            IMPORTED_LOCATION "${CAM_SYSTEM_YOCTO_TURBOJPEG_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_YOCTO_TURBOJPEG_INCLUDE_DIR}"
        )
        set(CAM_SYSTEM_TURBOJPEG_TARGET TurboJPEG::turbojpeg)
    endif()

    if(CAM_SYSTEM_ENABLE_CPU_DETECTORS)
        find_package(ncnn CONFIG QUIET)
        if(TARGET ncnn)
            set(CAM_SYSTEM_NCNN_TARGET ncnn)
        elseif(TARGET ncnn::ncnn)
            set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
        else()
            find_path(CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR
                NAMES ncnn/net.h
                PATH_SUFFIXES include)
            find_library(CAM_SYSTEM_YOCTO_NCNN_LIBRARY
                NAMES ncnn)
            cam_system_require_path("${CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR}" "Yocto ncnn include directory")
            cam_system_require_path("${CAM_SYSTEM_YOCTO_NCNN_LIBRARY}" "Yocto ncnn library")
            add_library(ncnn::ncnn STATIC IMPORTED)
            set_target_properties(ncnn::ncnn PROPERTIES
                IMPORTED_LOCATION "${CAM_SYSTEM_YOCTO_NCNN_LIBRARY}"
                INTERFACE_INCLUDE_DIRECTORIES "${CAM_SYSTEM_YOCTO_NCNN_INCLUDE_DIR}"
            )
            set(CAM_SYSTEM_NCNN_TARGET ncnn::ncnn)
        endif()
    endif()
else()
    message(FATAL_ERROR "Unsupported CAM_SYSTEM_DEPENDENCY_MODE=${CAM_SYSTEM_DEPENDENCY_MODE}")
endif()

set(CAM_SYSTEM_C_SOURCES
    src/common/log.c
    src/common/app_config.c
    src/common/fps_meter.c
    src/platform/hal_ov5640.c
    src/platform/hal_frmbuf.c
    src/capture/frame_mailbox.c
    src/capture/frame_view_timeline.c
    src/capture/capture.c
    src/capture/control_tick.c
    src/capture/capture_recovery.c
    src/capture/dvp_counts.c
    src/capture/stream_control.c
    src/capture/camera_switch.c
    src/detect/detector_recovery.c
    src/detect/face_decode_fast.c
    src/vision/face_accel_client.c
    third_party/face_accel_ref/ultraface_decode.c
    src/media/jpeg_enc.c
    src/media/jpeg_worker.c
    src/media/stream_rate.c
    src/network/net_server.c
    src/network/http_parse.c
    src/network/mjpeg_framing.c
    src/network/status_json.c
    src/network/ws_protocol.c
    src/network/cmd_handler.c
    src/control/tracker_servo.c
    src/control/track_trace.c
    src/control/tracker.c
    third_party/cjson/cJSON.c
)

set(CAM_SYSTEM_CXX_SOURCES
    src/app/main.cpp
    src/detect/detector.cpp
    src/detect/backend_face_accel.cpp
    src/vision/face_temporal_tracker.cpp
)

# Third-party sources are built unmodified; their warnings stay visible but never fail the build.
set(CAM_SYSTEM_THIRD_PARTY_SOURCES
    third_party/face_accel_ref/ultraface_decode.c
    third_party/cjson/cJSON.c
)

if(CAM_SYSTEM_ENABLE_CPU_DETECTORS)
    set(CAM_SYSTEM_CPU_DETECTOR_THIRD_PARTY
        third_party/ultraface/UltraFace.cpp
        third_party/libfacedetection/src/facedetectcnn.cpp
        third_party/libfacedetection/src/facedetectcnn-data.cpp
        third_party/libfacedetection/src/facedetectcnn-model.cpp
    )
    list(APPEND CAM_SYSTEM_CXX_SOURCES src/detect/backend_cpu.cpp ${CAM_SYSTEM_CPU_DETECTOR_THIRD_PARTY})
    list(APPEND CAM_SYSTEM_THIRD_PARTY_SOURCES ${CAM_SYSTEM_CPU_DETECTOR_THIRD_PARTY})
endif()

add_executable(cam_system
    ${CAM_SYSTEM_C_SOURCES}
    ${CAM_SYSTEM_CXX_SOURCES}
)

target_include_directories(cam_system
    PRIVATE
        "${CMAKE_BINARY_DIR}/generated"
        "${CMAKE_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/src"
        "${CMAKE_SOURCE_DIR}/third_party"
        "${CMAKE_SOURCE_DIR}/third_party/face_accel_ref"
)

if(CAM_SYSTEM_ENABLE_CPU_DETECTORS)
    target_compile_definitions(cam_system PRIVATE CAM_SYSTEM_ENABLE_CPU_DETECTORS)
    target_include_directories(cam_system
        PRIVATE
            "${CMAKE_SOURCE_DIR}/third_party/ultraface"
            "${CMAKE_SOURCE_DIR}/third_party/libfacedetection/src"
    )
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm|ARM|aarch|AARCH")
        target_compile_definitions(cam_system PRIVATE _ENABLE_NEON)
    endif()
endif()

if(CAM_SYSTEM_ENABLE_WARNINGS)
    target_compile_options(cam_system
        PRIVATE
            -Wall
            -Wextra
            -Wformat=2
            -Wno-unused-parameter
    )
    if(CAM_SYSTEM_WERROR)
        target_compile_options(cam_system PRIVATE -Werror)
        set_source_files_properties(${CAM_SYSTEM_THIRD_PARTY_SOURCES}
            PROPERTIES COMPILE_FLAGS "-Wno-error")
    endif()
endif()

target_link_libraries(cam_system
    PRIVATE
        ${CAM_SYSTEM_NCNN_TARGET}
        ${CAM_SYSTEM_TURBOJPEG_TARGET}
        ssl
        crypto
        Threads::Threads
        dl
        m
)

install(TARGETS cam_system RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES config/cam_system.conf DESTINATION ${CMAKE_INSTALL_FULL_SYSCONFDIR})
if(CAM_SYSTEM_ENABLE_CPU_DETECTORS)
    install(DIRECTORY assets/models/
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cam-system/models
        PATTERN "face_accel" EXCLUDE
    )
endif()
