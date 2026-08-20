cmake_minimum_required(VERSION 3.16)

# Portable repository quality gate. It intentionally depends only on CMake,
# Git and the compiler already required by the project.
get_filename_component(QUALITY_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

if(NOT DEFINED QUALITY_CONFIG OR "${QUALITY_CONFIG}" STREQUAL "")
    set(QUALITY_CONFIG Debug)
endif()

if(NOT DEFINED QUALITY_BUILD_DIR OR "${QUALITY_BUILD_DIR}" STREQUAL "")
    set(QUALITY_BUILD_DIR "${QUALITY_SOURCE_DIR}/build")
elseif(NOT IS_ABSOLUTE "${QUALITY_BUILD_DIR}")
    get_filename_component(
        QUALITY_BUILD_DIR "${QUALITY_SOURCE_DIR}/${QUALITY_BUILD_DIR}" ABSOLUTE)
else()
    get_filename_component(QUALITY_BUILD_DIR "${QUALITY_BUILD_DIR}" ABSOLUTE)
endif()

file(TO_CMAKE_PATH "${QUALITY_SOURCE_DIR}" QUALITY_SOURCE_DIR_NORMALIZED)
file(TO_CMAKE_PATH "${QUALITY_BUILD_DIR}" QUALITY_BUILD_DIR_NORMALIZED)
file(RELATIVE_PATH QUALITY_BUILD_RELATIVE
    "${QUALITY_SOURCE_DIR_NORMALIZED}" "${QUALITY_BUILD_DIR_NORMALIZED}")

if("${QUALITY_BUILD_RELATIVE}" STREQUAL "" OR
   "${QUALITY_BUILD_RELATIVE}" STREQUAL "." OR
   "${QUALITY_BUILD_RELATIVE}" MATCHES "^\\.\\.($|/)")
    message(FATAL_ERROR
        "QUALITY_BUILD_DIR must be a build directory inside the repository: "
        "${QUALITY_BUILD_DIR_NORMALIZED}")
endif()

if(NOT "${QUALITY_BUILD_RELATIVE}" STREQUAL "build" AND
   NOT "${QUALITY_BUILD_RELATIVE}" MATCHES "^build/")
    message(FATAL_ERROR
        "Quality-gate outputs must stay under the canonical build/ directory; got: "
        "${QUALITY_BUILD_RELATIVE}")
endif()

find_program(GIT_EXECUTABLE NAMES git)
if(NOT GIT_EXECUTABLE)
    message(FATAL_ERROR "Git is required to run repository quality checks")
endif()

message(STATUS "[quality] checking whitespace errors")
execute_process(
    COMMAND "${GIT_EXECUTABLE}" diff --check HEAD --
    WORKING_DIRECTORY "${QUALITY_SOURCE_DIR}"
    RESULT_VARIABLE QUALITY_DIFF_RESULT
    OUTPUT_VARIABLE QUALITY_DIFF_OUTPUT
    ERROR_VARIABLE QUALITY_DIFF_ERROR)
if(NOT QUALITY_DIFF_RESULT EQUAL 0)
    message(FATAL_ERROR
        "git diff --check failed:\n${QUALITY_DIFF_OUTPUT}${QUALITY_DIFF_ERROR}")
endif()

message(STATUS "[quality] checking tracked files and machine-local paths")
execute_process(
    COMMAND "${GIT_EXECUTABLE}" ls-files
    WORKING_DIRECTORY "${QUALITY_SOURCE_DIR}"
    RESULT_VARIABLE QUALITY_FILES_RESULT
    OUTPUT_VARIABLE QUALITY_TRACKED_FILES
    ERROR_VARIABLE QUALITY_FILES_ERROR
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT QUALITY_FILES_RESULT EQUAL 0)
    message(FATAL_ERROR "git ls-files failed:\n${QUALITY_FILES_ERROR}")
endif()

string(REPLACE "\r\n" "\n" QUALITY_TRACKED_FILES "${QUALITY_TRACKED_FILES}")
string(REPLACE "\n" ";" QUALITY_TRACKED_FILES "${QUALITY_TRACKED_FILES}")
set(QUALITY_TRACKED_ARTIFACTS "")
set(QUALITY_LOCAL_PATH_FILES "")

foreach(QUALITY_FILE IN LISTS QUALITY_TRACKED_FILES)
    if(QUALITY_FILE STREQUAL "")
        continue()
    endif()

    if(QUALITY_FILE MATCHES "(^|/)build[^/]*/" OR
       QUALITY_FILE MATCHES "(^|/)(CMakeCache\\.txt|cmake_install\\.cmake)$" OR
       QUALITY_FILE MATCHES "\\.(o|obj|a|lib|so|dll|exe|pdb|ilk|log|pyc)$")
        list(APPEND QUALITY_TRACKED_ARTIFACTS "${QUALITY_FILE}")
    endif()

    if(QUALITY_FILE MATCHES "(^|/)CMakeLists\\.txt$" OR
       QUALITY_FILE MATCHES "\\.(c|h|cmake|json|ya?ml|ps1|sh|bat)$")
        file(READ "${QUALITY_SOURCE_DIR}/${QUALITY_FILE}" QUALITY_FILE_CONTENT)
        if(QUALITY_FILE_CONTENT MATCHES
           "(^|[^A-Za-z0-9_])[A-Za-z]:[/\\\\]")
            list(APPEND QUALITY_LOCAL_PATH_FILES "${QUALITY_FILE}")
        endif()
    endif()
endforeach()

if(QUALITY_TRACKED_ARTIFACTS)
    list(JOIN QUALITY_TRACKED_ARTIFACTS "\n  " QUALITY_TRACKED_ARTIFACTS_TEXT)
    message(FATAL_ERROR
        "Generated build artifacts are tracked by Git:\n  "
        "${QUALITY_TRACKED_ARTIFACTS_TEXT}")
endif()

if(QUALITY_LOCAL_PATH_FILES)
    list(JOIN QUALITY_LOCAL_PATH_FILES "\n  " QUALITY_LOCAL_PATH_FILES_TEXT)
    message(FATAL_ERROR
        "Machine-local absolute Windows paths were found in portable sources:\n  "
        "${QUALITY_LOCAL_PATH_FILES_TEXT}")
endif()

if(QUALITY_CHECK_ONLY)
    message(STATUS "QUALITY_GATE_SUCCESS (repository checks only)")
    return()
endif()

set(QUALITY_CONFIGURE_COMMAND
    "${CMAKE_COMMAND}"
    -S "${QUALITY_SOURCE_DIR}"
    -B "${QUALITY_BUILD_DIR}"
    "-DCMAKE_BUILD_TYPE=${QUALITY_CONFIG}"
    -DNAV_WARNINGS_AS_ERRORS=ON)

# A fresh Windows tree commonly has MinGW but no Visual Studio installation.
# Reusing a configured tree must never change its selected generator.
if(WIN32 AND NOT EXISTS "${QUALITY_BUILD_DIR}/CMakeCache.txt")
    find_program(QUALITY_MINGW_MAKE NAMES mingw32-make.exe mingw32-make)
    if(QUALITY_MINGW_MAKE)
        list(APPEND QUALITY_CONFIGURE_COMMAND -G "MinGW Makefiles")
    endif()
endif()

message(STATUS "[quality] configuring ${QUALITY_CONFIG} in ${QUALITY_BUILD_RELATIVE}")
execute_process(
    COMMAND ${QUALITY_CONFIGURE_COMMAND}
    WORKING_DIRECTORY "${QUALITY_SOURCE_DIR}"
    RESULT_VARIABLE QUALITY_CONFIGURE_RESULT)
if(NOT QUALITY_CONFIGURE_RESULT EQUAL 0)
    message(FATAL_ERROR "CMake configure failed with code ${QUALITY_CONFIGURE_RESULT}")
endif()

message(STATUS "[quality] clean-building with warnings as errors")
execute_process(
    COMMAND "${CMAKE_COMMAND}" --build "${QUALITY_BUILD_DIR}"
            --config "${QUALITY_CONFIG}" --clean-first --parallel
    WORKING_DIRECTORY "${QUALITY_SOURCE_DIR}"
    RESULT_VARIABLE QUALITY_BUILD_RESULT)
if(NOT QUALITY_BUILD_RESULT EQUAL 0)
    message(FATAL_ERROR "Build failed with code ${QUALITY_BUILD_RESULT}")
endif()

find_program(CTEST_EXECUTABLE NAMES ctest)
if(NOT CTEST_EXECUTABLE)
    message(FATAL_ERROR "CTest is required to run the regression suite")
endif()
message(STATUS "[quality] running the complete CTest suite")
execute_process(
    COMMAND "${CTEST_EXECUTABLE}" --build-config "${QUALITY_CONFIG}"
            --output-on-failure
    WORKING_DIRECTORY "${QUALITY_BUILD_DIR}"
    RESULT_VARIABLE QUALITY_TEST_RESULT)
if(NOT QUALITY_TEST_RESULT EQUAL 0)
    message(FATAL_ERROR "CTest failed with code ${QUALITY_TEST_RESULT}")
endif()

message(STATUS "QUALITY_GATE_SUCCESS")
