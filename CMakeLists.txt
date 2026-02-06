cmake_minimum_required(VERSION 3.10)
project(ALPR_C_Prj)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 🔹 Add project base path as macro (so you can use PROJECT_DIR in code)
add_definitions(-DPROJECT_DIR="${CMAKE_SOURCE_DIR}")

# 🔹 Include SDK headers
include_directories(${CMAKE_SOURCE_DIR}/ultimateALPR/c++/)

# 🔹 Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 🔹 Find nlohmann/json
find_package(nlohmann_json 3.2.0 REQUIRED)

# 🔹 Find curl
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIRS})

# 🔹 🔹 Explicitly link pthread manually 🔹 🔹
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
include_directories(${CMAKE_THREAD_INCLUDE_DIRS})


# 🔹 Add executable
add_executable(ALPRTest main.cpp)

# 🔹 Link libraries
target_link_libraries(ALPRTest
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    ${CMAKE_SOURCE_DIR}/ultimateALPR/binaries/jetson/aarch64/libultimate_alpr-sdk.so
    pthread
)

# 🔹 (Optional) Set RPATH so .so is found at runtime
set_target_properties(ALPRTest PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/ultimateALPR/binaries/jetson/aarch64"
)


# 🔹 ADDED: Add executable for License Key Generator
add_executable(LicenseKeyGen license_keygen.cpp)

# 🔹 ADDED: Link libraries for LicenseKeyGen (same as ALPRTest)
target_link_libraries(LicenseKeyGen
    ${OpenCV_LIBS}
    ${CURL_LIBRARIES}
    nlohmann_json::nlohmann_json
    ${CMAKE_SOURCE_DIR}/ultimateALPR/binaries/jetson/aarch64/libultimate_alpr-sdk.so
    pthread
)

# 🔹 ADDED: Set RPATH for LicenseKeyGen as well
set_target_properties(LicenseKeyGen PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/ultimateALPR/binaries/jetson/aarch64"
)
