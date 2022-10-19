cmake_minimum_required (VERSION 3.5)
project(carla_localization)

#add_subdirectory(src)
#set (EXTRA_LIBS carla_localization_lib) 


file(GLOB SOURCES carla_localization.cpp)

add_executable(carla_localization ${SOURCES})

target_include_directories(carla_localization PRIVATE 
                    ${_ADAS_PROJECT_DIR_}
                    ${_ADAS_PROJECT_DIR_}/adas_dds/node/carla_simulator/carla_localization
                    ${_ADAS_PROJECT_DIR_}/release/build/
                    ${CMAKE_BINARY_DIR}
                    ${PROTOBUF_INCLUDE_DIR}
                    ${FASTRTPS_INCLUDE_DIR}
                    ${CERES_INCLUDE_DIR}
                    ${GLOG_INCLUDE_DIR}
                    ${GFLAGS_INCLUDE_DIR}
                    ${ADAS_DDS_INCLUDE_DIR}
                    /usr/include/jsoncpp/
                )

target_link_libraries(carla_localization
                        adas_dds_convertor
                        ${_ADAS_LIBRARIES_}
                        ${CERES_LIBRARY_DIR}
			#${EXTRA_LIBS}
                        ${ADAS_DDS_LIBRARIES} -L${ADAS_DDS_LIBRARY_DIR}
                        /usr/lib/x86_64-linux-gnu/
			
                    )

