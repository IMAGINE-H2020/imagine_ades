configure_file(config.h.in "${CMAKE_CURRENT_BINARY_DIR}/config.h" @ONLY)

add_library(${LIBRARY_NAME} SHARED
	src/storage/adesdb.cpp
	src/types/ades.cpp
	src/types/motion_sequence.cpp
	src/types/motion_type.cpp)

set_target_properties(${LIBRARY_NAME} PROPERTIES
  PUBLIC_HEADER "include/libades.h;${CMAKE_CURRENT_BINARY_DIR}/config.h")

target_link_libraries(${LIBRARY_NAME}
	boost
	armadillo
	libgp
	mlpack)

install(TARGETS ${LIBRARY_NAME}
  EXPORT ADESTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR} COMPONENT bin
  LIBRARY DESTINATION ${INSTALL_LIB_DIR} COMPONENT lib
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR} COMPONENT lib
  PUBLIC_HEADER DESTINATION ${INSTALL_INCLUDE_DIR}/${LIBRARY_NAME}
    COMPONENT dev)

#Dirty way of copying header infrestructure
install(DIRECTORY include/ DESTINATION ${INSTALL_INCLUDE_DIR}/${LIBRARY_NAME}
        FILES_MATCHING PATTERN "*.h")
