add_executable(populating_ades
	src/populating_ades.cpp)

add_executable(populating_db
	src/populating_db.cpp)

add_executable(save_load_ades
	src/save_load_ades.cpp)

target_link_libraries(populating_ades
	${LIBRARY_NAME})

target_link_libraries(populating_db
	${LIBRARY_NAME})

target_link_libraries(save_load_ades
       	${LIBRARY_NAME})
