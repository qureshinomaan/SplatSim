cmake_minimum_required(VERSION 3.5)
if (POLICY CMP0074)
	cmake_policy(SET CMP0074 NEW)
endif()
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# options
option(CODE "Build primary libraries" ON)
option(EXAMPLES "Build example executables." ON)
option(PYTHON_BINDINGS "Build python bindings for libraries" ON)
option(USE_WERROR "Use more strict compile flags" OFF)
option(WINDOWS_INSTALLER "Use this to generate a windows installer" OFF)
option(BUILD_STATIC "Use this option to build the library STATIC" OFF)

set(BUILD_TYPE SHARED)
if(BUILD_STATIC)
	set(BUILD_TYPE STATIC)
endif()

if(NOT DEFINED DOC)
	set(DOC OFF CACHE STRING
			"The documentation type to generate." FORCE)
endif()

# disable base languages
unset(PROJECT_LANGUAGES)

if(${CODE})
	set(PROJECT_LANGUAGES ${PROJECT_LANGUAGES} C CXX)
	set(CMAKE_CXX_STANDARD 11)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)

	if(${USE_WERROR} AND NOT DEFINED MSVC)
		set(WALL "-Wall -Wno-strict-aliasing -Wno-unused-function -Wno-pragmas")
		set(WERROR "-Werror")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WALL} ${WERROR}")
		message(STATUS "CompileFlags: ${CMAKE_CXX_FLAGS}")
	endif()
	if(DEFINED WIN32)
		ADD_DEFINITIONS(-D_WIN32_WINNT=0x0A00)
		ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	endif()

endif()

project(ur_rtde VERSION 1.5.7 LANGUAGES ${PROJECT_LANGUAGES})
set(PROJECT_VERSION_SUFFIX "") # alpha/beta/rc, e.g. "-rc0"
set(PROJECT_VERSION "${PROJECT_VERSION}${PROJECT_VERSION_SUFFIX}")
set(PROJECT_AUTHOR "Anders Prier Lindvig")
set(PROJECT_COPYRIGHT "2023")
set(PROJECT_MAIL "anpl@mmmi.sdu.dk")

if(${CODE})
	set(Boost_USE_MULTITHREAD ON)
	if(WINDOWS_INSTALLER)
		set(Boost_USE_STATIC_LIBS ON)
	endif()
	# Check if the conan file exist to find the dependencies
	if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
		include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
		conan_basic_setup(NO_OUTPUT_DIRS KEEP_RPATHS SKIP_STD TARGETS)
		set(Boost_USE_STATIC_LIBS ON)
		set(Boost_ADDITIONAL_VERSIONS 1.75.0)
	endif()
	find_package(Boost REQUIRED COMPONENTS system thread program_options)
	find_package(Threads)

	if(${PYTHON_BINDINGS})
		# This is required to find python site-packages for installation of ur_rtde python bindings.
		if(NOT PYBIND11_PYTHON_VERSION)
			set(PYBIND11_PYTHON_VERSION "" CACHE STRING "Python version to use for compiling modules")
		endif()
		set(Python_ADDITIONAL_VERSIONS 3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4)
		find_package(PythonLibsNew ${PYBIND11_PYTHON_VERSION} REQUIRED)

		if(${CMAKE_VERSION} VERSION_LESS 3.11)
			# pybind11 must be found from submodule
			add_subdirectory(pybind11)
		else()
			# Get pybind11 with FetchContent
			include(FetchContent)

			FetchContent_Declare(
					pybind11-src
					GIT_REPOSITORY https://github.com/pybind/pybind11.git
					GIT_TAG        v2.10.1
			)
			if(${CMAKE_VERSION} VERSION_LESS 3.14)
				macro(FetchContent_MakeAvailable NAME)
					FetchContent_GetProperties(${NAME})
					if(NOT ${NAME}_POPULATED)
						FetchContent_Populate(${NAME})
						add_subdirectory(${${NAME}_SOURCE_DIR} ${${NAME}_BINARY_DIR})
					endif()
				endmacro()
			endif()
			FetchContent_MakeAvailable(pybind11-src)
		endif()

	endif()
endif()

if(DEFINED WIN32)
	set(LIB_SOURCE_FILES
			src/robot_state.cpp
			src/rtde.cpp
			src/dashboard_client.cpp
			src/dashboard_enums.cpp
			src/script_client.cpp
			src/rtde_control_interface.cpp
			src/rtde_receive_interface.cpp
			src/rtde_io_interface.cpp
			src/robotiq_gripper.cpp)

	set(LIB_HEADER_FILES
			include/ur_rtde/rtde.h
			include/ur_rtde/rtde_utility.h
			include/ur_rtde/dashboard_enums.h
			include/ur_rtde/dashboard_client.h
			include/ur_rtde/robot_state.h
			include/ur_rtde/script_client.h
			include/ur_rtde/rtde_control_interface.h
			include/ur_rtde/rtde_control_interface_doc.h
			include/ur_rtde/rtde_receive_interface.h
			include/ur_rtde/rtde_receive_interface_doc.h
			include/ur_rtde/rtde_io_interface.h
			include/ur_rtde/rtde_io_interface_doc.h
			include/ur_rtde/robotiq_gripper.h)
else()
	set(LIB_SOURCE_FILES
			src/robot_state.cpp
			src/rtde.cpp
			src/dashboard_client.cpp
			src/dashboard_enums.cpp
			src/script_client.cpp
			src/rtde_control_interface.cpp
			src/rtde_receive_interface.cpp
			src/rtde_io_interface.cpp
			src/robotiq_gripper.cpp
			src/urcl/script_sender.cpp
			src/urcl/tcp_server.cpp
			src/urcl/log.cpp
			src/urcl/default_log_handler.cpp
            )

	set(LIB_HEADER_FILES
			include/ur_rtde/rtde.h
			include/ur_rtde/rtde_utility.h
			include/ur_rtde/dashboard_enums.h
			include/ur_rtde/dashboard_client.h
			include/ur_rtde/robot_state.h
			include/ur_rtde/script_client.h
			include/ur_rtde/rtde_control_interface.h
			include/ur_rtde/rtde_control_interface_doc.h
			include/ur_rtde/rtde_receive_interface.h
			include/ur_rtde/rtde_receive_interface_doc.h
			include/ur_rtde/rtde_io_interface.h
			include/ur_rtde/rtde_io_interface_doc.h
			include/ur_rtde/robotiq_gripper.h)

	set(LIB_URCL_HEADER_FILES
			include/urcl/log.h
			include/urcl/default_log_handler.h
			include/urcl/script_sender.h
			include/urcl/tcp_server.h
			include/urcl/tcp_socket.h)

endif()

if(NOT DOC STREQUAL "OFF")
	add_subdirectory(doc)
endif()

if(${CODE})
	include(GenerateExportHeader)

	# Write the rtde_control script to a header file
	file(READ scripts/rtde_control.script RTDE_CONTROL_SCRIPT)
	set(header "const std::string NEW_LINE= \"\\n\"; const std::string QUOTATION = \"\\\"\"; std::string UR_SCRIPT = \"")
	string(REGEX REPLACE "\"" "\"+QUOTATION+\"" FILEVAR ${RTDE_CONTROL_SCRIPT})
	string(REGEX REPLACE "\\\n" "\" + NEW_LINE  + \n\"" FILEVAR ${FILEVAR})
	set(end "\";")
	file(WRITE include/ur_rtde/rtde_control_script.h "${header}${FILEVAR}${end}")

	add_library(rtde ${BUILD_TYPE} ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} ${LIB_URCL_HEADER_FILES})
	target_include_directories(rtde
			PUBLIC ${Boost_INCLUDE_DIRS}
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>)
	target_link_libraries(rtde PUBLIC
			${Boost_SYSTEM_LIBRARY}
			${Boost_THREAD_LIBRARY}
			${CMAKE_THREAD_LIBS_INIT})
	set_target_properties(rtde PROPERTIES 
			VERSION ${PROJECT_VERSION} 
			SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

	if(${PYTHON_BINDINGS})
		pybind11_add_module(rtde_control SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} ${LIB_URCL_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(rtde_control
				PUBLIC ${Boost_INCLUDE_DIRS}
				$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
				$<INSTALL_INTERFACE:include>
				PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
				PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
				PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(rtde_control PRIVATE
				#${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY}
				${CMAKE_THREAD_LIBS_INIT})
                target_compile_definitions(rtde_control PRIVATE rtde_EXPORTS)

		pybind11_add_module(rtde_receive SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(rtde_receive
			PUBLIC ${Boost_INCLUDE_DIRS}
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(rtde_receive PRIVATE
				#${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY})
                target_compile_definitions(rtde_receive PRIVATE rtde_EXPORTS)

		pybind11_add_module(rtde_io SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(rtde_io
				PUBLIC ${Boost_INCLUDE_DIRS}
				$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
				$<INSTALL_INTERFACE:include>
				PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
				PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
				PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(rtde_io PRIVATE
				#${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY})
		target_compile_definitions(rtde_io PRIVATE rtde_EXPORTS)

		pybind11_add_module(script_client SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(script_client
				PUBLIC ${Boost_INCLUDE_DIRS}
				$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
				$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
				$<INSTALL_INTERFACE:include>
				PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
				PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
				PRIVATE ${PYTHON_INCLUDE_DIRS})
			target_link_libraries(script_client PRIVATE
				#${PYTHON_LIBRARIES}
				${Boost_SYSTEM_LIBRARY}
				${Boost_THREAD_LIBRARY})
			target_compile_definitions(script_client PRIVATE rtde_EXPORTS)


		pybind11_add_module(dashboard_client SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
		target_include_directories(dashboard_client
			PUBLIC ${Boost_INCLUDE_DIRS}
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>
			PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
			PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
			PRIVATE ${PYTHON_INCLUDE_DIRS})
		target_link_libraries(dashboard_client PRIVATE
			#${PYTHON_LIBRARIES}
			${Boost_SYSTEM_LIBRARY}
			${Boost_THREAD_LIBRARY})
		target_compile_definitions(dashboard_client PRIVATE rtde_EXPORTS)

	endif()

	if(${EXAMPLES})
		add_executable(servoj_example examples/cpp/servoj_example.cpp)
		target_include_directories(servoj_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(servoj_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(forcemode_example examples/cpp/forcemode_example.cpp)
		target_include_directories(forcemode_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(forcemode_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(speedj_example examples/cpp/speedj_example.cpp)
		target_include_directories(speedj_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(speedj_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(movej_path_with_blend_example examples/cpp/movej_path_with_blend_example.cpp)
		target_include_directories(movej_path_with_blend_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(movej_path_with_blend_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(io_example examples/cpp/io_example.cpp)
		target_include_directories(io_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(io_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(move_async_example examples/cpp/move_async_example.cpp)
		target_include_directories(move_async_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(move_async_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
	
		add_executable(move_path_async_example examples/cpp/move_path_async_example.cpp)
		target_include_directories(move_path_async_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(move_path_async_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(robotiq_gripper_example examples/cpp/robotiq_gripper_example.cpp)
		target_include_directories(robotiq_gripper_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(robotiq_gripper_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(move_until_contact_example examples/cpp/move_until_contact.cpp)
		target_include_directories(move_until_contact_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(move_until_contact_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		add_executable(record_data_example examples/cpp/record_data_example.cpp)
		target_include_directories(record_data_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(record_data_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY})

		add_executable(realtime_control_example examples/cpp/realtime_control_example.cpp)
		target_include_directories(realtime_control_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(realtime_control_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY})

		add_executable(contact_detection_example examples/cpp/contact_detection_example.cpp)
		target_include_directories(contact_detection_example PUBLIC ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
		target_link_libraries(contact_detection_example PRIVATE rtde ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

		if (UNIX)
			find_package(Curses)
			if(Curses_FOUND)
				add_executable(jog_example examples/cpp/jog_example.cpp)
				target_include_directories(jog_example PUBLIC ${CURSES_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
				target_link_libraries(jog_example PRIVATE rtde ${CURSES_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
				set_target_properties(jog_example
					PROPERTIES
					RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
					)
				add_executable(ur_rtde::jog_example ALIAS jog_example)
			endif()
		endif (UNIX)

		set_target_properties(servoj_example forcemode_example speedj_example movej_path_with_blend_example io_example move_async_example move_path_async_example robotiq_gripper_example move_until_contact_example record_data_example realtime_control_example contact_detection_example
				PROPERTIES
				RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
				)
	endif()

	generate_export_header(rtde EXPORT_FILE_NAME ${CMAKE_CURRENT_SOURCE_DIR}/include/ur_rtde/rtde_export.h)

	add_library(ur_rtde::rtde ALIAS rtde)

	if(${EXAMPLES})
		add_executable(ur_rtde::servoj_example ALIAS servoj_example)
		add_executable(ur_rtde::forcemode_example ALIAS forcemode_example)
		add_executable(ur_rtde::speedj_example ALIAS speedj_example)
		add_executable(ur_rtde::movej_path_with_blend_example ALIAS movej_path_with_blend_example)
		add_executable(ur_rtde::io_example ALIAS io_example)
		add_executable(ur_rtde::move_async_example ALIAS move_async_example)
		add_executable(ur_rtde::move_path_async_example ALIAS move_path_async_example)
		add_executable(ur_rtde::robotiq_gripper_example ALIAS robotiq_gripper_example)
		add_executable(ur_rtde::move_until_contact_example ALIAS move_until_contact_example)
		add_executable(ur_rtde::record_data_example ALIAS record_data_example)
		add_executable(ur_rtde::realtime_control_example ALIAS realtime_control_example)
	endif()

	if (CMAKE_COMPILER_IS_GNUCXX)
		# Give error in Linux if library is not fully linked (like Windows)
		set_target_properties(rtde PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	endif()

	if(NOT DEFINED LIB_INSTALL_DIR)
		set(LIB_INSTALL_DIR "lib")
	endif()
	if(NOT DEFINED BIN_INSTALL_DIR)
		set(BIN_INSTALL_DIR "bin")
	endif()
	if(NOT DEFINED INCLUDE_INSTALL_DIR)
		set(INCLUDE_INSTALL_DIR "include")
	else()
		set(INCLUDE_INSTALL_DIR "${INCLUDE_INSTALL_DIR}")
	endif()

	install(TARGETS rtde
			EXPORT RTDETargets
			LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT ur_rtde_lib
			ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT ur_rtde_lib
			RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT ur_rtde_lib
			INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
			)
	if(${PYTHON_BINDINGS})
		install(TARGETS rtde_control rtde_receive rtde_io dashboard_client script_client
				DESTINATION ${PYTHON_SITE_PACKAGES}
				COMPONENT ur_rtde-python)
	endif()

	# Install ur_rtde headers
	install(
			FILES
			${LIB_HEADER_FILES}
			"${CMAKE_CURRENT_SOURCE_DIR}/include/ur_rtde/rtde_export.h"
			DESTINATION ${INCLUDE_INSTALL_DIR}/ur_rtde
			COMPONENT ur_rtde_dev
	)

	# Install urcl specific headers
	install(
			FILES
			${LIB_URCL_HEADER_FILES}
			DESTINATION ${INCLUDE_INSTALL_DIR}/urcl
			COMPONENT ur_rtde_dev
	)

	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfigVersion.cmake"
		VERSION ${PROJECT_VERSION}
		COMPATIBILITY AnyNewerVersion
	)
	export(EXPORT RTDETargets
		FILE "${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeTargets.cmake"
		NAMESPACE ur_rtde::
	)
	configure_file(cmake/ur_rtdeConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfig.cmake"
		COPYONLY
	)
	configure_file(cmake/ur_rtdeBuildConfig.cmake.in
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeBuildConfig.cmake" @ONLY
	)
	install(EXPORT RTDETargets
		FILE ur_rtdeTargets.cmake
		NAMESPACE ur_rtde::
		DESTINATION ${LIB_INSTALL_DIR}/cmake/ur_rtde
		COMPONENT ur_rtde_cmake
	)
	install(
		FILES cmake/ur_rtdeConfig.cmake
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeConfigVersion.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/ur_rtde/ur_rtdeBuildConfig.cmake"
		DESTINATION ${LIB_INSTALL_DIR}/cmake/ur_rtde
		COMPONENT ur_rtde_cmake
	)
endif()

macro(getBoostLibraryList output list)
	foreach(s ${list})
		if("${s}" STREQUAL "optimized")

		elseif("${s}" STREQUAL "debug")

		elseif("${s}" MATCHES "NOTFOUND")

		elseif("${s}" STREQUAL "Boost::headers")

		elseif("${s}" STREQUAL "Threads::Threads")

		elseif("${s}" IN_LIST ${output})

		elseif(TARGET ${s})
			get_target_property(LIB ${s} IMPORTED_LOCATION_RELEASE)
			if(LIB)
				list(APPEND ${output} "${LIB}")
			endif()
			

			get_target_property(L_LIBS ${s} INTERFACE_LINK_LIBRARIES)
			getBoostLibraryList(${output} "${L_LIBS}")
		else()
			get_filename_component(_dir "${s}" DIRECTORY)
			get_filename_component(_file "${s}" NAME_WLE)
			string(REGEX MATCH "[a-z]*_[a-z]*" _file ${_file})
			file(GLOB _files "${_dir}/${_file}*lib")
			foreach(file ${_files})
				list(APPEND BOOST_LIBRARIES_INSTALL "${file}")
			endforeach()
		endif()
	endforeach()
endmacro()


if(WINDOWS_INSTALLER)
	set(BOOST_LIBRARIES_INSTALL "")

	getBoostLibraryList(BOOST_LIBRARIES_INSTALL "${Boost_LIBRARIES}")

	# Install external libraries
	install(
		FILES ${BOOST_LIBRARIES_INSTALL}
		DESTINATION ${LIB_INSTALL_DIR}
		COMPONENT boost
	)
	install(
		DIRECTORY "${Boost_INCLUDE_DIR}/boost/asio" "${Boost_INCLUDE_DIR}/boost/config"
		"${Boost_INCLUDE_DIR}/boost/core" "${Boost_INCLUDE_DIR}/boost/date_time" "${Boost_INCLUDE_DIR}/boost/detail"
		"${Boost_INCLUDE_DIR}/boost/exception" "${Boost_INCLUDE_DIR}/boost/mpl" "${Boost_INCLUDE_DIR}/boost/numeric"
		"${Boost_INCLUDE_DIR}/boost/predef" "${Boost_INCLUDE_DIR}/boost/preprocessor" "${Boost_INCLUDE_DIR}/boost/smart_ptr"
		"${Boost_INCLUDE_DIR}/boost/system" "${Boost_INCLUDE_DIR}/boost/type_traits" "${Boost_INCLUDE_DIR}/boost/winapi"
		DESTINATION "${INCLUDE_INSTALL_DIR}/ext/boost/"
		COMPONENT boost
	)
	install(
		FILES "${Boost_INCLUDE_DIR}/boost/assert.hpp" "${Boost_INCLUDE_DIR}/boost/cerrno.hpp" "${Boost_INCLUDE_DIR}/boost/checked_delete.hpp"
		"${Boost_INCLUDE_DIR}/boost/config.hpp" "${Boost_INCLUDE_DIR}/boost/cstdint.hpp" "${Boost_INCLUDE_DIR}/boost/current_function.hpp"
		"${Boost_INCLUDE_DIR}/boost/limits.hpp" "${Boost_INCLUDE_DIR}/boost/operators.hpp" "${Boost_INCLUDE_DIR}/boost/shared_ptr.hpp"
		"${Boost_INCLUDE_DIR}/boost/static_assert.hpp" "${Boost_INCLUDE_DIR}/boost/throw_exception.hpp" "${Boost_INCLUDE_DIR}/boost/type.hpp"
		"${Boost_INCLUDE_DIR}/boost/version.hpp"
		DESTINATION "${INCLUDE_INSTALL_DIR}/ext/boost/"
		COMPONENT boost
	)

    if(DEFINED MSVC)
        set(SLASH "\\\\")

		############################### CPACK setup ######################################### 
        set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
        set(CPACK_PACKAGE_VENDOR "University of Southern Denmark")
        set(
            CPACK_PACKAGE_DESCRIPTION_SUMMARY
            "A C++ interface for sending and receiving data to/from a UR robot using the Real-Time Data Exchange (RTDE) interface of the robot"
        )
        set(CPACK_PACKAGE_HOMEPAGE_URL "https://sdurobotics.gitlab.io/ur_rtde/")
        set(CPACK_PACKAGE_INSTALL_DIRECTORY "ur_rtde")
        set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}${SLASH}LICENSE")
		set(CPACK_PACKAGE_CONTACT "Anders Prier Lindvig (anpl@mmmi.sdu.dk)")
		
		set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
        set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
        set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
        set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

        set(CPACK_PACKAGE_FILE_NAME ${CPACK_PACKAGE_NAME}-install)

		############################### NSIS setup ######################################### 
        set(CPACK_NSIS_DISPLAY_NAME "${PROJECT_NAME}")
        set(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME}")
        set(CPACK_NSIS_HELP_LINK ${CPACK_PACKAGE_HOMEPAGE_URL})
        set(CPACK_NSIS_URL_INFO_ABOUT ${CPACK_PACKAGE_HOMEPAGE_URL})
        #set(
        #    CPACK_NSIS_MUI_ICON
        #    "${PROJECT_SOURCE_DIR}${SLASH}doc${SLASH}_static${SLASH}robots.png" # needs to be 32x32.ico file
        #)
        #set(
        #    CPACK_NSIS_MUI_UNWELCOMEFINISHPAGE_BITMAP
        #    "${PROJECT_SOURCE_DIR}${SLASH}doc${SLASH}_static${SLASH}robots.png"  # needs to be 128x64.bmp file
        #)
		#set(CPACK_NSIS_MUI_WELCOMEFINISHPAGE_BITMAP ${CPACK_NSIS_MUI_UNWELCOMEFINISHPAGE_BITMAP})
		
        set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON) # When installing on a previus
                                                           # instalation ask to uninstall first
        set(CPACK_NSIS_UNINSTALL_NAME "ur_rtde_uninstall")
        set(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})

		# Make sure the system findes the project at installation
		if(BUILD_STATIC)
			set(CPACK_NSIS_MODIFY_PATH OFF) # Add the binary folder to PATH
		else() 
			set(CPACK_NSIS_MODIFY_PATH ON) # Add the binary folder to PATH
		endif()

        set(
            CPACK_NSIS_EXTRA_INSTALL_COMMANDS # let cmake find ur_rtde
            "WriteRegStr HKCU \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}ur_rtde\\\" \\\"Location\\\" \\\"$INSTDIR\\\""
            "WriteRegStr HKLM \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}ur_rtde\\\" \\\"Location\\\" \\\"$INSTDIR\\\""
        )
        set(
            CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS # clean up when uninstalling ur_rtde
            "DeleteRegKey HKCU \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}ur_rtde\\\""
            "DeleteRegKey HKLM \\\"Software${SLASH}Kitware${SLASH}CMake${SLASH}Packages${SLASH}ur_rtde\\\""
        )
        string(
            REPLACE
                ";" "\n" CPACK_NSIS_EXTRA_INSTALL_COMMANDS "${CPACK_NSIS_EXTRA_INSTALL_COMMANDS}"
        )
        string(
            REPLACE
                ";" "\n" CPACK_NSIS_EXTRA_UNSTALL_COMMANDS "${CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS}"
        )

		############################### Package setup ######################################### 
		get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS) # Get all components
		
		include(CPack)
        # Setup Install Groupes ####
		cpack_add_component_group(UR_RTDE 
			DISPLAY_NAME "ur_rtde"
            DESCRIPTION "Libraries for controlling UR robots"
            EXPANDED
            BOLD_TITLE
        )
		cpack_add_component_group(MISC 
			DISPLAY_NAME "Miscellaneous" 
			DESCRIPTION "These are miscellaneous packages"
		)
		cpack_add_component_group(DEP
			DISPLAY_NAME "Dependencies" 
			DESCRIPTION "These are external dependencies for the ur_rtde"
		)
		cpack_add_component_group(DEVEL 
			DISPLAY_NAME "Development Files" 
			DESCRIPTION "These are packages needed when you want to use ur_rtde for development"
        )

        # Setup Install Types ####
        cpack_add_install_type(Full DISPLAY_NAME "Full")

		# Setup Install Components
		cpack_add_component( ur_rtde_lib
			DISPLAY_NAME "DevelFiles"
			DESCRIPTION "This package includes the files necessary to run a program that depends on ur_rtde"
			GROUP UR_RTDE
			DEPENDS ur_rtde_lib
			INSTALL_TYPES Full 
			# DOWNLOADED ARCHIVE_FILE #Name_of_file_to_generate_for_download
		)
		cpack_add_component( ur_rtde_dev
			DISPLAY_NAME "DevelFiles"
			DESCRIPTION "This package includes the files necessary for development of a program that depends on ur_rtde"
			GROUP UR_RTDE
			DEPENDS ur_rtde_lib
			INSTALL_TYPES Full 
			# DOWNLOADED ARCHIVE_FILE #Name_of_file_to_generate_for_download
		)
        cpack_add_component( ur_rtde_cmake
			DISPLAY_NAME "CMakeFiles"
            DESCRIPTION "This package includes the files necessary for CMake to find and configure ur_rtde"
            GROUP UR_RTDE
			DEPENDS ur_rtde_lib ur_rtde_dev
            INSTALL_TYPES Full 
			# DOWNLOADED ARCHIVE_FILE #Name_of_file_to_generate_for_download
		)
		cpack_add_component( boost
			DISPLAY_NAME "Boost"
			DESCRIPTION "This package includes the files ur_rtde needs for runtime and development"
			GROUP DEP
			INSTALL_TYPES Full 
			# DOWNLOADED ARCHIVE_FILE #Name_of_file_to_generate_for_download
		)
	endif()
endif()

