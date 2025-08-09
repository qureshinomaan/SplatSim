# - Config file for the ur_rtde package

# Compute paths
get_filename_component(URRTDE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET ur_rtde)
  include("${URRTDE_CMAKE_DIR}/ur_rtdeTargets.cmake")
endif()

include("${URRTDE_CMAKE_DIR}/ur_rtdeBuildConfig.cmake")
if(IS_WINDOWS_INSTALLER)
  cmake_minimum_required(VERSION 3.5)
  #find static filepath properties
  get_target_property(I_DIR ur_rtde::rtde INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(L_LIBS ur_rtde::rtde INTERFACE_LINK_LIBRARIES)


  #If use internal boost
  if(EXISTS "${URRTDE_CMAKE_DIR}/../../../include/ext")

    #replace boost path
    if("${RTDE_BOOST_LIBRARY_DIR}" STREQUAL "" ) 
      string(REPLACE "${RTDE_BOOST_INCLUDE_DIRS}" "${URRTDE_CMAKE_DIR}/../../../include/ext" I_DIR "${I_DIR}")
      file(GLOB L_LIBS ${URRTDE_CMAKE_DIR}/../../libboost*.lib)

      set_target_properties(ur_rtde::rtde PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${I_DIR}"
        INTERFACE_LINK_LIBRARIES  "${L_LIBS}"
      )
    else()
      string(REPLACE "${RTDE_BOOST_INCLUDE_DIRS}" "${URRTDE_CMAKE_DIR}/../../../include/ext" I_DIR "${I_DIR}")
      string(REPLACE "${RTDE_BOOST_LIBRARY_DIR}" "${URRTDE_CMAKE_DIR}/../.." L_LIBS "${L_LIBS}")
      set_target_properties(ur_rtde::rtde PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES "${I_DIR}"
          INTERFACE_LINK_LIBRARIES  "${L_LIBS}"
      )
    endif()

  #if use external boost
  else()
    set(Boost_USE_STATIC_LIBS ON)
    find_package(Boost REQUIRED COMPONENTS system thread date_time)
    set(tmp)
    foreach(dir ${I_DIR})
      if(NOT ("${dir}" MATCHES "/boost.*"))
        list(APPEND tmp ${dir})
      endif()
    endforeach()
    set(I_DIR ${tmp})

    set(tmp)
    foreach(lib ${L_LIBS})
      if(NOT ("${lib}" MATCHES "/boost.*"))
        list(APPEND tmp ${lib})
      endif()
    endforeach()
    set(L_LIBS ${tmp})

    
    set_target_properties(ur_rtde::rtde PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${I_DIR}"
      INTERFACE_LINK_LIBRARIES  "${L_LIBS}"
    )
    target_include_directories(ur_rtde::rtde INTERFACE ${Boost_INCLUDE_DIRS})
    target_link_libraries(ur_rtde::rtde INTERFACE
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_THREAD_LIBRARY}
      ${Boost_DATE_TIME_LIBRARY}
    )
    
    unset(Boost_USE_STATIC_LIBS)
  endif()
elseif ( "${RTDE_BOOST_LIBRARY_DIR}" STREQUAL "" )
  find_package(Boost REQUIRED COMPONENTS system thread)
endif()
