find_package(Doxygen REQUIRED COMPONENTS dot)

#get_target_property(DOXY_DOT_PATH Doxygen::dot IMPORTED_LOCATION)
#get_filename_component(DOXY_DOT_PATH "${DOXY_DOT_PATH}" DIRECTORY)
#file(TO_NATIVE_PATH "${DOXY_DOT_PATH}" DOXY_DOT_PATH)

set(DOC_SRC "${PROJECT_SOURCE_DIR}/doc")
set(DOC_DEST "${PROJECT_BINARY_DIR}/doc/${DOC}")
set(DOC_TMP "${PROJECT_BINARY_DIR}/tmp/doc")

set(DOXY_BUILD_LATEX "NO")
if (${WERROR})
	set(DOXY_WARN_AS_ERR "YES")
else()
	set(DOXY_WARN_AS_ERR "NO")
endif()

# if the target is doxygen build only doxygen HTML
if(DOC STREQUAL "doxygen")
	set(DOXY_BUILD_HTML "YES")
	set(DOXY_BUILD_XML "NO")
	set(DOXY_OUTPUT_DIR "${DOC_DEST}")
# otherwise generate only XML and configure sphinx
else()
	if(DOC STREQUAL "html")
		set(EXTRA_PYTHON_MODULES "sphinx_rtd_theme")
	else()
		unset(EXTRA_PYTHON_MODULES)
	endif()

	set(DOXY_BUILD_HTML "NO")
	set(DOXY_BUILD_XML "YES")
	set(DOXY_OUTPUT_DIR "${DOC_TMP}/doxygen")

	# https://git.mel.vin/template/c/issues/50
	# https://bugzilla.gnome.org/show_bug.cgi?id=792676
	set(DOXY_WARN_AS_ERR "NO")
	
	find_package(PythonInterp 3.6 REQUIRED)
	include(find_python_module)

	# do not change the ENV, doesn't work with add_custom_target
	# https://cmake.org/Bug/view.php?id=5145
	if(ENV{PYTHONPATH})
		set(PYTHONPATH "$ENV{PYTHONPATH}")
	else()
		# PYTHONPATH disables defaults, manually append them
		execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
			"import sys; sys.stdout.write(':'.join(sys.path))"
			OUTPUT_VARIABLE PYTHONPATH)
	endif()

	foreach(module
			breathe
			sphinx
			${EXTRA_PYTHON_MODULES})
		string(TOUPPER "${module}" module_upper)
		find_python_module("${module}" "${module_upper}")
		set(PYTHONPATH "${${module_upper}_PATH}:${PYTHONPATH}")
	endforeach(module)

	if(${WERROR})
		set(SPHINX_WERROR "-W")
	else()
		unset(SPHINX_WERROR)
	endif()

	configure_file(
		"${DOC_SRC}/conf.py.in"
		"${DOC_TMP}/sphinx/conf.py"
		@ONLY)
endif()

configure_file(
	"${DOC_SRC}/doxyfile.in"
	"${DOC_TMP}/doxygen/doxyfile"
	@ONLY)

if(DOC STREQUAL "doxygen")
	add_custom_target(doc
		COMMAND Doxygen::doxygen "${DOC_TMP}/doxygen/doxyfile"
		COMMENT "Generating HTML API documentation with Doxygen")

	add_custom_command(TARGET doc POST_BUILD
		COMMAND ;
		COMMENT "Output at ${DOC_DEST}/html/index.html")
else()
	add_custom_target(doc
		COMMAND "${CMAKE_COMMAND}" -E echo
			"Generating Doxygen XML API documentation"
		COMMAND doxygen "${DOC_TMP}/doxygen/doxyfile"

		COMMAND "${CMAKE_COMMAND}" -E echo
			"Generating sphinx ${DOC} documentation"
		COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${PYTHONPATH}"
			"${PYTHON_EXECUTABLE}" -m sphinx -b "${DOC}"
			-d "${DOC_TMP}/sphinx/doctrees" -j 4
			-c "${DOC_TMP}/sphinx" -q ${SPHINX_WERROR}
			"${DOC_SRC}" "${DOC_DEST}")

	add_custom_command(TARGET doc POST_BUILD
		COMMAND ;
		COMMENT "Output at ${DOC_DEST}")
endif()
