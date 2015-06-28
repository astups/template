###############################################################################
#                         ASTUPS – libvelocity_control                        #
###############################################################################
#
# Author : Raphaël Lallement - raphael [dot] lallement [at] laposte [dot] net
#	Julien Lechalupé - lechalupe [dot] julien [at] gmail [dot] com
# Creation Date : 28/06/2015
# License : BSD-3-Clause
#


cmake_minimum_required(VERSION 2.8)
project(libvelocity_control CXX C)

file(GLOB libvelocity_control_headers RELATIVE ${PROJECT_SOURCE_DIR} *.hpp)

#Install the headers and the tools to find the library
install(FILES ${libvelocity_control_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/astups/${PROJECT_NAME})

configure_file(${PROJECT_SOURCE_DIR}/Findlibvelocity_control.cmake.in ${CMAKE_BINARY_DIR}/Findlibvelocity_control.cmake @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/Findlibvelocity_control.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake/Modules)

configure_file(${PROJECT_SOURCE_DIR}/libvelocity_control.pc.in ${CMAKE_BINARY_DIR}/libvelocity_control.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/libvelocity_control.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

#Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
	#An option for the documentation is to generate graph (for dependencies, ...)
	#To do so it needs Graphviz/DOT, so it is necessary to detect it
	find_package(PkgConfig)
	if(PKG_CONFIG_FOUND)
		pkg_check_modules(GVC libgvc)
		#If Graphviz is found, then a flag "HAVE_DOT" is created, it is used in the Doxyfile.in
		set(HAVE_DOT "YES")
	endif(PKG_CONFIG_FOUND)

	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)
