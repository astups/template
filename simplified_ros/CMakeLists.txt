#****************************************************************************/
#                            ASTUPS – PROJECT_NAME                          */
#****************************************************************************/
#
#Author : Raphaël Lallement - raphael.lallement [at] laposte [dot] net
#Creation Date : 2015
#License : BSD-3-Clause

cmake_minimum_required(VERSION 2.8)
project(simplified_ros CXX)

set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "5")
set(CPACK_PACKAGE_VERSION_PATCH "0")

set(APP_VERSION "${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
set(APP_ASSOCIATED_TAG "v${APP_VERSION}")

file(GLOB template_headers RELATIVE ${PROJECT_SOURCE_DIR} *.hpp)
install(FILES ${template_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/astups/${PROJECT_NAME})

configure_file(${PROJECT_SOURCE_DIR}/scripts/Findsimplified_ros.cmake.in ${CMAKE_BINARY_DIR}/Findsimplified_ros.cmake @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/Findsimplified_ros.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cmake/Modules)
