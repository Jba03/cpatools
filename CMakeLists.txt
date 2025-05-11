project(cpatools)
add_library(cpatools SHARED cpatools.cc cpatools.hh)
set_property(TARGET cpatools PROPERTY CXX_STANDARD 20)

option(CPA_VERSION "Desired engine version" OFF)

if (NOT CPA_VERSION)
  message(FATAL_ERROR "CPA_VERSION not set!")
endif()

target_compile_options(cpatools PRIVATE -DCPA_VERSION=${CPA_VERSION})