# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-src"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-build"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/tmp"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/src/dear_imgui-populate-stamp"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/src"
  "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/src/dear_imgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/src/dear_imgui-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/mnt/c/Users/신정호/GG_GUI/build/_deps/dear_imgui-subbuild/dear_imgui-populate-prefix/src/dear_imgui-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
