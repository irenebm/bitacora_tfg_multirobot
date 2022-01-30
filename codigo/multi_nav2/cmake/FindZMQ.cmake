# - Try to find ZMQ
# Once done this will define
#
#  ZMQ_FOUND - system has ZMQ
#  ZMQ_INCLUDE_DIRS - the ZMQ include directory
#  ZMQ_LIBRARIES - Link these to use ZMQ
#  ZMQ_DEFINITIONS - Compiler switches required for using ZMQ
#
# Copyright 2021 Irene Bandera Moreno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(ZMQ_LIBRARIES AND ZMQ_INCLUDE_DIRS)
  # in cache already
  set(ZMQ_FOUND TRUE)
else()
  # else(ZMQ_LIBRARIES AND ZMQ_INCLUDE_DIRS)

  find_path(ZMQ_INCLUDE_DIR
    NAMES
      zmq.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  find_library(ZMQ_LIBRARY
    NAMES
      zmq
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(ZMQ_INCLUDE_DIRS
    ${ZMQ_INCLUDE_DIR}
  )

  if(ZMQ_LIBRARY)
    set(ZMQ_LIBRARIES
        ${ZMQ_LIBRARIES}
        ${ZMQ_LIBRARY}
    )
  endif()

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(ZMQ DEFAULT_MSG ZMQ_LIBRARIES ZMQ_INCLUDE_DIRS)

  # show the ZMQ_INCLUDE_DIRS and ZMQ_LIBRARIES variables only in the advanced view
  mark_as_advanced(ZMQ_INCLUDE_DIRS ZMQ_LIBRARIES)

endif()

