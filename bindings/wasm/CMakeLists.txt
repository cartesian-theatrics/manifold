# Copyright 2022 The Manifold Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

add_executable(manifoldjs bindings.cpp mesh_utils.cpp)
target_include_directories(manifoldjs PRIVATE ${PROJECT_SOURCE_DIR}/bindings/java/src/main/cpp/manifold3d)
option(MANIFOLD_JS_ESM "Emit an ES module loader" ON)
target_compile_options(manifoldjs PRIVATE -fexceptions)

set_source_files_properties(
  bindings.cpp
  PROPERTIES OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/bindings.js
)
target_link_libraries(
  manifoldjs
  PRIVATE manifold $<$<BOOL:${MANIFOLD_PAR}>:TBB::tbb>
)
target_compile_options(manifoldjs PRIVATE ${MANIFOLD_FLAGS})
target_link_options(
  manifoldjs
  PUBLIC
    --pre-js
    ${CMAKE_CURRENT_SOURCE_DIR}/bindings.js
    --bind
    -sALLOW_TABLE_GROWTH=1
    -sEXPORTED_RUNTIME_METHODS=addFunction,removeFunction,FS
    -sMODULARIZE=1
    -sEXPORT_ES6=$<BOOL:${MANIFOLD_JS_ESM}>
    -sEXPORT_NAME=ManifoldModule
    --no-entry
    -fexceptions
    -sDISABLE_EXCEPTION_CATCHING=0
    -sSTACK_SIZE=8MB
)

set_target_properties(manifoldjs PROPERTIES OUTPUT_NAME "manifold")

# ensure that interface files are copied over when modified
add_custom_target(
  js_deps
  ALL
  DEPENDS manifoldjs ${CMAKE_CURRENT_SOURCE_DIR}/manifold*.d.ts
)

add_custom_command(
  TARGET js_deps
  POST_BUILD
  # copy WASM build back here for publishing to npm
  COMMAND
    ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/manifold.*
    ${CMAKE_CURRENT_SOURCE_DIR}
)
