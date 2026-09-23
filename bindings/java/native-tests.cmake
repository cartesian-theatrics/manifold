# Exercise the fork's native appearance and surface mapping extensions alongside
# upstream's core test suite. These do not require Java or Maven.
foreach(NAME model surface_uv)
  add_executable(clj_${NAME}_test ${PROJECT_SOURCE_DIR}/bindings/java/src/test/cpp/${NAME}_test.cpp)
  target_include_directories(clj_${NAME}_test PRIVATE ${PROJECT_SOURCE_DIR}/bindings/java/src/main/cpp/manifold3d)
  target_link_libraries(clj_${NAME}_test PRIVATE manifold)
  add_test(NAME clj_${NAME}_test COMMAND clj_${NAME}_test)
endforeach()
