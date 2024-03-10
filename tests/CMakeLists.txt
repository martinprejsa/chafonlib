add_executable(tests commands.c)

target_link_libraries(tests PRIVATE chafon-reader)
target_include_directories(tests PRIVATE ../include)

add_test(
  NAME tests
  COMMAND $<TARGET_FILE:tests>
)