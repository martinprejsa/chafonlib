add_executable(tests commands.c)
target_link_libraries(tests PRIVATE chafon-reader)

add_test(
  NAME tests
  COMMAND $<TARGET_FILE:tests>
)