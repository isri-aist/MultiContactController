if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  function(add_MCC_test NAME)
    add_executable(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} PUBLIC GTest::gtest MultiContactController mc_rtc::mc_rtc_utils)
    gtest_discover_tests(${NAME})
  endfunction()
else()
  function(add_MCC_test NAME)
    catkin_add_gtest(${NAME} src/${NAME}.cpp)
    target_link_libraries(${NAME} MultiContactController mc_rtc::mc_rtc_utils)
  endfunction()
endif()

set(MCC_gtest_list
  TestMathUtils
  TestCommandTypes
  )

foreach(NAME IN LISTS MCC_gtest_list)
  add_MCC_test(${NAME})
endforeach()
