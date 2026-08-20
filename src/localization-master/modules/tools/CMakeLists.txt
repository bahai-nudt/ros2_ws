# Tools: 通用融合工具（时间线归并 + 名义状态推进循环），只依赖 core 接口。
# Iron rule: tools 不得感知具体传感器/优化器，只经 core 的抽象接缝驱动融合。

add_library(msf_tools
    src/nominal_advance.cpp
)

target_include_directories(msf_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(msf_tools PUBLIC msf_core)

msf_apply_compiler_options(msf_tools)
msf_module(msf_tools DEPS msf_core)
