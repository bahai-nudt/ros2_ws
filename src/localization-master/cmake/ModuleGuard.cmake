# 模块依赖护栏：库模块只允许依赖 msf_core 与第三方库。
#
# 用法：
#   msf_module(<name> DEPS <dep...>)   # 在库模块 target 创建后调用：注册 + 校验
#   msf_app(<name>)                   # 组合层（apps / demo / 用户 main）标记，不做约束

function(msf_module name)
    cmake_parse_arguments(ARG "" "" "DEPS" ${ARGN})

    set_property(GLOBAL APPEND PROPERTY MSF_LIB_MODULES ${name})
    get_property(known_modules GLOBAL PROPERTY MSF_LIB_MODULES)

    # 库模块允许依赖的库模块（核心底座）；第三方库不在此列表，天然放行。
    set(allowed_core_modules msf_core)

    foreach(dep ${ARG_DEPS})
        list(FIND known_modules ${dep} dep_is_module)
        if(dep_is_module EQUAL -1)
            continue()
        endif()
        list(FIND allowed_core_modules ${dep} allowed)
        if(allowed EQUAL -1)
            message(FATAL_ERROR
                "[msf_module] ${name} 依赖库模块 ${dep}，违反依赖铁律："
                "库模块之间互不依赖，只能依赖 msf_core 与第三方库。"
                "请把组合逻辑移到 apps/ demo/ 或用户 main。")
        endif()
    endforeach()
endfunction()

function(msf_app name)
    # 组合层标记：允许链接任意模块，不做约束。
endfunction()
