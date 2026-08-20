# Frozen compiler flags for bit-exact / reproducible builds.
# Do NOT add -march=native or -ffast-math: they change floating-point accumulation order.

function(msf_apply_compiler_options target_name)
    target_compile_options(${target_name} PRIVATE
        -Wall
        $<$<CONFIG:Debug>:-O0 -g>
        $<$<CONFIG:Release>:-O2>
    )
endfunction()
