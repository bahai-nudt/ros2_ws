# GNSS/INS 组合导航 demo：复现 POST_MSF GNSS/INS 主流程（含速度更新与 ZUPT）。
# 参考输出（POST_MSF）在 ref_out/，本程序输出到 out/，可用 tools/compare_result.py 对比。

add_executable(gnss_ins main.cpp)
target_link_libraries(gnss_ins PRIVATE msf_io msf_gnss msf_ekf msf_ins msf_tools)
msf_apply_compiler_options(gnss_ins)
msf_app(gnss_ins)
