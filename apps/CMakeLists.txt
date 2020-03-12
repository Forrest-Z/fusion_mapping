# 准备RPATH
file(RELATIVE_PATH _rel ${CMAKE_INSTALL_PREFIX}/${INSTALL_BINDIR} ${CMAKE_INSTALL_PREFIX})
set(_rpath "\$ORIGIN/${_rel}")
file(TO_NATIVE_PATH "${_rpath}/${INSTALL_LIBDIR}" rpath)

add_subdirectory(front_end)
add_subdirectory(test)
add_subdirectory(back_end)
add_subdirectory(viewer)
add_subdirectory(data_preprocess)
