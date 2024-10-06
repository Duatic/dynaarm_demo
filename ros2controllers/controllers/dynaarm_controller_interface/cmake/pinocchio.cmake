set(PINOCCHIO_FLAGS
    ${pinocchio_CFLAGS_OTHER}
    -Wno-ignored-attributes
    -Wno-invalid-partial-specialization
    -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
    -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)