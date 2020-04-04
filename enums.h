#pragma once


enum voxel_type {
    interior    = 1 << 2,
    shell       = 1 << 4
};
    
enum swizzle_mode {
    xyz = 0,
    xzy,
    yxz,
    yzx,
    zxy,
    zyx
};
