#include <iostream>
#include "voxelizer.h"
#include "rules.h"
#include "checks.h"
#include "stl/stl_import.h"
#include "xml_config.h"


int main()
{
    cfg::xml_project pro(".");
    voxelize::voxelizer v(pro);
    v.run();
    //v.to_fs_stl<build_stl_cube>();
    //v.to_fs_bytes();
    
    return 0;
}
