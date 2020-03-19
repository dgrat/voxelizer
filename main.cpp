#include <iostream>
#include "voxelizer.h"
#include "rules.h"
#include "checks.h"
#include "stl/stl_import.h"
#include "xml_config.h"


int main()
{
    cfg::xml_project pro("shapes/");
    voxelize::voxelizer<shell_rule> v(pro);
    v.run();
    v.to_stl("voxels.stl");
    v.to_vox("voxels.vox");
    return 0;
}
