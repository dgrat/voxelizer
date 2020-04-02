#include <iostream>
#include "voxelizer.h"
#include "rules.h"
#include "checks.h"
#include "stl/stl_import.h"
#include "xml_config.h"


int main()
{
    stl::format stl;
    stl.load("shapes/model.stl");
    auto poly = stl.to_polyhedron(stl.faces());
    auto poly_int = poly.prepare_for_rasterization(glm::ivec3(16,16,16));
    auto poly_flt = poly.prepare_for_rasterization(glm::vec3(32,32,32));


    poly.to_obj("orig.obj");
    poly_int.to_obj("int.obj");
    poly_flt.to_obj("flt.obj");
/*
    cfg::xml_project pro("shapes/");
    voxelize::voxelizer<shell_rule> v(pro);
    v.run();
    v.to_stl("voxels.stl");
    v.to_vox("voxels.vox");
*/
    return 0;
}
