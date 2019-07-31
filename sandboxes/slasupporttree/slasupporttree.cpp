#include <iostream>
#include <fstream>
#include <string>

#include <libslic3r/libslic3r.h>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Tesselate.hpp>
#include <libslic3r/ClipperUtils.hpp>
#include <libslic3r/SLA/SLAAutoSupports.hpp>
#include <libslic3r/SLA/SLASupportTree.hpp>
#include <libslic3r/MTUtils.hpp>

#include <tbb/parallel_for.h>
#include <tbb/mutex.h>
#include <future>

const std::string USAGE_STR = {
    "Usage: slasupporttree stlfilename.stl"
};

int main(const int argc, const char *argv[]) {
    using namespace Slic3r;
    using std::cout; using std::endl;

    if(argc < 2) {
        cout << USAGE_STR << endl;
        return EXIT_SUCCESS;
    }
    
    SpinMutex mutex;
    
    std::vector<int> vec;
    
    srand (time(NULL)); /*std::array<std::future<void>, 1000> res;*/
    
//    for(size_t i = 0; i < 1000; i++) res[i] = std::async(std::launch::async, [&vec, &mutex](size_t i) {
//        usleep(std::rand() % 100000);
//        std::lock_guard<SpinMutex> lk(mutex);
//        vec.emplace_back(i);
//        vec.back() = vec.back() - 1;
//        std::cout << vec.back() << std::endl;
//    }, i);
    

    tbb::parallel_for(size_t(0), size_t(1000), [&vec, &mutex](size_t i) {
        int t = std::rand() % 10000;
        usleep(t);
        std::lock_guard<SpinMutex> lk(mutex);
        vec.emplace_back(t);
        vec.back() = vec.back() - 1;
        std::cout << vec.back() << std::endl;
    });

//    TriangleMesh model;

//    model.ReadSTLFile(argv[1]);
//    model.align_to_origin();
//    model.require_shared_vertices();
    
//    TriangleMeshSlicer slicer(&model);
    
//    auto bb = model.bounding_box();
//    Vec3f bbmin = bb.min.cast<float>(), bbmax = bb.max.cast<float>();
    
//    std::vector<float> slicegrid = grid(bbmin.z(), bbmax.z(), 0.05f);
//    std::vector<ExPolygons> slices;
    
//    slicer.slice(slicegrid, 0.1f, &slices, [](){});

    return EXIT_SUCCESS;
}
