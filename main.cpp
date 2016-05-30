#include "bvh.h"
using namespace BVH;
int main(int argc , char* argv[])
{
    BvhDocument doc = BvhDocument::fromFile("./bvh_0.bvh");
    doc.toFile("./bvh_0_new.bvh");
    Joint* s = SubstractJoints(doc.rootJoint());
    BvhDocument dst;
    dst.loadRootJoint(s);
    dst.toFile("./bvh_0_new2.bvh");
    return 0;
}
