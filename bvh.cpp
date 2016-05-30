#include "bvh.h"
#include <sstream>
#include <vector>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <fstream>
using namespace BVH;
using namespace std;

Joint::Joint(Joint *parent)
    : m_parent(parent)
    , m_positonOrder(AxisOrder::Invalid)
    , m_rotationOrder(AxisOrder::Invalid)
{
    if (parent)
        parent->m_children.push_back(this);
}


Joint::~Joint()
{
    //! Remove self from parent
    if (m_parent)
        m_parent->removeChild(this);

    //! Delete children recursively
    for(auto i : m_children)
    {
        i->m_parent = nullptr;
        delete i;
    }
}

void Joint::apendChild(Joint *child)
{
    assert(child != nullptr);
    assert(child != this);
    assert(!child->isChildInTree(this));

    if (child->parent())
    {
        if (child->parent() == this) return;
        auto& children = child->parent()->m_children;
        for(auto i = children.begin(); i != children.end(); ++i)
        {
            if (*i == child)
            {
                children.erase(i);
                break;
            }
        }
    }    
    child->m_parent = this;
    m_children.push_back(child);
}

void Joint::removeChild(Joint *child)
{
    assert(child != nullptr);
    assert(child->parent() == this);
    for(auto i = m_children.begin() ; i != m_children.end() ; ++i)
    {
        if (*i == child)
        {
            m_children.erase(i);
            child->m_parent = nullptr;
            break;
        }
    }
}

void Joint::setParent(Joint *parent)
{
    if (parent)
    {
        assert(!isChildInTree(parent));
        parent->apendChild(this);
    }
    else
    {
        if (m_parent)
        {
            m_parent->removeChild(this);
        }
    }
}

bool Joint::isChildInTree(Joint *child, bool bRecursive) const
{
    assert(child != nullptr);
    if (bRecursive)
    {
        for(const auto i : m_children)
        {
            if (i == child)
            {
                return true;
            }
            else if (i->isChildInTree(child , true))
            {
                return true;
            }
            else
                continue;
        }
    }
    else
    {
        for(const auto i : m_children)
        {
            if (i == child)
            {
                return true;
            }
        }
    }
    return false;
}

int Joint::indexOfChild(Joint *child) const
{
    int counter = 0;
    for(auto i : m_children)
    {
        if (i == child)
        {
            return counter;
        }
        ++counter;
    }
    return -1;
}

Joint *Joint::childAt(int index) const
{
    int counter = 0;
    for(auto i : m_children)
    {
        if (counter == index)
        {
            return i;
        }
        ++counter;
    }
    return nullptr;
}

size_t Joint::frameCount() const
{
    if (m_positonOrder == AxisOrder::Invalid)
    {
        return m_frameData.size() / 3;
    }
    else
    {
        return m_frameData.size() / 6;
    }
}

int Joint::calcDepth() const
{
    if (m_parent == nullptr)
    {
        return 0;
    }
    else
    {
        return m_parent->calcDepth() + 1;
    }
}

static const char* jointNames_3DMaxBiped[] = {
    "Hips" ,
    "LeftHip" ,
    "LeftUpLeg" ,
    "LeftKnee" ,
    "LeftLowLeg" ,
    "LeftAnkle" ,
    "LeftFoot" ,

    "RightHip" ,
    "RightUpLeg" ,
    "RightKnee" ,
    "RightLowLeg" ,
    "RightAnkle" ,
    "RightFoot" ,

    "Chest" ,
    "Chest2" ,
    "Chest3" ,
    "Chest4" ,

    "LeftCollar" ,
    "LeftShoulder" ,
    "LeftUpArm" ,
    "LeftElbow" ,
    "LeftWrist" ,
    "LeftHand" ,
    "LeftFinger0" ,
    "LeftFinger01" ,
    "LeftFinger02" ,
    "LeftFinger1" ,
    "LeftFinger11" ,
    "LeftFinger12" ,
    "LeftFinger2" ,
    "LeftFinger21" ,
    "LeftFinger22" ,
    "LeftFinger3" ,
    "LeftFinger31" ,
    "LeftFinger32" ,
    "LeftFinger4" ,
    "LeftFinger41" ,
    "LeftFinger42" ,

    "RightCollar" ,
    "RightShoulder" ,
    "RightUpArm" ,
    "RightElbow" ,
    "RightWrist" ,
    "RightHand" ,
    "RightFinger0" ,
    "RightFinger01" ,
    "RightFinger02" ,
    "RightFinger1" ,
    "RightFinger11" ,
    "RightFinger12" ,
    "RightFinger2" ,
    "RightFinger21" ,
    "RightFinger22" ,
    "RightFinger3" ,
    "RightFinger31" ,
    "RightFinger32" ,
    "RightFinger4" ,
    "RightFinger41" ,
    "RightFinger42" ,

    "Neck" ,
    "Head" ,
    "Invalid"
};

static const char* jointNames_BioVision[] = {
  "Hips" ,
  "LeftUpLeg" ,
  "LeftLeg" ,
  "LeftFoot" ,

  "RightUpLeg" ,
  "RightLeg" ,
  "RightFoot" ,

  "Spine" ,
  "Spine1" ,
  "Spine2" ,
  "Spine3" ,

  "Neck" ,
  "Head" ,

  "LeftShoulder" ,
  "LeftArm" ,
  "LeftForeArm" ,
  "LeftHand" ,
  "LeftHandThumb1" ,
  "LeftHandThumb2" ,
  "LeftHandThumb3" ,

  "LeftInHandIndex" ,
  "LeftHandIndex1" ,
  "LeftHandIndex2" ,
  "LeftHandIndex3" ,

  "LeftInHandMiddle" ,
  "LeftHandMiddle1" ,
  "LeftHandMiddle2" ,
  "LeftHandMiddle3" ,

  "LeftInHandRing" ,
  "LeftHandRing1" ,
  "LeftHandRing2" ,
  "LeftHandRing3" ,

  "LeftInHandPinky" ,
  "LeftHandPinky1" ,
  "LeftHandPinky2" ,
  "LeftHandPinky3" ,

  "RightShoulder" ,
  "RightArm" ,
  "RightForeArm" ,
  "RightHand" ,
  "RightHandThumb1" ,
  "RightHandThumb2" ,
  "RightHandThumb3" ,

  "RightInHandIndex" ,
  "RightHandIndex1" ,
  "RightHandIndex2" ,
  "RightHandIndex3" ,

  "RightInHandMiddle" ,
  "RightHandMiddle1" ,
  "RightHandMiddle2" ,
  "RightHandMiddle3" ,

  "RightInHandRing" ,
  "RightHandRing1" ,
  "RightHandRing2" ,
  "RightHandRing3" ,

  "RightInHandPinky" ,
  "RightHandPinky1" ,
  "RightHandPinky2" ,
  "RightHandPinky3" ,
  "Invalid"
};

const std::string BVH::jointTypeToName_3DMaxBiped(JointType_3DMaxBiped type)
{
    return jointNames_3DMaxBiped[static_cast<int>(type)];
}

const std::string BVH::jointTypeToName_BioVision(JointType_BioVision type)
{
    return jointNames_BioVision[static_cast<int>(type)];
}

JointType_3DMaxBiped BVH::jointTypeFromName_3DMaxBiped(const std::string &name)
{
    for(unsigned i = 0; i < sizeof(jointNames_3DMaxBiped) / sizeof(char*); ++i)
    {
        if (jointNames_3DMaxBiped[i] == name)
        {
            return static_cast<JointType_3DMaxBiped>(i);
        }
    }
    return JointType_3DMaxBiped::Invalid;
}


JointType_BioVision BVH::jointTypeFromName_BioVision(const std::string &name)
{
    unsigned count = sizeof(jointNames_BioVision) / sizeof(char*);
    for(unsigned i = 0; i < count; ++i)
    {
        if (jointNames_BioVision[i] == name)
        {
            return static_cast<JointType_BioVision>(i);
        }
    }
    return JointType_BioVision::Invalid;
}

static bool readHIERARCHY(std::istream &is)
{
    is.seekg(0);
    string line;
    stringstream ss;
    string word;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        while(ss >> word)
        {
            if (word == "HIERARCHY")
            {
                return true;
            }
        }
        ss.clear();
    }
    return false;
}

static bool readRootHip(std::istream &is)
{
    string line;
    stringstream ss;
    string word;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        while(ss >> word)
        {
            if (word == "ROOT")
            {
                continue;
            }
            if (word == "Hips")
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

static bool readOpenBrace(istream& is)
{
    string line;
    string word;
    stringstream ss;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        while(ss >> word)
        {
            if (word == "{")
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

static bool readCloseBrace(istream& is)
{
    string line;
    string word;
    stringstream ss;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        while(ss >> word)
        {
            if (word == "}")
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

static bool readOffset(float &v0 , float &v1 , float &v2 , istream& is)
{
    string line;
    string word;
    stringstream ss;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        if (ss >> word && word == "OFFSET")
        {
            if (ss >> v0 >> v1 >> v2)
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

static std::vector<AxisOrder> readChannels(istream& is)
{
    string line;
    string word;
    stringstream ss;
    std::vector<AxisOrder> ret;
    int channelsCount = 0;
    string word0 , word1 , word2;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);

        if (!(ss >> word && word == "CHANNELS"))
        {
            return std::vector<AxisOrder>();
        }

        if (!(ss >> channelsCount))
        {
            return std::vector<AxisOrder>();
        }

        if (channelsCount == 3)
        {
            if (ss >> word0 >> word1 >> word2)
            {
                if (word0 == "Xrotation")
                {
                    if (word1 == "Yrotation" && word2 == "Zrotation")
                    {
                        ret.push_back(AxisOrder::XYZ);
                        break;
                    }
                    else if (word1 == "Zrotation" && word2 == "Yrotation")
                    {
                        ret.push_back(AxisOrder::XZY);
                        break;
                    }
                }
                else if (word0 == "Yrotation")
                {
                    if (word1 == "Xrotation" && word2 == "Zrotation")
                    {
                        ret.push_back(AxisOrder::YXZ);
                        break;
                    }
                    else if (word1 == "Zrotation" && word2 == "Xrotation")
                    {
                        ret.push_back(AxisOrder::YZX);
                        break;
                    }
                }
                else if (word0 == "Zrotation")
                {
                    if (word1 == "Xrotation" && word2 == "Yrotation")
                    {
                        ret.push_back(AxisOrder::ZXY);
                        break;
                    }
                    else if (word1 == "Yrotation" && word2 == "Xrotation")
                    {
                        ret.push_back(AxisOrder::ZYX);
                        break;
                    }
                }
            }
        }
        else if (channelsCount == 6)
        {
            if (ss >> word0 >> word1 >> word2)
            {
                if (word0 == "Xposition")
                {
                    if (word1 == "Yposition" && word2 == "Zposition")
                    {
                        ret.push_back(AxisOrder::XYZ);
                    }
                    else if (word1 == "Zposition" && word2 == "Yposition")
                    {
                        ret.push_back(AxisOrder::XZY);
                    }
                    else
                    {
                        return std::vector<AxisOrder>();
                    }
                }
                else if (word0 == "Yposition")
                {
                    if (word1 == "Xposition" && word2 == "Zposition")
                    {
                        ret.push_back(AxisOrder::YXZ);
                    }
                    else if (word1 == "Zposition" && word2 == "Xposition")
                    {
                        ret.push_back(AxisOrder::YZX);
                    }
                    else
                    {
                        return std::vector<AxisOrder>();
                    }
                }
                else if (word0 == "Zposition")
                {
                    if (word1 == "Xposition" && word2 == "Yposition")
                    {
                        ret.push_back(AxisOrder::ZXY);
                    }
                    else if (word1 == "Yposition" && word2 == "Xposition")
                    {
                        ret.push_back(AxisOrder::ZYX);
                    }
                    else
                    {
                        return std::vector<AxisOrder>();
                    }
                }
                else
                {
                    return std::vector<AxisOrder>();
                }
            }

            if (ss >> word0 >> word1 >> word2)
            {
                if (word0 == "Xrotation")
                {
                    if (word1 == "Yrotation" && word2 == "Zrotation")
                    {
                        ret.push_back(AxisOrder::XYZ);
                        break;
                    }
                    else if (word1 == "Zrotation" && word2 == "Yrotation")
                    {
                        ret.push_back(AxisOrder::XZY);
                        break;
                    }
                }
                else if (word0 == "Yrotation")
                {
                    if (word1 == "Xrotation" && word2 == "Zrotation")
                    {
                        ret.push_back(AxisOrder::YXZ);
                        break;
                    }
                    else if (word1 == "Zrotation" && word2 == "Xrotation")
                    {
                        ret.push_back(AxisOrder::YZX);
                        break;
                    }
                }
                else if (word0 == "Zrotation")
                {
                    if (word1 == "Xrotation" && word2 == "Yrotation")
                    {
                        ret.push_back(AxisOrder::ZXY);
                        break;
                    }
                    else if (word1 == "Yrotation" && word2 == "Xrotation")
                    {
                        ret.push_back(AxisOrder::ZYX);
                        break;
                    }
                }
            }
        }
        return std::vector<AxisOrder>();
    }
    return ret;
}

static bool readJointOrEndSite(string& type , string& name , istream& is)
{
    string line;
    stringstream ss;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);
        if (ss >> type)
        {
            if (type == "}")
            {
                return true;
            }
            else if ((type == "JOINT" || type == "End") && ss >> name)
            {
                return true;
            }
        }
        return false;
    }
    return false;
}

static Joint* readJoint(std::istream &is , Joint* parent = 0)
{
    auto g = is.tellg();
    if (!readOpenBrace(is))
    {
        is.seekg(g);
        return nullptr;
    }
    float v0 , v1 , v2;
    if (!readOffset(v0 , v1 , v2 , is))
    {
        is.seekg(g);
        return nullptr;
    }
    std::vector<AxisOrder> channels = readChannels(is);
    if (channels.empty())
    {
        is.seekg(g);
        return nullptr;
    }
    Joint* j = new Joint(parent);
    if (channels.size() == 1)
    {
        j->setRotationAxisOrder(channels[0]);
    }
    else
    {
        j->setPositionAxisOrder(channels[0]);
        j->setRotationAxisOrder(channels[1]);
    }

    j->setOffset(v0 , v1 , v2);
    for (;;)
    {
        string type , name;
        if (!readJointOrEndSite(type , name , is))
        {
            delete j;
            is.seekg(g);
            return nullptr;
        }
        if (type == "JOINT")
        {
            Joint* jChild = readJoint(is , j);
            if (jChild != nullptr)
            {
                jChild->setJointName(name);
                j->apendChild(jChild);
            }
            else
            {
                delete j;
                is.seekg(g);
                return nullptr;
            }
        }
        else if (type == "End")
        {
            if (!readOpenBrace(is))
            {
                delete j;
                is.seekg(g);
                return nullptr;
            }
            if (!readOffset(v0 , v1 , v2 , is))
            {
                delete j;
                is.seekg(g);
                return nullptr;
            }
            if (!readCloseBrace(is))
            {
                delete j;
                is.seekg(g);
                return nullptr;
            }
            Joint* jChild = new Joint(j);
            jChild->setOffset(v0 , v1 , v2);
            jChild->setAsEndSite(true);
            jChild->setJointName("EndSite");
        }
        else if (type == "}")
        {
            return j;
        }
    }    
    return nullptr;
}

bool writeToOStream(const Joint* joint , std::ostream &os)
{
    for (int i = 0; i < joint->depth(); ++i)
    {
        os << "    ";
    }
    os << "{" << endl;
    for (int i = 0; i <= joint->depth(); ++i)
    {
        os << "    ";
    }
    os << "OFFSET " << fixed << setprecision(8) << joint->x() << ' ' << fixed << setprecision(8) << joint->y() << ' ' << fixed << setprecision(10) << joint->z() << endl;
    if (joint->isEndSite())
    {
        for (int i = 0; i < joint->depth(); ++i)
        {
            os << "    ";
        }
        os << '}' << endl;
    }
    else
    {
        for (int i = 0; i <= joint->depth(); ++i)
        {
            os << "    ";
        }
        os << "CHANNELS ";
        if (joint->positionAxisOrder() == AxisOrder::Invalid)
        {
            os << 3 << " ";
        }
        else
        {
            os << 6 << " ";
            switch (joint->positionAxisOrder()) {
            case AxisOrder::XYZ:
                os << "Xposition Yposition Zposition ";
                break;
            case AxisOrder::XZY:
                os << "Xposition Zposition Yposition ";
                break;
            case AxisOrder::YXZ:
                os << "Yposition Xposition Zposition ";
                break;
            case AxisOrder::YZX:
                os << "Yposition Zposition Xposition ";
                break;
            case AxisOrder::ZXY:
                os << "Zposition Xposition Yposition ";
                break;
            case AxisOrder::ZYX:
                os << "Zposition Yposition Xposition ";
                break;
            default:
                break;
            }
        }
        switch(joint->rotationAxisOrder())
        {
        case AxisOrder::XYZ:
            os << "Xrotation Yrotation Zrotation" << endl;
            break;
        case AxisOrder::XZY:
            os << "Xrotation Zrotation Yrotation" << endl;
            break;
        case AxisOrder::YXZ:
            os << "Yrotation Xrotation Zrotation" << endl;
            break;
        case AxisOrder::YZX:
            os << "Yrotation Zrotation Xrotation" << endl;
            break;
        case AxisOrder::ZXY:
            os << "Zrotation Xrotation Yrotation" << endl;
            break;
        case AxisOrder::ZYX:
            os << "Zrotation Yrotation Xrotation" << endl;
            break;
        default:
            break;
        }
        for (auto i : joint->children())
        {
            for (int j = 0; j <= joint->depth(); ++j)
            {
                os << "    ";
            }
            if (i->isEndSite())
            {
                os << "End Site" << endl;
            }
            else
            {
                os << "JOINT " << i->jointName() << endl;
            }
            writeToOStream(i , os);
        }
        for (int i = 0; i < joint->depth(); ++i)
        {
            os << "    ";
        }
        os << '}' << endl;
    }
    return os.good();
}

static bool readMotion(std::istream& is)
{
    string line;
    stringstream ss;
    string word;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);

        if (ss >> word && word == "MOTION")
            return true;

        else
            return false;
    }

    return false;
}

static bool readFrameCount(int& frameCount , std::istream& is)
{
    string line;
    stringstream ss;
    string word;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);

        if (ss >> word)
        {
            if (word == "Frames")
            {
                if (ss >> word)
                {
                    if (word == ":")
                    {
                        if (ss >> frameCount)
                            return true;
                    }
                }

            }
            else if (word == "Frames:")
            {
                if (ss >> frameCount)
                    return true;
            }
        }

        return false;
    }
    return false;
}

static bool readFrameInterval(float& interval , std::istream& is)
{
    string line;
    stringstream ss;
    string word;
    while(std::getline(is , line))
    {
        if (line.empty()) continue;
        ss.str(line);

        if (ss >> word)
        {
            if (word == "Frame")
            {
                if (ss >> word)
                {
                    if (word == "Time:")
                    {
                        if (ss >> interval)
                            return true;
                    }
                    else if (word == "Time")
                    {
                        if (ss >> word && word == ":")
                        {
                            if (ss >> interval)
                                return true;
                        }
                    }
                }

            }
        }

        return false;
    }
    return false;
}

//! Todo 如果返回非空，接下来该去读取时间可帧
static Joint* fromIStream(std::istream &is)
{
    auto g = is.tellg();
    if(!readHIERARCHY(is))
    {
        is.seekg(g);
        is.clear();
        return nullptr;
    }
    if (!readRootHip(is))
    {
        is.seekg(g);
        is.clear();
        return nullptr;
    }
    Joint* j = readJoint(is);
    if (j != nullptr)
    {
        j->setJointName("Hips");
    }
    return j;
}

static bool toOStream(const Joint* joint , std::ostream &os)
{
    if (!(os << "HIERARCHY" << endl))
        return false;

    if (!(os << "ROOT Hips" << endl))
        return false;

    if (writeToOStream(joint , os))
        return true;

    return false;
}

static std::vector<Joint*> sequenceJoint(Joint* j)
{
    std::vector<Joint*> ret;

    if (!j || j->isEndSite())
        return ret;

    ret.push_back(j);

    auto children = j->children();
    for(auto i : children)
    {
        auto retTmp = sequenceJoint(i);
        ret.insert(ret.end() , retTmp.begin() , retTmp.end());
    }
    return ret;
}

BvhDocument::BvhDocument()
    : m_rootJoint(0)
    , m_frameInterval(0.0)
{

}

BvhDocument::BvhDocument(BvhDocument &&rhs)
    : m_rootJoint(rhs.m_rootJoint)
    , m_frameInterval(rhs.m_frameInterval)
{
    rhs.unloadRootJoint();
}

BvhDocument::~BvhDocument()
{
    if (m_rootJoint)
        delete m_rootJoint;

    m_rootJoint = 0;
    m_frameInterval = 0.0;
}

Joint *BvhDocument::unloadRootJoint()
{
    auto ret = m_rootJoint;
    m_rootJoint = 0;
    m_frameInterval = 0.0;
    return ret;
}

void BvhDocument::loadRootJoint(Joint *joint)
{
    if (m_rootJoint == joint)
        return;

    if (m_rootJoint)
        delete m_rootJoint;

    m_rootJoint = joint;
}

bool BvhDocument::toFile(const string &filename) const
{
    if (!m_rootJoint)
        return false;

    std::ofstream out(filename);

    if (!toOStream(m_rootJoint , out))
        return false;


    if (!(out << "MOTION" << endl << endl))
        return false;

    size_t frameCount = m_rootJoint->frameCount();

    if (out << "Frames: ")
    {
        if (!(out << frameCount << endl))
            return false;
    }
    else
    {
        return false;
    }

    out << "Frame Time: " << fixed << setprecision(8) << m_frameInterval << endl;

    std::vector<Joint*> jointSequence = sequenceJoint(m_rootJoint);

    for(unsigned int i = 0; i < frameCount; ++i)
    {
        for(Joint* j : jointSequence)
        {
            if (j->positionAxisOrder() != AxisOrder::Invalid)
            {
                size_t index = 6 * i;
                float px = j->frameData()[index + 0];
                float py = j->frameData()[index + 1];
                float pz = j->frameData()[index + 2];
                float rx = j->frameData()[index + 3];
                float ry = j->frameData()[index + 4];
                float rz = j->frameData()[index + 5];

                switch(j->positionAxisOrder())
                {
                case AxisOrder::XYZ:
                    out << px << ' ' << py << ' ' << pz << ' ';
                    break;
                case AxisOrder::XZY:
                    out << px << ' ' << pz << ' ' << py << ' ';
                    break;
                case AxisOrder::YXZ:
                    out << py << ' ' << px << ' ' << pz << ' ';
                    break;
                case AxisOrder::YZX:
                    out << py << ' ' << pz << ' ' << px << ' ';
                    break;
                case AxisOrder::ZXY:
                    out << pz << ' ' << px << ' ' << py << ' ';
                    break;
                case AxisOrder::ZYX:
                    out << pz << ' ' << py << ' ' << px << ' ';
                    break;
                default:
                    break;
                }


                switch(j->rotationAxisOrder())
                {
                case AxisOrder::XYZ:
                    out << rx << ' ' << ry << ' ' << rz << ' ';
                    break;
                case AxisOrder::XZY:
                    out << rx << ' ' << rz << ' ' << ry << ' ';
                    break;
                case AxisOrder::YXZ:
                    out << ry << ' ' << rx << ' ' << rz << ' ';
                    break;
                case AxisOrder::YZX:
                    out << ry << ' ' << rz << ' ' << rx << ' ';
                    break;
                case AxisOrder::ZXY:
                    out << rz << ' ' << rx << ' ' << ry << ' ';
                    break;
                case AxisOrder::ZYX:
                    out << rz << ' ' << ry << ' ' << rx << ' ';
                    break;
                default:
                    break;
                }
            }
            else
            {
                size_t index = 3 * i;
                float rx = j->frameData()[index + 0];
                float ry = j->frameData()[index + 1];
                float rz = j->frameData()[index + 2];
                switch(j->rotationAxisOrder())
                {
                case AxisOrder::XYZ:
                    out << rx << ' ' << ry << ' ' << rz << ' ';
                    break;
                case AxisOrder::XZY:
                    out << rx << ' ' << rz << ' ' << ry << ' ';
                    break;
                case AxisOrder::YXZ:
                    out << ry << ' ' << rx << ' ' << rz << ' ';
                    break;
                case AxisOrder::YZX:
                    out << ry << ' ' << rz << ' ' << rx << ' ';
                    break;
                case AxisOrder::ZXY:
                    out << rz << ' ' << rx << ' ' << ry << ' ';
                    break;
                case AxisOrder::ZYX:
                    out << rz << ' ' << ry << ' ' << rx << ' ';
                    break;
                default:
                    break;
                }
            }

        }
        out << endl;
    }

    return out.good();
}

BvhDocument BvhDocument::fromFile(const string &filename)
{
    std::ifstream in(filename);
    Joint* j = fromIStream(in);

    if (!j)
    {
        return BvhDocument();
    }

    if (!readMotion(in))
    {
        delete j;
        return BvhDocument();
    }

    int framesCount = 0;
    float frameInterval = 0.0;

    if (!readFrameCount(framesCount , in))
    {

        delete j;
        return BvhDocument();
    }

    if (!readFrameInterval(frameInterval , in))
    {
        delete j;
        return BvhDocument();
    }

    std::vector<Joint*> jointSequence = sequenceJoint(j);
    //! 读取帧数据，将数据与每一个节点绑定
    string line;
    stringstream ss;
    while(std::getline(in , line))
    {
        if (line.empty())
            continue;

        ss.str(line);
        for(Joint* i : jointSequence)
        {
            if (i->positionAxisOrder() != AxisOrder::Invalid)
            {
                //! 读取六个数据
                float p0 , p1 , p2;
                ss >> p0 >> p1 >> p2;
                switch(i->positionAxisOrder())
                {
                case AxisOrder::XYZ:
                    i->pushData(p0);
                    i->pushData(p1);
                    i->pushData(p2);
                    break;
                case AxisOrder::XZY:
                    i->pushData(p0);
                    i->pushData(p2);
                    i->pushData(p1);
                    break;
                case AxisOrder::YXZ:
                    i->pushData(p1);
                    i->pushData(p0);
                    i->pushData(p2);
                    break;
                case AxisOrder::YZX:
                    i->pushData(p2);
                    i->pushData(p0);
                    i->pushData(p1);
                    break;
                case AxisOrder::ZXY:
                    i->pushData(p1);
                    i->pushData(p2);
                    i->pushData(p0);
                    break;
                case AxisOrder::ZYX:
                    i->pushData(p2);
                    i->pushData(p1);
                    i->pushData(p0);
                    break;
                default:
                    break;
                }
            }
            float r0 , r1 , r2;
            ss >> r0 >> r1 >> r2;
            switch(i->rotationAxisOrder())
            {
            case AxisOrder::XYZ:
                i->pushData(r0);
                i->pushData(r1);
                i->pushData(r2);
                break;
            case AxisOrder::XZY:
                i->pushData(r0);
                i->pushData(r2);
                i->pushData(r1);
                break;
            case AxisOrder::YXZ:
                i->pushData(r1);
                i->pushData(r0);
                i->pushData(r2);
                break;
            case AxisOrder::YZX:
                i->pushData(r2);
                i->pushData(r0);
                i->pushData(r1);
                break;
            case AxisOrder::ZXY:
                i->pushData(r1);
                i->pushData(r2);
                i->pushData(r0);
                break;
            case AxisOrder::ZYX:
                i->pushData(r2);
                i->pushData(r1);
                i->pushData(r0);
                break;
            default:
                break;
            }
        }

        ss.clear();
    }

    BvhDocument doc;
    doc.m_rootJoint = j;
    doc.m_frameInterval = frameInterval;

    return doc;
}

static Joint* CopyLeftEndJoint(const Joint* src)
{
    Joint* j = new Joint;
    j->setAsEndSite(true);
    j->setOffset(src->x() , src->y() , src->z());
    j->setJointName(src->jointName());
    j->setPositionAxisOrder(src->positionAxisOrder());
    j->setRotationAxisOrder(src->rotationAxisOrder());
    j->frameData() = src->frameData();
    return j;
}

static Joint* CopyRightEndJoint(const Joint* src)
{
    Joint* j = new Joint;
    j->setAsEndSite(true);
    j->setOffset(src->x() , src->y() , src->z());
    j->setJointName(src->jointName());
    j->setPositionAxisOrder(src->positionAxisOrder());
    j->setRotationAxisOrder(src->rotationAxisOrder());
    j->frameData() = src->frameData();
    return j;
}

Joint *BVH::SubstractJoints(const Joint *src)
{
    if (src->jointName() == "LeftFinger1Nub")
    {
        return CopyLeftEndJoint(src);
    }
    else if (src->jointName() == "LeftFinger2Nub")
    {
        return CopyLeftEndJoint(src);
    }
    else if (src->jointName() == "LeftFinger3Nub")
    {
        return CopyLeftEndJoint(src);
    }
    else if (src->jointName() == "LeftFinger4Nub")
    {
        return CopyLeftEndJoint(src);
    }
    else if (src->jointName() == "RightFinger1Nub")
    {
        return CopyRightEndJoint(src);
    }
    else if (src->jointName() == "RightFinger2Nub")
    {
        return CopyRightEndJoint(src);
    }
    else if (src->jointName() == "RightFinger3Nub")
    {
        return CopyRightEndJoint(src);
    }
    else if (src->jointName() == "RightFinger4Nub")
    {
        return CopyRightEndJoint(src);
    }
    else
    {
        Joint* j = new Joint;
        j->setAsEndSite(src->isEndSite());
        j->setOffset(src->x() , src->y() , src->z());
        j->setJointName(src->jointName());
        j->setPositionAxisOrder(src->positionAxisOrder());
        j->setRotationAxisOrder(src->rotationAxisOrder());
        j->frameData() = src->frameData();
        for(const Joint* child : src->children())
        {
            Joint*jChild = SubstractJoints(child);
            j->apendChild(jChild);
        }
        return j;
    }
}
