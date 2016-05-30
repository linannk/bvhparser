#ifndef BVH_H
#define BVH_H

#include <string>
#include <list>
#include <istream>
#include <ostream>
#include <iostream>
#include <vector>
#include <map>

namespace BVH {

//!
//! \brief The JointType_3DMaxBiped enum Joints alias
//!
enum class JointType_3DMaxBiped {
	Hip = 0 ,
    LeftHip ,
    LeftUpLeg ,
    LeftKnee ,
    LeftLowLeg ,
    LeftAnkle ,
    LeftFoot ,

    RightHip ,
    RightUpLeg ,
    RightKnee ,
    RightLowLeg ,
    RightAnkle ,
    RightFoot ,

    Chest ,
    Chest2 ,
    Chest3 ,
    Chest4 ,

    LeftCollar ,
    LeftShoulder ,
    LeftUpArm ,
    LeftElbow ,
    LeftWrist ,
    LeftHand ,
    LeftFinger0 ,
    LeftFinger01 ,
    LeftFinger02 ,
    LeftFinger1 ,
    LeftFinger11 ,
    LeftFinger12 ,
    LeftFinger2 ,
    LeftFinger21 ,
    LeftFinger22 ,
    LeftFinger3 ,
    LeftFinger31 ,
    LeftFinger32 ,
    LeftFinger4 ,
    LeftFinger41 ,
    LeftFinger42 ,

    RightCollar ,
    RightShoulder ,
    RightUpArm ,
    RightElbow ,
    RightWrist ,
    RightHand ,
    RightFinger0 ,
    RightFinger01 ,
    RightFinger02 ,
    RightFinger1 ,
    RightFinger11 ,
    RightFinger12 ,
    RightFinger2 ,
    RightFinger21 ,
    RightFinger22 ,
    RightFinger3 ,
    RightFinger31 ,
    RightFinger32 ,
    RightFinger4 ,
    RightFinger41 ,
    RightFinger42 ,

    Neck ,
    Head ,

    Invalid
};

//!
//! \brief The JointType_BioVision enum Joints alias
//!
enum class JointType_BioVision {
    Hip  = 0 ,
    LeftUpLeg ,
    LeftLeg ,
    LeftFoot ,

    RightUpLeg ,
    RightLeg ,
    RightFoot ,

    Spine ,
    Spine1 ,
    Spine2 ,
    Spine3 ,

    Neck ,
    Head ,

    LeftShoulder ,
    LeftArm ,
    LeftforeArm ,
    LeftHand ,
    LeftHandThumb1 ,
    LeftHandThumb2 ,
    LeftHandThumb3 ,

    LeftInHandIndex ,
    LeftInHandIndex1 ,
    LeftInHandIndex2 ,
    LeftInHandIndex3 ,

    LeftInHandMiddle ,
    LeftInHandMiddle1 ,
    LeftInHandMiddle2 ,
    LeftInHandMiddle3 ,

    LeftInHandRing ,
    LeftInHandRing1 ,
    LeftInHandRing2 ,
    LeftInHandRing3 ,

    LeftInHandPinky ,
    LeftInHandPinky1 ,
    LeftInHandPinky2 ,
    LeftInHandPinky3 ,

    RightShoulder ,
    RightArm ,
    RightforeArm ,
    RightHand ,
    RightHandThumb1 ,
    RightHandThumb2 ,
    RightHandThumb3 ,

    RightInHandIndex ,
    RightInHandIndex1 ,
    RightInHandIndex2 ,
    RightInHandIndex3 ,

    RightInHandMiddle ,
    RightInHandMiddle1 ,
    RightInHandMiddle2 ,
    RightInHandMiddle3 ,

    RightInHandRing ,
    RightInHandRing1 ,
    RightInHandRing2 ,
    RightInHandRing3 ,

    RightInHandPinky ,
    RightInHandPinky1 ,
    RightInHandPinky2 ,
    RightInHandPinky3 ,

    Invalid
};

//!
//! \brief The AxisOrder enum Axis Order
//! \remarks The data is read and written in order of axis.
//!
enum AxisOrder {
    Invalid = 0x0 ,
    XYZ  = 0x01,
    XZY  = 0x02,
    YXZ  = 0x04,
    YZX  = 0x08,
    ZXY  = 0x10,
    ZYX  = 0x20
};

//!
//! \brief jointTypeToName_3DMaxBiped Retrieve the name of a specified joint type.
//! \param type Joint type
//! \return The name of the joint type
//!
const std::string jointTypeToName_3DMaxBiped(JointType_3DMaxBiped type);
const std::string jointTypeToName_BioVision(JointType_BioVision type);

//!
//! \brief jointTypeFromName_3DMaxBiped 获取节点的类型
//! \param name 节点的名称
//! \return 节点的类型，如果有该类型则返回类型，否则返回Invalid
//!
JointType_3DMaxBiped jointTypeFromName_3DMaxBiped(const std::string& name);
JointType_BioVision jointTypeFromName_BioVision(const std::string& name);

class Joint {
public:

    //!
    //! \brief Joint Construct a joint object with the given parent.
    //! \param parent The pointer to the parent object
    //!
    explicit Joint(Joint* parent = 0);

    //!
    //! \brief ~Joint Destruct a joint.
    //! \remarks Before a joint is destruct , it's children will be automatically destructed.
    ~Joint();

    //!
    //! \brief jointName 获取节点的名称
    //! \return 节点的名称
    //!
    std::string jointName() const { return m_jointName; }
    void setJointName(const std::string& name) { m_jointName = name; }

    void setX(float x) { m_x = x; }
    void setY(float y) { m_y = y; }
    void setZ(float z) { m_z = z; }
    void setOffset(float x , float y , float z) { m_x = x , m_y = y , m_z = z; }
    float x() const { return m_x; }
    float y() const { return m_y; }
    float z() const { return m_z; }

    //!
    //! \brief isEndSite 判断节点是否是结束节点
    //! \return
    //! \remarks 不拥有子节点的节点就是结束节点
    bool isEndSite() const { return m_isEndSite; }

    //!
    //! \brief setEndSite 设置EndSite
    //! \param isEndSite 是否是为EndSite
    //! \remarks 该属性将影响序列化
    void setAsEndSite(bool isEndSite = true) { m_isEndSite = isEndSite; }

    //!
    //! \brief apendChild 添加一个子节点
    //! \param child 想要添加的子节点
    //! \remarks 添加的节点不应该为nullptr，如果子节点拥有父节点则会将其从原有的父节点上移除
    //!
    void apendChild(Joint* child);

    //!
    //! \brief removeChild 移除一个子节点
    //! \param child 想要移除的子节点
    //!
    void removeChild(Joint* childAt);

    //!
    //! \brief setParent 设置父对象
    //! \param parent 父对象的指针
    //!
    void setParent(Joint* parent);

    //!
    //! \brief isChildInTree 是否拥有某个孩子节点
    //! \param child 孩子节点的指针
    //! \param bRecursive 是否递归搜索
    //! \return 如果指定的额节点被找到则返回true，否则返回false。
    //!
    bool isChildInTree(Joint* child , bool bRecursive = true) const;

    //!
    //! \brief findChild 查找子节点
    //! \param child 被查找的节点
    //! \return 如果存在该子节点，则返回其索引，否则返回-1
    //!
    int indexOfChild(Joint* child) const;

    //!
    //! \brief children 返回孩子节点的列表
    //! \return 孩子节点的列表
    //! \remarks 可以利用返回的列表修改孩子节点的属性
    const std::vector<Joint*>& children() const { return m_children; }
    std::vector<Joint*>& children() { return m_children; }

    //!
    //! \brief depth 返回节点的深度
    //! \return 节点的深读
    //! \remarks 如果一个节点没有父节点则其深度为0
    int depth() const { return calcDepth(); }

    //!
    //! \brief parent 获取节点的父节点
    //! \return 父节点的指针
    //! \remarks 如果一个节点没有父节点，返回值为nullptr
    Joint* parent() const { return m_parent; }

    //!
    //! \brief child
    //! \param index
    //! \return
    //!
    Joint* childAt(int index) const;

    //!
    //! \brief childrenCount 获取拥有的子节点的数量
    //! \return 子节点的数量
    size_t childrenCount() const { return m_children.size(); }

    AxisOrder positionAxisOrder() const { return m_positonOrder; }

    AxisOrder rotationAxisOrder() const { return m_rotationOrder; }

    void setPositionAxisOrder(AxisOrder order) { m_positonOrder = order; }
    void setRotationAxisOrder(AxisOrder order) { m_rotationOrder = order; }

    void pushData(float data) { m_frameData.push_back(data); }
    const std::vector<float>& frameData() const { return m_frameData; }
    std::vector<float>& frameData() { return m_frameData; }

    size_t frameCount() const;

protected:

    //!
    //! \brief calcDepth 计算节点的深度
    //! \return  节点的深度。
    //! \remarks 如果一个节点没有父节点，则其深度为0
    //!
    int calcDepth() const;
private:

    Joint(const Joint& other) = delete;
    Joint& operator = (const Joint& other) = delete;

    //!
    //! \brief m_isEndSite  节点是否是结束节点
    //!
    bool m_isEndSite = false;

    //!
    //! \brief m_parent 父节点
    //!
    Joint* m_parent = nullptr;

    //!
    //! \brief m_jointName 节点的名称
    //!
    std::string m_jointName;

    AxisOrder m_positonOrder;
    AxisOrder m_rotationOrder;

    float m_x;
    float m_y;
    float m_z;

    //!
    //! \brief m_children 节点的子节点
    //!
    std::vector<Joint*> m_children;

    std::vector<float> m_frameData;
};

Joint* SubstractJoints(const Joint* src);



class BvhDocument {
public:

    //!
    //! \brief BvhDocument 默认的构造函数，构造一个空的对象
    //!
    BvhDocument();

    //!
    //! \brief BvhDocument 移动拷贝构造函数
    //! \param rhs 将要销毁的对象
    //!
    BvhDocument(BvhDocument&& rhs);

    //!
    //! 销毁一个对象
    ~BvhDocument();

    bool isEmpty() const { return m_rootJoint == nullptr; }

    //!
    //! \brief unloadRootJoint 将拥有的RootJoint卸载
    //! \return 根节点的指针
    //! \remarks 调用该函数后，对象将不会再维护RootJoint
    Joint* unloadRootJoint();

    //!
    //! \brief loadRootJoint 加载根节点
    //! \param joint
    //!
    void loadRootJoint(Joint* joint);

    //!
    //! \brief rootJoint 获取根节点的指针
    //! \return 根节点的指针
    //!
    Joint* rootJoint() const { return m_rootJoint; }

    //!
    //! \brief writeToFile 将节点信息和帧信息写到文件中去
    //! \param filename 创建的文件的名称
    //! \return 如果创建并且写入并且写入成功则返回真，负责返回假
    //!
    bool toFile(const std::string& filename) const;

    void  setFrameInterval(float interval) { m_frameInterval = interval; }
    float frameInterval() const { return m_frameInterval; }
private:
    BvhDocument(const BvhDocument& other) = delete;
    BvhDocument& operator = (const BvhDocument& other) = delete;

    //!
    //! \brief m_rootJoint 根节点
    //!
    Joint* m_rootJoint;

    //!
    //! \brief m_frameInterval 一帧持续的时间
    //!
    float m_frameInterval;

public:
    static BvhDocument fromFile(const std::string& filename);
};

}

#endif // BVH_H
