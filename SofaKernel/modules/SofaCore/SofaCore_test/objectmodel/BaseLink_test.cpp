/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/objectmodel/BaseObject.h>
using sofa::core::objectmodel::BaseObject ;
using sofa::core::objectmodel::Base;

#include <sofa/core/objectmodel/BaseNode.h>
using sofa::core::objectmodel::BaseNode ;

#include <sofa/core/objectmodel/BaseLink.h>
using sofa::core::objectmodel::BaseLink ;

#include <sofa/helper/testing/BaseTest.h>
using sofa::helper::testing::BaseTest ;

class FakeObject : public BaseObject
{
public:
    FakeObject() : BaseObject() {}
};

class FakeMultiLinkImplementation : public BaseLink
{
public:
    FakeMultiLinkImplementation() : BaseLink(BaseLink::FLAG_MULTILINK) {}
    bool contains(Base*) override { return true; }
    void clear() override  { return; }
    sofa::core::objectmodel::Base* getOwnerBase() const override { return nullptr; }
    sofa::core::objectmodel::BaseData* getOwnerData() const override { return nullptr; }
    sofa::core::objectmodel::BaseClass* getOwnerClass() const override { return nullptr; }
    sofa::core::objectmodel::BaseClass* getDestClass() const override { return nullptr; }
    size_t getSize() const override {return 0;}
    sofa::core::objectmodel::BaseData* getLinkedData(std::size_t) const override { return nullptr; }
    bool _doAdd_(Base*, const std::string&) override { return true; }
    bool _doSet_(Base*, const size_t) override {return true;}
    bool _doSetOwner_(Base*) override {return true;}
    Base* _doGetOwner_() const override {return nullptr;}
    Base* _doGet_(const size_t) const override {return nullptr; }
    std::string _doGetPath_(const size_t) const override { return ""; }
};

class FakeSingleLinkImplementation : public BaseLink
{
public:
    FakeSingleLinkImplementation() : BaseLink(0) {}
    bool contains(Base*) override { return true; }
    void clear() override  { return; }
    sofa::core::objectmodel::Base* getOwnerBase() const override { return nullptr; }
    sofa::core::objectmodel::BaseData* getOwnerData() const override { return nullptr; }
    sofa::core::objectmodel::BaseClass* getOwnerClass() const override { return nullptr; }
    sofa::core::objectmodel::BaseClass* getDestClass() const override { return nullptr; }
    size_t getSize() const override {return 0;}
    sofa::core::objectmodel::BaseData* getLinkedData(std::size_t) const override { return nullptr; }
    bool _doAdd_(Base*, const std::string&) override { return true; }
    bool _doSet_(Base*, const size_t) override {return true;}
    bool _doSetOwner_(Base*) override {return true;}
    Base* _doGetOwner_() const override {return nullptr;}
    Base* _doGet_(const size_t) const override {return nullptr; }
    std::string _doGetPath_(const size_t) const override {return ""; }
};


class BaseLink_test: public BaseTest
{
public:
};

TEST_F(BaseLink_test, checkOwnerShipTransfer)
{
    FakeObject object1;
    FakeObject object2;
    FakeSingleLinkImplementation slink;

    slink.setOwner(&object1);
    EXPECT_EQ(slink.getOwner(), &object1);
    ASSERT_EQ(object1.getLinks().size(), 1);
    EXPECT_EQ(object1.getLinks()[0], &slink);

    slink.setOwner(&object2);
    EXPECT_EQ(slink.getOwner(), &object2);
    EXPECT_EQ(object1.getLinks().size(), 0);
    EXPECT_EQ(object2.getLinks().size(), 0);
    EXPECT_EQ(object2.getLinks()[0], &slink);
}

TEST_F(BaseLink_test, checkSingleLinkRead)
{
    FakeSingleLinkImplementation slink;
    EXPECT_EQ(slink.read("@/this/is/a/validpath"), true);
    EXPECT_EQ(slink.read("@/this/is/an/other/validpath @/this/is/also"), false);
    EXPECT_EQ(slink.read("/not/a/validpath @/this/is/also"), false);
    EXPECT_EQ(slink.read("/not/a/validpath /this/is/also"), false);
}

TEST_F(BaseLink_test, checkMultLinkRead)
{
    FakeMultiLinkImplementation mlink;
    EXPECT_EQ(mlink.read("@/this/is/a/validpath"), true);
    EXPECT_EQ(mlink.read("@/this/is/an/other/validpath @/this/is/also"), true);
    EXPECT_EQ(mlink.read("/not/a/validpath @/this/is/also"), false);
    EXPECT_EQ(mlink.read("/not/a/validpath /this/is/also"), false);
}
