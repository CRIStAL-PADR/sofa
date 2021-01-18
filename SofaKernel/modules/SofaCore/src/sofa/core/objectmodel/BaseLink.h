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
#ifndef SOFA_CORE_OBJECTMODEL_BASELINK_H
#define SOFA_CORE_OBJECTMODEL_BASELINK_H

#include <sofa/core/config.h>
#include <sofa/core/ExecParams.h>
#include <string>

namespace sofa
{

namespace core
{

namespace objectmodel
{

class Base;
class BaseData;
class BaseClass;
class BaseObjectDescription;

/**
 *  \brief Abstract base class for all links in the scene grapn, independently of their type.
 */
class SOFA_CORE_API BaseLink
{
public:
    enum LinkFlagsEnum
    {
        FLAG_NONE       = 0,
        FLAG_MULTILINK  = 1 << 0, ///< True if link is an array
        FLAG_STRONGLINK = 1 << 1, ///< True if link has ownership of linked object(s)
        FLAG_DOUBLELINK = 1 << 2, ///< True if link has a reciprocal link in linked object(s)
        FLAG_DUPLICATE  = 1 << 4, ///< True if link duplicates another one (possibly with a different/specialized DestType)
        FLAG_STOREPATH  = 1 << 5, ///< True if link requires a path string in order to be created
    };
    typedef unsigned LinkFlags;

    /// This internal class is used by the initLink() methods to store initialization parameters of a Data
    class BaseInitLink
    {
    public:
        BaseInitLink(const std::string& name, const std::string& help) : name(name), help(help) {}
        std::string name;
        std::string help;
    };

    /// This internal class is used by the initLink() methods to store initialization parameters of a Data
    template<class Owner>
    class InitLink : public BaseInitLink
    {
    public:
        InitLink(Owner* o, const std::string& n, const std::string& h) : BaseInitLink(n, h), owner(o) {}
        Owner* owner;
    };

    BaseLink(LinkFlags flags);
    BaseLink(const BaseInitLink& init, LinkFlags flags);
    virtual ~BaseLink();

    const std::string& getName() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    /// Get help message
    const std::string& getHelp() const { return m_help; }


    /// Set help message
    void setHelp(const std::string& val) { m_help = val; }

    bool setOwner(Base* newOwner);
    Base* getOwner() const { return m_owner; }

    virtual size_t size() const = 0;
    bool addPath(const std::string& path);
    bool removePath(const std::string& path);
    bool removeAt(size_t index){ return _doRemoveAt_(index); }

    /// Add a new target to the link.
    bool add(Base* baseptr, const std::string& path) { return _doAdd_(baseptr, path); }

    /// Change the link's target at the provided index.
    bool set(Base* baseptr, size_t index=0) { return _doSet_(baseptr, index); }
    bool set(Base* baseptr, const std::string& path, size_t index=0) { return _doSet_(baseptr, path, index); }

    /// Get the link's target at the provided index.
    /// Returns nullptr if > size() or the link is pointing to a null object
    Base* get(size_t i=0) const { return _doGet_(i); }

    /// Get the link's target path at the provided index.
    /// Returns empty string if > size() or the link is pointing to a null object
    std::string getPath(size_t index=0) const;
    void setPath(const std::string& s, size_t index=0);

    /// Get the link's target path at the provided index.
    /// Returns empty string if > size() or the link is pointing to a null object
    std::string getLinkedPath(std::size_t index=0) const;

    /// Returns true if the Link is containing a pointer to the provided 'item'
    virtual bool contains(Base* item) = 0;

    /// Remove all links
    virtual void clear() = 0;

    /// Set one of the flags.
    void setFlag(LinkFlagsEnum flag, bool b)
    {
        if(b) m_flags |= LinkFlags(flag);
        else m_flags &= ~LinkFlags(flag);
    }

    /// Get one flag
    bool getFlag(LinkFlagsEnum flag) const { return (m_flags&LinkFlags(flag))!=0; }

    bool isMultiLink() const { return getFlag(FLAG_MULTILINK); }
    bool isStrongLink() const { return getFlag(FLAG_STRONGLINK); }
    bool isDoubleLink() const { return getFlag(FLAG_DOUBLELINK); }
    bool isDuplicate() const { return getFlag(FLAG_DUPLICATE); }
    bool storePath() const { return getFlag(FLAG_STOREPATH); }

    virtual const BaseClass* getDestClass() const = 0;
    virtual const BaseClass* getOwnerClass() const = 0;

    /// Return the number of changes since creation
    /// This can be used to efficiently detect changes
    int getCounter() const { return m_counter; }

    /// @name Serialization API
    /// @{

    /// Read the command line
    bool read( const std::string& str );

    /// Update pointers in case the pointed-to objects have appeared
    /// @return false if there are broken links
    bool updateLinks() ;

    /// Print the value of the associated variable
    virtual void printValue( std::ostream& ) const;

    /// Print the value of the associated variable
    virtual std::string getValueString() const;

    /// Print the value type of the associated variable
    virtual std::string getValueTypeString() const;

    bool parseString(const std::string& text, std::string* path, std::string* data = nullptr) const;

    /// @}

    /// @name Serialization Helper API
    /// @{

    static bool ParseString(const std::string& text, std::string* path, std::string* data = nullptr, Base* start = nullptr);
    static std::string CreateString(const std::string& path, const std::string& data="");
    static std::string CreateStringPath(Base* object, Base* from);
    static std::string CreateStringData(BaseData* data);
    static std::string CreateString(Base* object, Base* from);
    static std::string CreateString(BaseData* data, Base* from);
    static std::string CreateString(Base* object, BaseData* data, Base* from);

    /// @}
    ///
    [[deprecated("2020-10-03: Deprecated since PR #1503. BaseLink cannot hold Data anymore. Use DataLink instead. Please update your code. ")]]
    sofa::core::objectmodel::BaseData* getOwnerData() const {return nullptr;}

    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    int getCounter(const core::ExecParams*) const { return getCounter(); }

    [[deprecated("2020-10-03: Deprecated since PR #1503. BaseLink cannot hold Data anymore. Use DataLink instead. Please update your code. ")]]
    bool isDataLink() const { return false; }

    [[deprecated("This function has been deprecated in PR#1503 and will be removed soon. Link<> cannot hold BaseData anymore. To make link between Data use DataLink instead.")]]
    BaseData* getLinkedData(std::size_t =0) const { return nullptr; }

    [[deprecated("TODO")]]
    Base* getLinkedBase(std::size_t index=0) const { return get(index); }

    [[deprecated("TODO")]]
    void setLinkedBase(Base* link);

    [[deprecated("TODO.")]]
    sofa::core::objectmodel::Base* getOwnerBase() const {return getOwner();}

    [[deprecated("TODO.")]]
    void setPersistent(bool b) { setFlag(FLAG_STOREPATH, b); } ///< Alias to match BaseData API

    [[deprecated("TODO.")]]
    bool isPersistent() const { return storePath(); } ///< Alias to match BaseData API

    [[deprecated("TODO.")]]
    bool isReadOnly() const   { return !storePath(); } ///< Alias to match BaseData API

    [[deprecated("TODO.")]]
    size_t getSize() const { return size(); }

protected:
    virtual Base* _doGet_(const size_t index) const = 0;
    virtual bool _doAdd_(Base* target, const std::string&) = 0;
    virtual bool _doRemoveAt_(size_t) = 0;
    virtual bool _doSet_(Base* target, const size_t index) = 0;
    virtual bool _doSet_(Base* v, const std::string& path, size_t=0) = 0;
    virtual bool _isCompatibleOwnerType_(const Base*) const = 0;
    virtual std::string _doGetPath_(const size_t index) const = 0;

    void setOwnerImpl(Base* owner);

    void updateCounter() { ++m_counter; }

    unsigned int m_flags;
    std::string m_name;
    std::string m_help;

    Base* m_owner {nullptr};  ///< The owner of this link.
    int m_counter;            ///< Number of changes since creation
};

} // namespace objectmodel

} // namespace core

// the BaseLink class is used everywhere
using core::objectmodel::BaseLink;

} // namespace sofa

#endif
