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
#ifndef SOFA_CORE_OBJECTMODEL_LINK_H
#define SOFA_CORE_OBJECTMODEL_LINK_H
#include <sofa/core/objectmodel/BaseLink.h>
#include <sofa/core/PathResolver.h>
#include <sofa/helper/stable_vector.h>
#include <functional>
namespace sofa
{

namespace core
{


namespace objectmodel
{

class DDGNode;

template<class TDestType, bool strongLink>
class LinkTraitsDestPtr;

template<class TDestType>
class LinkTraitsDestPtr<TDestType, false>
{
public:
    typedef TDestType* T;
    static TDestType* get(T p) { return p; }
};

template<class TDestType>
class LinkTraitsDestPtr<TDestType, true>
{
public:
    typedef typename TDestType::SPtr T;
    static TDestType* get(const T& p) { return p.get(); }
};

template<class TDestType, class TDestPtr, bool strongLink, bool storePath>
class LinkTraitsValueType;

template<class TDestType, class TDestPtr, bool strongLink>
class LinkTraitsValueType<TDestType,TDestPtr,strongLink, false>
{
public:
    typedef TDestPtr T;
    static bool path(const T& /*ptr*/, std::string& /*str*/)
    {
        return false;
    }
    static const TDestPtr& get(const T& v) { return v; }
    static void set(T& v, const TDestPtr& ptr) { v = ptr; }
    static void setPath(T& /*ptr*/, const std::string& /*name*/) {}
};

template<class TDestType, class TDestPtr, bool strongLink>
class LinkTraitsValueType<TDestType,TDestPtr,strongLink, true>
{
public:
    typedef LinkTraitsDestPtr<TDestType, strongLink> TraitsDestPtr;

    struct T
    {
        TDestPtr ptr;
        std::string path;
        T() : ptr(TDestPtr()) {}
        explicit T(const TDestPtr& p) : ptr(p) {}
        operator TDestType*() const { return TraitsDestPtr::get(ptr); }
        void operator=(const TDestPtr& v) { if (v != ptr) { ptr = v; path.clear(); } }
        TDestType& operator*() const { return *ptr; }
        TDestType* operator->() const { return TraitsDestPtr::get(ptr); }
        TDestType* get() const { return TraitsDestPtr::get(ptr); }
        bool operator!() const { return !ptr; }
        bool operator==(const TDestPtr& p) const { return ptr == p; }
        bool operator!=(const TDestPtr& p) const { return ptr != p; }
    };
    static bool path(const T& v, std::string& str)
    {
        if (v.path.empty()) return false;
        str = v.path;
        return true;
    }
    static const TDestPtr& get(const T& v) { return v.ptr; }
    static void set(T& v, const TDestPtr& ptr) { if (v.ptr && ptr != v.ptr) v.path.clear(); v.ptr = ptr; }
    static void setPath(T& v, const std::string& name) { v.path = name; }
};

template<class TDestType, class TDestPtr, class TValueType, bool multiLink>
class LinkTraitsContainer;


/// Class to hold 0-or-1 pointer. The interface is similar to std::vector (size/[]/begin/end), plus an automatic convertion to one pointer.
template < class T, class TPtr = T* >
class SinglePtr
{
protected:
    TPtr elems[1];
public:
    typedef T pointed_type;
    typedef TPtr value_type;
    typedef value_type const * const_iterator;
    typedef value_type const * const_reverse_iterator;

    SinglePtr()
    {
        elems[0] = TPtr();
    }
    const_iterator begin() const
    {
        return elems;
    }
    const_iterator end() const
    {
        return (!elems[0])?elems:elems+1;
    }
    const_reverse_iterator rbegin() const
    {
        return begin();
    }
    const_reverse_iterator rend() const
    {
        return end();
    }
    const_iterator cbegin() const
    {
        return begin();
    }
    const_iterator cend() const
    {
        return end();
    }
    const_reverse_iterator crbegin() const
    {
        return rbegin();
    }
    const_reverse_iterator crend() const
    {
        return rend();
    }
    std::size_t size() const
    {
        return (!elems[0])?0:1;
    }
    bool empty() const
    {
        return !elems[0];
    }
    void clear()
    {
        elems[0] = TPtr();
    }
    const TPtr& get() const
    {
        return elems[0];
    }
    TPtr& get()
    {
        return elems[0];
    }
    const TPtr& operator[](std::size_t i) const
    {
        return elems[i];
    }
    TPtr& operator[](std::size_t i)
    {
        return elems[i];
    }
    const TPtr& operator()(std::size_t i) const
    {
        return elems[i];
    }
    TPtr& operator()(std::size_t i)
    {
        return elems[i];
    }
    operator T*() const
    {
        return elems[0];
    }
    T* operator->() const
    {
        return elems[0];
    }
};

template<class TDestType, class TDestPtr, class TValueType>
class LinkTraitsContainer<TDestType, TDestPtr, TValueType, false>
{
public:
    typedef SinglePtr<TDestType, TValueType> T;
    static void clear(T& c)
    {
        c.clear();
    }
    static std::size_t add(T& c, TDestPtr v)
    {
        c.get() = v;
        return 0;
    }
    static std::size_t find(const T& c, TDestPtr v)
    {
        if (c.get() == v) return 0;
        else return 1;
    }
    static void remove(T& c, std::size_t index)
    {
        if (!index)
            c.clear();
    }
};

template<class TDestType, class TDestPtr, class TValueType>
class LinkTraitsContainer<TDestType, TDestPtr, TValueType, true>
{
public:
    /// Container type.
    /// We use stable_vector to allow insertion/removal of elements
    /// while iterators are used (required to add/remove objects
    /// while visitors are in progress).
    typedef sofa::helper::stable_vector<TValueType> T;
    static void clear(T& c)
    {
        c.clear();
    }
    static std::size_t add(T& c, TDestPtr v)
    {
        std::size_t index = c.size();
        c.push_back(TValueType(v));
        return index;
    }
    static std::size_t find(const T& c, TDestPtr v)
    {
        size_t s = c.size();
        for (size_t i=0; i<s; ++i)
            if (c[i] == v) return i;
        return s;
    }
    static void remove(T& c, std::size_t index)
    {
        c.erase( c.begin()+index );
    }
    static void set(T& c, std::size_t index, TDestPtr v)
    {
        c[index] = v;
    }
};

/**
 *  \brief Container of all links in the scenegraph, from a given type of object (Owner) to another (Dest)
 *
 */
template<class TOwnerType, class TDestType, unsigned TFlags>
class TLink : public BaseLink
{
public:
    typedef TOwnerType OwnerType;
    typedef TDestType DestType;
    enum { ActiveFlags = TFlags };
#define ACTIVEFLAG(f) ((ActiveFlags & (f)) != 0)
    typedef LinkTraitsDestPtr<DestType, ACTIVEFLAG(FLAG_STRONGLINK)> TraitsDestPtr;
    typedef typename TraitsDestPtr::T DestPtr;
    typedef LinkTraitsValueType<DestType, DestPtr, ACTIVEFLAG(FLAG_STRONGLINK), ACTIVEFLAG(FLAG_STOREPATH)> TraitsValueType;
    typedef typename TraitsValueType::T ValueType;
    typedef LinkTraitsContainer<DestType, DestPtr, ValueType, ACTIVEFLAG(FLAG_MULTILINK)> TraitsContainer;
    typedef typename TraitsContainer::T Container;
    typedef typename Container::const_iterator const_iterator;
    typedef typename Container::const_reverse_iterator const_reverse_iterator;
#undef ACTIVEFLAG

    TLink()
        : BaseLink(ActiveFlags)
    {
    }

    TLink(const InitLink<OwnerType>& init)
        : BaseLink(init, ActiveFlags)
    {
        setOwner(init.owner);
    }

    ~TLink() override
    {
    }

    size_t size() const override
    {
        return static_cast<size_t>(m_value.size());
    }

    bool empty() const
    {
        return m_value.empty();
    }

    const Container& getValue() const
    {
        return m_value;
    }

    const_iterator begin() const
    {
        return m_value.cbegin();
    }

    const_iterator end() const
    {
        return m_value.cend();
    }

    const_reverse_iterator rbegin() const
    {
        return m_value.crbegin();
    }

    const_reverse_iterator rend() const
    {
        return m_value.crend();
    }

    void clear() override
    {
        TraitsContainer::clear(m_value);
    }

    bool contains(Base* baseptr) override
    {
        auto destptr = dynamic_cast<DestType*>(baseptr);
        return TraitsContainer::find(m_value, destptr) < m_value.size();
    }

    void setNotificationFunction(std::function<DestPtr(OwnerType* owner, DestPtr before, DestPtr after, size_t index)> t)
    {
        notifyChangeCb = t;
    }

    bool add(DestPtr v)
    {
        if (!v)
            return false;
        std::size_t index = TraitsContainer::add(m_value,v);
        updateCounter();
        notifyChange( nullptr, v, index);
        return true;
    }

    bool add(DestPtr v, const std::string& path)
    {
        if (!v && path.empty())
            return false;
        std::size_t index = TraitsContainer::add(m_value,v);
        TraitsValueType::setPath(m_value[index],path);
        updateCounter();
        notifyChange( nullptr, v, index);
        return true;
    }

    bool remove(DestPtr v)
    {
        if (!v)
            return false;
        return removeAt(TraitsContainer::find(m_value, v));
    }

    bool removeAt(std::size_t index)
    {
        if (index >= m_value.size())
            return false;

        DestPtr v = TraitsDestPtr::get(TraitsValueType::get(m_value[index]));
        TraitsContainer::remove(m_value,index);
        updateCounter();
        notifyChange( v, nullptr, index);
        return true;
    }

    const BaseClass* getDestClass() const override
    {
        return DestType::GetClass();
    }

    const BaseClass* getOwnerClass() const override
    {
        return OwnerType::GetClass();
    }

    static const BaseClass* GetDestClass()
    {
        return DestType::GetClass();
    }

    static const BaseClass* GetOwnerClass()
    {
        return OwnerType::GetClass();
    }

    void setOwner(OwnerType* owner)
    {
        /// Forward the owner to the parent implementation.
        BaseLink::setOwnerImpl(owner);
    }

    OwnerType* getOwner()
    {
        return dynamic_cast<OwnerType*>(m_owner);
    }

    DestType* get(std::size_t index=0) const
    {
        return TraitsDestPtr::get(TraitsValueType::get(m_value[index]));
    }

    DestType* operator[](std::size_t index) const
    {
        return get(index);
    }


    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    const Container& getValue(const core::ExecParams*) const { return getValue(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    const_iterator begin(const core::ExecParams*) const { return begin(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    const_iterator end(const core::ExecParams*) const { return end(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    const_reverse_iterator rbegin(const core::ExecParams*) const { return rbegin(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    const_reverse_iterator rend(const core::ExecParams*) const { return rend(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    bool empty(const core::ExecParams* param) const ;
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    size_t size(const core::ExecParams*) const { return size(); }
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    DestType* get(std::size_t index, const core::ExecParams*) const { return _doGet_(index); }

protected:
    std::function<DestPtr(OwnerType* owner, DestPtr before, DestPtr after, size_t index)> notifyChangeCb;
    void notifyChange(DestPtr oldValue, DestPtr newValue, size_t index=0)
    {
        if(notifyChangeCb)
        {
            DestPtr overridenValue = notifyChangeCb(TLink::getOwner(), oldValue, newValue, index);
            if(overridenValue!=newValue)
                TraitsValueType::set(m_value[index], overridenValue);
        }
    }

    /// Returns false on type mismatch
    bool _doSet_(Base* baseptr, const size_t index) override
    {
        assert(index < m_value.size());
        auto destptr = dynamic_cast<DestType*>(baseptr);

        if(!destptr)
            return false;

        TraitsValueType::set(m_value[index], destptr);
        return true;
    }

    /// Set a new link entry from a Base*
    /// returns false if neither base & path are provided or if the provided base object has the wrong type.
    bool _doAdd_(Base* baseptr, const std::string& path) override
    {
        /// If the pointer is null and the path empty we do nothing
        if(!baseptr && path.empty())
            return false;

        /// Downcast the pointer to a compatible type and
        /// If the types are not compatible with the Link we returns false
        auto destptr = dynamic_cast<DestType*>(baseptr);
        if(baseptr && !destptr)
        {
            return false;
        }

        /// TLink:adding accepts nullptr (for a not yet resolved link).
        return TLink::add(destptr, path);
    }

    bool _doRemoveAt_(std::size_t index) override
    {
        if (index >= m_value.size())
            return false;

        DestPtr v = TraitsDestPtr::get(TraitsValueType::get(m_value[index]));
        TraitsContainer::remove(m_value,index);
        updateCounter();
        notifyChange( v, nullptr, index);
        return true;
    }

    /// Returns the not resolved path at the provided index.
    std::string _doGetPath_(const std::size_t index) const override
    {
        assert(index < m_value.size() );
        std::string path;
        const ValueType& value = m_value[index];
        TraitsValueType::path(value, path);
        return path;
    }

    bool _doSet_(Base* baseptr, const std::string& path, size_t index=0) override
    {
        if(index >= size())
            return false;

        /// Downcast the pointer to a compatible type and
        /// If the types are not compatible with the Link we returns false
        auto v = dynamic_cast<DestType*>(baseptr);

        ValueType& value = m_value[index];
        const DestPtr before = TraitsValueType::get(value);
        if (v != before)
            TraitsValueType::set(value, v);
        TraitsValueType::setPath(value, path);
        updateCounter();
        if (v != before)
            notifyChange( before, v);
        return true;
    }

    Base* _doGet_(const std::size_t index=0) const override
    {
        return TLink::get(index);
    }

    bool _isCompatibleOwnerType_(const Base* obj) const final
    {
        return dynamic_cast<const OwnerType*>(obj) != nullptr;
    }

    Container m_value;

    DestType* getIndex(std::size_t index) const
    {
        if (index < m_value.size())
            return TraitsDestPtr::get(TraitsValueType::get(m_value[index]));
        else
            return nullptr;
    }
};

/**
 *  \brief Container of vectors of links in the scenegraph, from a given type of object (Owner) to another (Dest)
 *
 */
template<class TOwnerType, class TDestType, unsigned TFlags>
class MultiLink : public TLink<TOwnerType,TDestType,TFlags|BaseLink::FLAG_MULTILINK>
{
public:
    typedef TLink<TOwnerType,TDestType,TFlags|BaseLink::FLAG_MULTILINK> Inherit;
    typedef TOwnerType OwnerType;
    typedef TDestType DestType;
    typedef typename Inherit::TraitsDestPtr TraitsDestPtr;
    typedef typename Inherit::DestPtr DestPtr;
    typedef typename Inherit::TraitsValueType TraitsValueType;
    typedef typename Inherit::ValueType ValueType;
    typedef typename Inherit::TraitsContainer TraitsContainer;
    typedef typename Inherit::Container Container;


    MultiLink() {}
    MultiLink(const BaseLink::InitLink<OwnerType>& init)
        : Inherit(init)
    {
    }

    MultiLink(const BaseLink::InitLink<OwnerType>& init, DestPtr val)
        : Inherit(init)
    {
        if (val) this->add(val);
    }

    virtual ~MultiLink()
    {
    }

    [[deprecated("2021-01-01: CheckPaths as been deprecated for complete removal in PR. You can update your code by using PathResolver::CheckPaths(Base*, BaseClass*, string).")]]
    static bool CheckPaths(const std::string& path, Base* context)
    {
        return PathResolver::CheckPaths(context, Inherit::GetDestClass(), path);
    }
};

/**
 *  \brief Container of single links in the scenegraph, from a given type of object (Owner) to another (Dest)
 *
 */
template<class TOwnerType, class TDestType, unsigned TFlags>
class SingleLink : public TLink<TOwnerType,TDestType,TFlags&~BaseLink::FLAG_MULTILINK>
{
public:
    typedef TLink<TOwnerType,TDestType,TFlags&~BaseLink::FLAG_MULTILINK> Inherit;
    typedef TOwnerType OwnerType;
    typedef TDestType DestType;
    typedef typename Inherit::TraitsDestPtr TraitsDestPtr;
    typedef typename Inherit::DestPtr DestPtr;
    typedef typename Inherit::TraitsValueType TraitsValueType;
    typedef typename Inherit::ValueType ValueType;
    typedef typename Inherit::TraitsContainer TraitsContainer;
    typedef typename Inherit::Container Container;
    using Inherit::updateCounter;
    using Inherit::m_value;
    using Inherit::m_owner;
    using Inherit::notifyChange;

    SingleLink()
    {
    }

    SingleLink(const BaseLink::InitLink<OwnerType>& init)
        : Inherit(init)
    {
    }

    SingleLink(const BaseLink::InitLink<OwnerType>& init, DestPtr val)
        : Inherit(init)
    {
        if (val) this->add(val);
    }

    virtual ~SingleLink()
    {
    }

    std::string getPath() const
    {
        return Inherit::getPath(0);
    }

    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    DestType* get(const core::ExecParams*) const { return get(); }
    DestType* get() const
    {
        return TraitsDestPtr::get(TraitsValueType::get(m_value.get()));
    }

    void reset()
    {
        set(nullptr);
    }

    void set(DestPtr newvalue)
    {
        ValueType& value = m_value.get();
        const DestPtr before = TraitsValueType::get(value);
        if (newvalue == before) return;
        TraitsValueType::set(value, newvalue);
        updateCounter();
        notifyChange( before, newvalue);
    }

    operator DestType*() const
    {
        return get();
    }
    DestType* operator->() const
    {
        return get();
    }
    DestType& operator*() const
    {
        return *get();
    }

    DestPtr operator=(DestPtr v)
    {
        set(v);
        return v;
    }

    [[deprecated("2021-01-01: CheckPath as been deprecated for complete removal in PR. You can update your code by using PathResolver::CheckPath(Base*, BaseClass*, string).")]]
    static bool CheckPath(const std::string& path, Base* context)
    {
        return PathResolver::CheckPath(context, Inherit::GetDestClass(), path);
    }
};

} // namespace objectmodel

} // namespace core

// the SingleLink class is used everywhere
using core::objectmodel::SingleLink;

// the MultiLink class is used everywhere
using core::objectmodel::MultiLink;

} // namespace sofa

#endif
