/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file StlMap.hxx
 * This file provides a feature limited abstraction of std::map
 *
 * @author Stuart W. Baker
 * @date 2 December 2013
 */

#ifndef _StlMap_hxx_
#define _StlMap_hxx_

#include <map>

#include "utils/macros.h"

/** Though at the surface, this may seem like an unnecessary abstraction of
 * std::map, it has the purpose of limiting the implementation to features
 * found only in common to @ref SysMap and @ref LinearMap implementations.  In
 * this way, one can use @ref Map and based on compile time settings choose to
 * use any one of @ref StlMap, @ref SysMap, or @ref LinearMap from the same
 * source usage of @ref Map.
 */
template <typename Key, typename Value> class StlMap
{
public:
    /** Default Constructor which with no mapping entry limit.
     */
    StlMap()
        : mappingAllocator(NULL),
          mapping(new Mapping())
    {
    }

    /** Constructor that limits the number of mappings to a static pool.
     * @param entries number of nodes to statically create and track
     */
    StlMap(size_t entries) 
        : mappingAllocator(new MappingAllocator(std::less<Key>(), Allocator<std::pair<const Key, Value>>(entries))),
          mapping(NULL)
    {
    }

    /** Destructor.
     */
    ~StlMap()
    {
        if (mappingAllocator)
        {
            delete mappingAllocator;
        }
        if (mapping)
        {
            delete mapping;
        }
    }

    /** This translation is done for consistency with @ref SysMap and @ref LinearMap */
    typedef pair<Key, Value> Pair;

    /** Short hand for the iterator type of a given instance */
    typedef typename std::map<Key, Value>::iterator Iterator;

    /** Remove an element from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        return mapping ? mapping->erase(key) : mappingAllocator->erase(key);
    }
    
    /** Remove a node from the tree.
     * @param it iterator index for the element to remove
     */
    void erase(Iterator it)
    {
        mapping ? mapping->erase(it) : mappingAllocator->erase(it);
    }
    
    /** Find the index associated with the key and create it if does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value& operator[](const Key &key)
    {
        return mapping ? (*mapping)[key] : (*mappingAllocator)[key];
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return mapping ? mapping->size() : mappingAllocator->size();
    }

    /** Maximum theoretical number of elements in the map.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return mapping ? mapping->max_size() : mappingAllocator->max_size();
    }

    /** Find an element matching the given key.
     * @param key key to search for
     * @return iterator index pointing to key, else iterator end() if not found
     */
    Iterator find( const Key &key )
    {
        return mapping ? mapping->find(key) : mappingAllocator->find(key);
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    Iterator end()
    {
        return mapping ? mapping->end() : mappingAllocator->end();
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    Iterator begin()
    {
        return mapping ? mapping->begin() : mappingAllocator->begin();
    }

private:
    /** This is a custom allocator that limits the number of mappings.  It also
     * performs a single dynamic allocation capable of holding the total number
     * of mappings on the first allocation request.  This is so that successive
     * allocation requests are faster, more deterministic, and don't waste
     * memory potentially used for the heap management headers.
     */
    template <typename T> class Allocator
    {
    public:
        /** List of unused elements.
         */
        union FreeList
        {
            T data; /**< unused item */
            FreeList *next; /**< next item in the list */
        };

        /** Start of free list */
        FreeList *freeList;
        
        /** flag that tell us if we have initilized our selves or not */
        bool init;
        
        /** number of elements in the fixed size pool */
        size_t entries;
        
        typedef T value_type; /**< value_type required by stl */
        typedef value_type* pointer; /**< pointer required by stl */
        typedef const value_type* const_pointer; /**< const_pointer required by stl */
        typedef value_type& reference; /**< reference required by stl */
        typedef const value_type& const_reference; /**< const_reference required by stl */
        typedef std::size_t size_type; /**< size_type required by stl */
        typedef std::ptrdiff_t difference_type; /**< difference_type required by stl */
        
        /** typedef for allocator specialization */
        template <typename U> struct rebind
        {
            typedef Allocator<U> other;
        };

        /** Constructor.
         * @param e number of entries in the fixed size pool
         */
        explicit Allocator(size_t e)
            : freeList(NULL),
              init(false),
              entries(e)
        {
        }

        /** Copy constructor.
         * @param a instance to copy
         */
        explicit Allocator(Allocator const& a)
            : freeList(a.freeList),
              init(a.init),
              entries(a.entries)
        {
        }
        
        /** Destructor.
         */
        ~Allocator()
        {
        }
        
        /** template copy constructor.
         * @param o insance to copy
         */
        template <typename U> Allocator(Allocator<U> const& o)
            : freeList(NULL),
              init(o.init),
              entries(o.entries)
        {
        }

        /** Address of item.
         * @param r item to take the address of
         * @return address of item
         */
        T *address(T &r)
        {
            return &r;
        }

        /** Const address of item.
         * @param r item to take the address of
         * @return address of item
         */
        const T *address(const T &r)
        {
            return &r;
        }

        /** Allocate item(s) out of the pool.
         * @param cnt number of items to allocate
         * @return newly allocated item(s)
         */
        T *allocate(size_t cnt, const void* = 0)
        {
            if (init == false)
            {
                HASSERT(entries != 0);
                init = true;
                FreeList *newList = (FreeList*)malloc(sizeof(FreeList) * max_size());
                for (size_t i = 0; i < max_size(); ++i)
                {
                    newList[i].next = freeList;
                    freeList = newList + i;
                }
            }
            HASSERT(freeList != NULL);
            HASSERT(cnt == 1);
            
            T *newT = &(freeList->data);
            freeList = freeList->next;
            return newT;
        }
        
        /** Free itme.
         * @param p item to free
         * @param n number of items to free
         */
        void deallocate(T *p, size_t n)
        {
            HASSERT(n == 1);
            FreeList *pFreeList = (FreeList*)p;
            pFreeList->next = freeList;
            freeList = pFreeList;
        }

        /** Maximum number of items that can be allocated.
         */
        size_t max_size() const
        {
            return entries;
        }

        
        /** Placement constructor.
         * @param p location of placement
         * @param t parameter to constructor
         */
        void construct(T *p, const T& t)
        {
            new(p) T(t);
        }
        
        /** Destruct.
         * @param p item to destruct
         */
        void destroy(T *p)
        {
            p->~T();
        }

        /** Overloaded operator ==.
         * @return always true.
         */
        bool operator==(Allocator const&)
        {
            return true;
        }
        
        /** Overloaded operator !=
         * @param a item to compare
         * @return always false
         */
        bool operator!=(Allocator const& a)
        {
            return !operator==(a);
        }
    private:
        /** Default Constructor.
         */
        Allocator()
            : init(false),
              entries(0)
        {
        }
    };

    /** short hand for the custom allocator std::map type */
    typedef std::map<Key, Value, std::less<Key>, Allocator<std::pair<const Key, Value>>> MappingAllocator;
    
    /** short hand for the default allocator std::map type */
    typedef std::map<Key, Value> Mapping;
    
    /** pointer to an std::map instance with a custom allocator */
    MappingAllocator *mappingAllocator;
    
    /** pointer to an std::map instance with a default allocator */
    Mapping *mapping;

    DISALLOW_COPY_AND_ASSIGN(StlMap);
};

#endif /* _StlMap_hxx_ */
