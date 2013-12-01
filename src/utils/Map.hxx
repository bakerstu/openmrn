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
 * \file RBTree.hxx
 * This file provides a C++ abstraction of the BSD <sys/tree.h>.
 *
 * @author Stuart W. Baker
 * @date 21 September 2013
 */

#ifndef _Map_hxx_
#define _Map_hxx_

extern "C"
{
#include <sys/tree.hxx>
};

#include "utils/macros.h"
#include <map>

/** Though the standard template library includes std::map, commonly
 * implemented as a Red Black tree, this alternative provides a transparent
 * mechanism for the user to manage all of the memory used by the tree.  This
 * allows for a instantiation that does not require the dynamic allocation
 * and freeing of small chunks of memory.  Note, there is no mutual exclusion
 * locking mechanism built into this class.  Mutual exclusion must be handled
 * by the user as needed.
 */
template <typename Key, typename Value> class Map
{
public:
    /** Default Constructor
     */
    Map() :
        entries(0),
#if defined (__LINEAR_MAP__)
        list(NULL),
        used(0)
    {
        /* dynamic allocation not supported */
        HASSERT(0);
    }
#elif defined (__USE_LIBSTDCPP__)
        mapping()
    {
    }
#else
        freeList(NULL)
    {
        RB_INIT(&head);
    }
#endif

    /** Constructor.
     * @param nodes number of nodes to statically create and track
     */
    Map(size_t entries) :
        entries(entries),
#if defined (__LINEAR_MAP__)
        list(new Pair[entries]()),
        used(0)
    {
    }
#elif defined (__USE_LIBSTDCPP__)
        mappingAllocator(std::less<Key>(), Allocator<std::pair<const Key, Value>>(entries))
    {
    }
#else
        freeList(NULL)
    {
        RB_INIT(&head);
        Node *first = new Node[entries];
        first->entry.rbe_left = (Node*)this;
        
        for (size_t i = 1; i < entries; i++)
        {
            first[i].entry.rbe_left = freeList;
            freeList = first + i;
        }
    }
#endif
#if defined (__LINEAR_MAP__)
    /** list entry. */
    struct Pair
    {
        union
        {
            Key key; /**< key for entry */
            Key first; /**< mimic first element in an std::pair */
        };
        union
        {
            Value value; /**< Value for entry */
            Value second; /**< mimic second element in an std::pair */
        };
    };
    
    class Iterator
    {
    public:
        Iterator(Map *context, ssize_t index)
            : index(index),
              m(context)
        {
        }
    
        Pair& operator*() const
        {
            return m->list[index];
        }

        /** Overloaded pre-increement operator. */
        Iterator& operator ++ ()
        {
            if (index >= 0)
            {
                ++index;
            }
            return *this;
        }

        /** Overloaded not equals operator. */
        bool operator != (const Iterator& it)
        {
            return index != it.index;
        }

        ssize_t index;
        Map *m;

        Iterator()
            : index(-1),
              m(NULL)
        {
        }
    };

    typedef Iterator iterator;
    
    Iterator endIterator;

    /** Remove a node from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        for (size_t i = 0; i < used; i++)
        {
            if (list[i].key == key)
            {
                /* scrunch up the list if we are able */
                if (i != --used)
                {
                    list[i].key = list[used].key;
                    list[i].value = list[used].value;
                }
                return 1;
            }
        }
        return 0;
    }
    
    /** Remove a node from the tree.
     * @param it iterator index for the element to remove
     */
    void erase(iterator it)
    {
        ssize_t i = it.index;
        HASSERT(i < (ssize_t)used && i > 0);

        /* scrunch up the list if we are able */
        if (i != (ssize_t)--used)
        {
            list[i].key = list[used].key;
            list[i].value = list[used].value;
        }
    }
    
    /** Find the index associated with the key and create it does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value& operator[](const Key &key)
    {
        for (size_t i = 0; i < used; i++)
        {
            if (list[i].key == key)
            {
                return list[i].value;
            }
        }
        
        HASSERT(used < entries);
        
        list[used].key = key;
        list[used].value = 0;
        
        return list[used++].value;
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return used;
    }

    /** Maximum theoretical number of elements in themap.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return entries;
    }

    /** Find an element matching the given key.
     * @param key key to search for
     * @return iterator index pointing to key, else iterator end() if not found
     */
    iterator find( const Key &key )
    {
        for (size_t i = 0; i < used; i++)
        {
            if (list[i].key == key)
            {
                return iterator(this, i);
            }
        }
        return endIterator;
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    iterator end()
    {
        return endIterator;
    }

    /** Get an iterator index pointing to the first element in the mapping.
     * @return iterator index pointing to the first element in the mapping or
     * to end() if the mapping is empty
     */
    iterator begin()
    {
        if (used > 0)
        {
            return iterator(this, 0);
        }
        else
        {
            return endIterator;
        }
    }

#elif defined (__USE_LIBSTDCPP__)
    template <typename T> class Allocator
    {
    public:
        union FreeList
        {
            T data;
            FreeList *next;
        };

        FreeList *freeList;
        
        bool init;
        
        size_t entries;
        
        typedef T value_type;
        typedef value_type* pointer;
        typedef const value_type* const_pointer;
        typedef value_type& reference;
        typedef const value_type& const_reference;
        typedef std::size_t size_type;
        typedef std::ptrdiff_t difference_type;
        
        template <typename U> struct rebind
        {
            typedef Allocator<U> other;
        };

        explicit Allocator(size_t e)
            : init(false),
              entries(e)
        {
        }

        explicit Allocator(Allocator const&)
            : init(false)
        {
        }
        
        ~Allocator()
        {
        }
        
        template <typename U> Allocator(Allocator<U> const& o)
            : init(false), entries(o.entries)
        {
        }

        //    address

        T *address(T &r)
        {
            return &r;
        }

        const T *address(const T &r)
        {
            return &r;
        }

        //    memory allocation

        T *allocate(size_t cnt, const void* = 0)
        {
            if (init == false)
            {
                HASSERT(entries != 0);
                init = true;
                FreeList *newList = (FreeList*)malloc(sizeof(FreeList) * max_size());
                newList->next = NULL;
                for (size_t i = 0; i < max_size(); ++i)
                {
                    freeList->next = &newList[i];
                    freeList = &newList[i];
                }
            }
            HASSERT(freeList != NULL);
            HASSERT(cnt == 1);
            
            T *newT = &(freeList->data);
            freeList = freeList->next;
            return newT;
        }
        
        void deallocate(T *p, size_t n)
        {
            HASSERT(n == 1);
            FreeList *pFreeList = (FreeList*)p;
            pFreeList->next = freeList;
            freeList = pFreeList;
        }

        //    size
        size_t max_size() const
        {
            return entries;
        }

        //    construction/destruction

        void construct(T *p, const T& t)
        {
            new(p) T(t);
        }
        
        void destroy(T *p)
        {
            p->~T();
        }

        bool operator==(Allocator const&)
        {
            return true;
        }
        
        bool operator!=(Allocator const& a)
        {
            return !operator==(a);
        }
    private:
        Allocator()
            : init(false),
              entries(0)
        {
        }
    };

    typedef std::map<Key, Value, std::less<Key>, Allocator<std::pair<const Key, Value>>> MappingAllocator;
    typedef std::map<Key, Value> Mapping;
    
    /** Remove an element from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        return mapping.erase(key);
    }
    
    /** Remove a node from the tree.
     * @param it iterator index for the element to remove
     */
    void erase(typename std::map<Key, Value>::iterator it)
    {
        mapping.erase(it);
    }
    
    /** Find the index associated with the key and create it does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value& operator[](const Key &key)
    {
        return mapping[key];
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return mapping.size();
    }

    /** Maximum theoretical number of elements in themap.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return mapping.max_size();
    }

    /** Find an element matching the given key.
     * @param key key to search for
     * @return iterator index pointing to key, else iterator end() if not found
     */
    typename std::map<Key, Value>::iterator find( const Key &key )
    {
        return mapping.find(key);
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    typename std::map<Key, Value>::iterator end()
    {
        return mapping.end();
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    typename std::map<Key, Value>::iterator begin()
    {
        return mapping.begin();
    }

#else
    struct Pair
    {
        Key first; /**< mimic first element in an std::pair */
        Value second; /**< mimic second element in an std::pair */
    };
    
    /** The metadata for a tree node. */
    class Node
    {
    public:
        RB_ENTRY(Node) entry;

        union
        {
            Pair p; /**< pair of elements */
            struct
            {
                Key key; /**< key by which to sort the node */
                Value value; /**< value of the node */
            };
        };

        /** Default constructor.  Does not initialize key or value.
         */
        Node()
        {
        }
        
        /** Constructor.
         * @param key initial value of key
         * @param value initial value of value
         */
        Node(Key key, Value value)
            : key(key),
              value(value)
        {
        }
    };

    class Iterator
    {
    public:
        Iterator(Map* context, Node *node)
            : node(node),
              m(context)
            
        {
        }
    
        Pair& operator*() const
        {
            return node->p;
        }

        /** Overloaded pre-increement operator. */
        Iterator& operator ++ ()
        {
            if (node)
            {
                node = RB_NEXT(m->tree, &(m->head), node);
            }
            return *this;
        }

        /** Overloaded not equals operator. */
        bool operator != (const Iterator& it)
        {
            return node != it.node;
        }

        Node *node;
        Map *m;

        Iterator()
            : node(NULL),
              m(NULL)
        {
        }
    };

    /** Find the index associated with the key and create it does not exist.
     * @param key key to lookup
     * @return value of the key by reference
     */
    Value& operator[](const Key &key)
    {
        Node *node = NULL; //find(key);
        if (!node)
        {
            node = alloc();
            if (node)
            {
                node->key = key;
                node->value = 0;
                RB_INSERT(tree, &head, node);
            }
        }
        return node->value;
    }

    /** Number of elements currently in the map.
     * @return number of elements in the map
     */
    size_t size()
    {
        return used;
    }

    /** Maximum theoretical number of elements in themap.
     * @return maximum theoretical number of elements in the map
     */
    size_t max_size()
    {
        return entries;
    }
    
    /** unique iterator instance to represent the end.
     */
    Iterator endIterator;
    
    typedef Iterator iterator;
    
    /** Find an element matching the given key.
     * @param key key to search for
     * @return iterator index pointing to key, else iterator end() if not found
     */
    iterator find( const Key &key )
    {
        Node lookup;
        lookup.key = key;
        Node *node = RB_FIND(tree, &head, &lookup);
        if (node);
        {
            return iterator(this, node);
        }
        return endIterator;
    }
    
    /** Get an iterator index pointing one past the last element in mapping.
     * @return iterator index pointing to one past the last element in mapping
     */
    iterator end()
    {
        return endIterator;
    }

    /** Get an iterator index pointing to the first element in the mapping.
     * @return iterator index pointing to the first element in the mapping or
     * to end() if the mapping is empty
     */
    iterator begin()
    {
        Node *node = RB_MIN(tree, &head);
        if (node)
        {
            return iterator(this, node);
        }
        else
        {
            return endIterator;
        }
    }

    /** Remove a node from the tree.
     * @param key key for the element to remove
     * @return number of elements removed
     */
    size_t erase(Key key)
    {
        Node lookup;
        lookup.key = key;
        Node *node = RB_FIND(tree, &head, &lookup);

        if (node)
        {
            RB_REMOVE(tree, &head, node);
            free(node);
            return 1;
        }
        return 0;
    }
    
    /** Remove a node from the tree.
     * @param it iterator index for the element to remove
     */
    void erase(iterator it)
    {
        Node *node = it.node;
        if (node)
        {
            RB_REMOVE(tree, &head, node);
            free(node);
        }
    }
    
#endif

    /** Default destructor */
    ~Map()
    {
    }
    
private:
    size_t entries; /**< total number of entries for this instance */

#if defined (__LINEAR_MAP__)
    Pair *list; /**< list of entries */
    size_t used; /**< total number of entries in use for this instance */

#elif defined (__USE_LIBSTDCPP__)
    union
    {
        MappingAllocator mappingAllocator;
        Mapping mapping;
    };

#else
    /** Allocate a node from the free list.
     * @return newly allocated node, else NULL if no free nodes left
     */
    Node *alloc()
    {
        if (freeList && freeList != (Node*)this)
        {
            Node *node = freeList;
            freeList = freeList->entry.rbe_left;
            return node;
        }
        return NULL;
    }
    
    /** free a node to the free list if it exists.
     * @param node node to free
     */
    void free(Node *node)
    {
        if (freeList || freeList == (Node*)this)
        {
            node->entry.rbe_left = freeList;
            freeList = node;
        }
    }

    /** Compare two nodes.
     * @param a first of two nodes to compare
     * @param b second of two nodes to compare
     * @return difference between node keys (a->key - b->key)
     */
    Key compare(Node *a, Node *b)
    {
        return a->key - b->key;
    }

    /** list of free nodes */
    Node *freeList;

    /** The datagram tree type. */
    RB_HEAD(tree, Node);

    /** The datagram tree methods. */
    RB_GENERATE(tree, Node, entry, compare);
    
    /** tree instance */
    struct tree head;

    size_t used; /**< total number of entries in use for this instance */
#endif
    DISALLOW_COPY_AND_ASSIGN(Map);
};

#endif /* _Map_hxx_ */
