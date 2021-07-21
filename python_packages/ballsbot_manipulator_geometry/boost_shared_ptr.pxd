# -*- coding: utf-8 -*-
from libcpp cimport bool

cdef extern from "boost/shared_ptr.hpp" namespace "boost" nogil:
    cdef cppclass shared_ptr[T]:
        shared_ptr()
        shared_ptr(T*)
        # shared_ptr(T*, T*)
        # shared_ptr(T*, T*, T*)
        # shared_ptr(weak_ptr[T])
        # shared_ptr(weak_ptr[T], boost::detail::sp_nothrow_tag)

        T* get()
        bool unique()
        long use_count()
        void swap(shared_ptr[T])
        void reset(T*)
