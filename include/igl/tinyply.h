/*
 * tinyply 2.4 (https://github.com/ddiakopoulos/tinyply)
 *
 * A single-header, zero-dependency (except the C++ STL) public domain implementation
 * of the PLY mesh file format. Requires C++11; errors are handled through exceptions.
 *
 * This software is in the public domain. Where that dedication is not
 * recognized, you are granted a perpetual, irrevocable license to copy,
 * distribute, and modify this file as you see fit.
 *
 * Authored by Dimitri Diakopoulos (http://www.dimitridiakopoulos.com)
 *
 * tinyply.h may be included in many files, however in a single compiled file,
 * the implementation must be created with the following defined prior to header inclusion
 * #define TINYPLY_IMPLEMENTATION
 *
 */

////////////////////////
//   tinyply header   //
////////////////////////

#ifndef tinyply_h
#define tinyply_h

#include <vector>
#include <string>
#include <stdint.h>
#include <cstddef>
#include <sstream>
#include <memory>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <functional>

namespace tinyply
{

    enum class Type : uint8_t
    {
        INVALID,
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        FLOAT32,
        FLOAT64
    };

    struct PropertyInfo
    {
        int32_t stride{ 0 };
        std::string str;
        PropertyInfo() = default;
        PropertyInfo(int32_t x, const std::string & y) : stride(x), str(y) {}
    };

    static std::map<Type, PropertyInfo> PropertyTable
    {
        { Type::INT8,    PropertyInfo(1, std::string("char")) },
        { Type::UINT8,   PropertyInfo(1, std::string("uchar")) },
        { Type::INT16,   PropertyInfo(2, std::string("short")) },
        { Type::UINT16,  PropertyInfo(2, std::string("ushort")) },
        { Type::INT32,   PropertyInfo(4, std::string("int")) },
        { Type::UINT32,  PropertyInfo(4, std::string("uint")) },
        { Type::FLOAT32, PropertyInfo(4, std::string("float")) },
        { Type::FLOAT64, PropertyInfo(8, std::string("double")) },
        { Type::INVALID, PropertyInfo(0, std::string("INVALID"))}
    };

    class Buffer
    {
        uint8_t * alias{ nullptr };
        struct delete_array { void operator()(uint8_t * p) { delete[] p; } };
        std::unique_ptr<uint8_t, decltype(Buffer::delete_array())> data;
        size_t size {0};
    public:
        Buffer() {};
        Buffer(const size_t size) : data(new uint8_t[size], delete_array()), size(size) { alias = data.get(); } // allocating
        Buffer(const uint8_t * ptr): alias(const_cast<uint8_t*>(ptr)) {}
        uint8_t * get() { return alias; }
        const uint8_t * get_const() {return const_cast<const uint8_t*>(alias); }
        size_t size_bytes() const { return size; }
    };

    struct PlyData
    {
        Type t;
        Buffer buffer;
        size_t count {0};
        bool isList {false};
        std::vector<size_t> list_indices;
    };

    struct PlyProperty
    {
        PlyProperty(std::istream & is);
        PlyProperty(Type type, std::string & _name) : name(_name), propertyType(type) {}
        PlyProperty(Type list_type, Type prop_type, std::string & _name, size_t list_count)
            : name(_name), propertyType(prop_type), isList(true), listType(list_type), listCount(list_count) {}
        std::string name;
        Type propertyType{ Type::INVALID };
        bool isList{ false };
        Type listType{ Type::INVALID };
        size_t listCount {0};
    };

    struct ProgressCallbackInfo
    {
        const std::string name;
        size_t current_bytes;
        size_t total_bytes;
    };

    struct PlyElement
    {
        PlyElement(std::istream & istream);
        PlyElement(const std::string & _name, size_t count) : name(_name), size(count) {}
        std::string name;
        size_t size {0};
        std::vector<PlyProperty> properties;
    };

    struct PlyFile
    {
        struct PlyFileImpl;
        std::unique_ptr<PlyFileImpl> impl;

        PlyFile();
        ~PlyFile();

        /*
         * The ply format starts with an ascii header. This can be used to determine at 
         * runtime which properties or elements exist in the file. Some validation of the 
         * header is performed; it is assumed the header correctly reflects the contents of the 
         * payload. Returns true on success, false if the header is malformed. tinyply might
         * be able to make some sense of a malformed header, but reading the file may crash. 
         */ 
        bool parse_header(std::istream & is);

        /*
         * These functions are only valid after a call to `parse_header(...)`. In the case of
         * writing, tje get_comments() reference may be used to add arbitrary text to the header.
         */
        std::vector<PlyElement> get_elements() const;
        std::vector<std::string> get_info() const;
        std::vector<std::string>& get_comments();
        bool is_binary_file() const;

        /*
         * A general use of the ply format is storing triangle meshes. When this fact 
         * is known a-priori, we may pass an expected list length that will apply to this element (3).
         * 
         * In the more general case where |list_size_hint| is zero, `read` performs a two-pass
         * parse over the file to support list types of variable or unknown length. Providing 
         * a hint enables an up-front memory allocation for lists and a much speedier parse. 
         * Vilya Harvey's ply-parsing-perf project calls this feature "precognition."
         * 
         * |propertyKeys| is a list of properties (available in the header) that will be read into the
         * output PlyData buffer. Note that properties will be provided in the order they appear in the
         * header, and no swizzling will be performed (e.g. if {"z", "y", "x"} is requested, the the items 
         * in PlyData will be {"x", "y", "z"} if that's how they are ordered in the header). 

         * This function is only valid after a call to `parse_header(...)` and might throw with
         * non-fatal exceptions. 
         */
        std::shared_ptr<PlyData> request_properties_from_element(const std::string & elementKey,
            const std::vector<std::string> propertyKeys, const uint32_t list_size_hint = 0);

        /*
         * This function mirrors `request_properties_from_element(...)` but for informing tinyply about
         * which data should be written. It primes the internal data structures prior to calling `write(...)`. 
         */
        void add_properties_to_element(const std::string & elementKey, 
            const std::vector<std::string> propertyKeys, const Type type, const size_t count, uint8_t * data, const Type listType, const size_t listCount);
        
        /* 
         * Execute a callback with reading progress every `num_bytes`. This is primarily for friendliness in working
         * with large files in gui-oriented applications. 
         */
        void set_progress_callback(const size_t num_bytes, std::function<void(const tinyply::ProgressCallbackInfo info)> callback);

        /* 
         * Parse the payload. Buffers must be requested via `request_properties_from_element(...)` 
         * prior to calling this function.
         */
        void read(std::istream & is);

        /* 
         * `write` performs no validation and assumes that the data passed into 
         * `add_properties_to_element(...)` is well-formed. Writing is also permitted
         * directly after a call to `read(...)` for transcoding (e.g. ascii to binary, etc),
         * but only for properties that were requested (which perhaps doesn't mirror
         * the original header). 
         */
        void write(std::ostream & os, bool isBinary);
    };

} // end namespace tinyply

#ifndef IGL_STATIC_LIBRARY
// implementation moved to tinyply.cpp
#  include "tinyply.cpp"
#endif


#endif // end tinyply_h
