#include "iobuffer.h"

namespace ros2_bridge {

#if 0
    static SPtr makeSPtr(std::size_t capacity)
    {
        std::size_t sz = (capacity > IoBuffer<S>::min_buffer_size) ? capacity : IoBuffer<S>::min_buffer_size ;
        SPtr mbp = std::make_shared<IoBuffer>((sz));
        return mbp;
    }
    static SPtr makeSPtr(std::string s)
    {
        SPtr mbp = std::make_shared<IoBuffer>((s.size()));
        mbp->append((void*) s.c_str(), s.size());
        return mbp;
    }
    static SPtr makeSPtr(void* mem, std::size_t size)
    {
        SPtr mbp = std::make_shared<IoBuffer>((size));
        mbp->append(mem, size);
        return mbp;
    }
    static SPtr makeSPtr(IoBuffer& mb)
    {
        SPtr mbp = std::make_shared<IoBuffer>((mb.capacity()));
        mbp->append(mb.data(), mb.size());
        return mbp;
    }
#endif
void IoBuffer::init(std::size_t cap)
{
    m_memPtr = malloc(cap);
    m_start_offset = 0;
    m_cPtr = (char*) m_memPtr;
    m_length = 0;
//    m_size = 0;
    m_capacity = cap;
}
//IoBuffer::IoBuffer()
//{
//    std::size_t tmp_cap =128;
//    m_memPtr = malloc(tmp_cap);
//    m_start_offset = 0;
//    m_cPtr = (char*) m_memPtr;
//    m_length = 0;
//    m_size = 0;
//    m_capacity = tmp_cap;
//}
IoBuffer::IoBuffer(const std::size_t cap)
{
    std::size_t tmp_cap = cap;
    m_memPtr = malloc(tmp_cap);
    m_start_offset = 0;
    m_cPtr = (char*) m_memPtr;
    m_length = 0;
//    m_size = 0;
    m_capacity = tmp_cap;
}
IoBuffer::IoBuffer(std::string& str, std::size_t cap)
{
    std::size_t tmp_cap = cap;
    m_memPtr = malloc(tmp_cap);
    m_start_offset = 0;
    m_cPtr = (char*) m_memPtr;
    m_length = 0;
//    m_size = 0;
    m_capacity = tmp_cap;
    this->append((void*)str.c_str(), str.size());
}
//IoBuffer::IoBuffer(IoBuffer& other)
//{
//    m_capacity = other.m_capacity;
//    m_memPtr = malloc(m_capacity);
//    m_start_offset = 0;
//    m_cPtr = (char*) m_memPtr;
//    m_size = other.m_size;
//    memcpy(m_memPtr, other.m_memPtr, other.m_size);
//}
//IoBuffer& IoBuffer::operator =(IoBuffer& other)
//{
//    if (&other == this) {
//        return *this;
//    }
//    m_capacity = other.m_capacity;
//    m_memPtr = malloc(m_capacity);
//    m_cPtr = (char*) m_memPtr;
//    m_size = other.m_size;
//    memcpy(m_memPtr, other.m_memPtr, other.m_size);
//    return *this;
//}
IoBuffer::IoBuffer(IoBuffer&& other) noexcept
{
    m_capacity = other.m_capacity;
    m_memPtr = other.m_memPtr;
    m_length = other.m_length;
    m_start_offset = other.m_start_offset;
    m_cPtr = (char*) m_memPtr;
//    m_size = other.m_size;
    other.init(m_capacity);
}
IoBuffer& IoBuffer::operator =(IoBuffer&& other) noexcept
{
    if (&other == this) {
        return *this;
    }
    m_memPtr = other.m_memPtr;
    m_cPtr = other.m_cPtr;
    m_capacity = other.m_capacity;
//    m_size = other.m_size;
    m_length = other.m_length;
    m_start_offset = other.m_start_offset;

    other.init(m_capacity);
    return *this;
}

IoBuffer::~IoBuffer()
{
    if( (m_memPtr != nullptr) && (m_capacity > 0) ){
        free(m_memPtr);
    }
}
void* IoBuffer::raw_memory()
{
    return m_memPtr;
}

void* IoBuffer::data()
{
    char* start_of_content = &((char*)m_memPtr)[m_start_offset];
    return (void*)start_of_content;
}
char* IoBuffer::get_first_char_ptr()
{
    char* start_of_content = &((char*)m_memPtr)[m_start_offset];
    return start_of_content;
}

std::size_t IoBuffer::size() const
{
    return m_length;
}
bool IoBuffer::empty()
{
    return (m_length == 0);
}
/**
 * capacity of the buffer - max value of size
*/
std::size_t IoBuffer::capacity() const
{
    return m_capacity;
}
/**
 * returns a pointer to the next available unused position in the buffer
*/
void* IoBuffer::nextAvailable()
{
    return (void*) (m_cPtr + m_length);
}
/**
 * Resets the buffer so that it is again an empty buffer
 */
void IoBuffer::clear()
{
    m_start_offset = 0;
    m_length = 0;
    m_cPtr[0] = (char)0;
}

void IoBuffer::append(void* data, std::size_t len)
{
    if ( ( (m_start_offset + m_length + len) >= m_capacity )  ) {
        std::size_t new_capacity = std::max(2 * m_capacity, m_capacity + 2*len);
        void* tmp = realloc(m_memPtr, new_capacity);
        m_memPtr = tmp;
        m_cPtr = (char*) m_memPtr;
        m_capacity = new_capacity;
    }
    void* na = nextAvailable();
    memcpy(na, data, len);
    m_length = m_length + len;
//    m_size = m_length;

    m_cPtr = (char*) m_memPtr;
}
void IoBuffer::append(std::string const & str)
{
    append((void*)str.c_str(), str.size());
}
void IoBuffer::append(std::string&& str)
{
    append((void*)str.c_str(), str.size());
}
void IoBuffer::append(std::string* str)
{
    append((void*)str->c_str(), str->size());
}
void IoBuffer::setSize(std::size_t n)
{
    m_length = n;
//    m_size = n;
}
void* IoBuffer::space_ptr()
{
    return &((char*)m_memPtr)[m_start_offset + m_length];
}
std::size_t IoBuffer::space_len() const
{
    size_t x = m_capacity - (m_start_offset + m_length);
    return x;
}
void IoBuffer::commit(std::size_t bytes_used)
{
    if(m_start_offset + m_length + bytes_used > m_capacity) {
        throw std::invalid_argument("bytes_used ");
    }
    m_length += bytes_used;
}
void IoBuffer::consume(std::size_t byte_count)
{
    if(byte_count > m_length) {
        throw std::invalid_argument("byte_count");
    }
    m_start_offset += byte_count;
    m_length -= byte_count;
    if(m_length == 0) {
        m_start_offset = 0;
    }

}
std::string IoBuffer::to_string()
{
    char* p = &(m_cPtr[m_start_offset]);
    std::string s(p, m_length);
    return s;
}
std::string IoBuffer::substr(std::size_t start, std::size_t len)
{
    char* p = &(m_cPtr[start]);
    std::string s(p, len);
    return s;
}

bool IoBuffer::contains(void* ptr)
{
    char* p = (char*) ptr;
    return contains(p);
}
bool IoBuffer::contains(const char* ptr)
{
    char* endPtr = m_cPtr + (long)m_capacity;
    char* sPtr = m_cPtr;
//    bool r1 = ptr <= endPtr;
//    bool r2 = ptr >= sPtr;
    bool r = ( ptr <= endPtr && ptr >= sPtr);
    return r;
}


} //namespace Marvin
