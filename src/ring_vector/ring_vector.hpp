#include <iostream>
#include <vector>

using std::vector;

template <typename T>
class RING_VECTOR
{
private:
    vector<T> data;
    uint32_t head_index;

public:
    RING_VECTOR();
    void push_back(const T& data_);
    void push_pop(const T& data_);
    T& at(uint32_t index);
    const T& back();
    void clear();
    uint32_t size();
};

template <typename T>
inline RING_VECTOR<T>::RING_VECTOR()
{
    head_index = 0;
}

template <typename T>
inline void RING_VECTOR<T>::push_back(const T& data_)
{
    data.push_back(data_);
}

//data不能为空,且使用push_pop之后不能再push_back
template <typename T>
inline void RING_VECTOR<T>::push_pop(const T& data_)
{
    if(data.size()==0)
    {
        return ;
    }
    data.at(head_index) = data_;
    head_index++;
    if(head_index >= data.size())
    {
        head_index = 0;
    }
}

template <typename T>
inline T &RING_VECTOR<T>::at(uint32_t index_)
{
    if(index_>= data.size())
    {
        return data.at(index_);
    }
    uint32_t t_index = head_index + index_;
    uint32_t final_index = t_index >= data.size() ? (t_index-data.size()) : t_index;
    return data.at(final_index);
}

template <typename T>
inline const T &RING_VECTOR<T>::back()
{
    if(head_index==0)
    {
        return data.back();
    }
    else
    {
        return data.at(head_index-1);
    }
    // TODO: insert return statement here
}

template <typename T>
inline void RING_VECTOR<T>::clear()
{
    data.clear();
    head_index = 0;
}

template <typename T>
inline uint32_t RING_VECTOR<T>::size()
{
    return data.size();
}
