#ifndef THREADSAFE_QUEUE_H
#define THREADSAFE_QUEUE_H
#include <queue>
#include <mutex>
#include <condition_variable>
#include <initializer_list>

/*
 * 线程安全队列
 * T为队列元素类型
 * 因为有std::mutex和std::condition_variable类成员,所以此类不支持复制构造函数也不支持赋值操作符(=)
 * */

/* 编译器支持c++11，经测试发现linux下gcc4.8.5支持，4.4.7不支持 */
//#define COMPILER_SUPPORTS_CPP11

#define THREADSAFE_QUEUE_QUENTITY_MAX  10000

template<typename T>
class threadsafe_queue{
private:

    //对列最大缓存数量
    unsigned int quentity_max;

    // data_queue访问信号量
    mutable std::mutex mut;
    mutable std::condition_variable data_cond;
#ifdef COMPILER_SUPPORTS_CPP11
    using queue_type = std::queue<T>;
#else
	typedef  std::queue<T> queue_type;
#endif
    queue_type data_queue;
public:
#ifdef COMPILER_SUPPORTS_CPP11
    using value_type= typename queue_type::value_type;
	using container_type = typename queue_type::container_type;
#else
	typedef typename queue_type::value_type value_type;
	typedef typename queue_type::container_type container_type;
#endif
    threadsafe_queue(){
        quentity_max = THREADSAFE_QUEUE_QUENTITY_MAX;
    }

    threadsafe_queue(const threadsafe_queue&)=delete;
    threadsafe_queue& operator=(const threadsafe_queue&)=delete;

    /*
     * 使用迭代器为参数的构造函数,适用所有容器对象
     * */
    template<typename _InputIterator>
    threadsafe_queue(_InputIterator first, _InputIterator last){
        for(auto itor=first;itor!=last;++itor){
            data_queue.push(*itor);
        }
    }

    explicit threadsafe_queue(const container_type &c):data_queue(c){}
    /*
     * 使用初始化列表为参数的构造函数
     * */
    threadsafe_queue(std::initializer_list<value_type> list):threadsafe_queue(list.begin(),list.end()){
    }

    void init(unsigned int quentity){
        quentity_max = quentity;
    }

    /*
     * 将元素加入队列，超过队列最大容量返回false
     * */
    bool push(const value_type &new_value){
        std::lock_guard<std::mutex>lk(mut);
        if (data_queue.size() >= quentity_max)
            return false;

        data_queue.push(std::move(new_value));
        data_cond.notify_one();

        return true;
    }

    /*
     * 将元素加入队列，超过队列最大容量或者指定数量时，先移除一个元素再入队
     * */
    int push(const value_type &new_value, value_type &old_value, unsigned int _quentity = THREADSAFE_QUEUE_QUENTITY_MAX){
        int ret = 1;
        std::lock_guard<std::mutex>lk(mut);

        unsigned int quentity = _quentity < quentity_max ? _quentity : quentity_max;
        if (data_queue.size() >= quentity && quentity > 0)
        {
            old_value=std::move(data_queue.front());
            data_queue.pop();
            ret = 0;
        }

        data_queue.push(std::move(new_value));
        data_cond.notify_one();
        return ret;
    }

#ifdef COMPILER_SUPPORTS_CPP11
    /*
     * 从队列中弹出一个元素,如果队列为空就阻塞
     * */
    value_type wait_and_pop(){
        std::unique_lock<std::mutex>lk(mut);
        data_cond.wait(lk,[this]{return !this->data_queue.empty();});
        auto value=std::move(data_queue.front());
        data_queue.pop();
        return value;
    }
#endif
    /*
     * 从队列中弹出一个元素,如果队列为空返回false
     * */
    bool try_pop(value_type& value){
        std::lock_guard<std::mutex>lk(mut);
        if(data_queue.empty())
            return false;
        value=std::move(data_queue.front());
        data_queue.pop();
        return true;
    }

    /*
     * 从队列中取一个元素,但不出队,如果队列为空返回false
     * */
    bool try_front(value_type& value){

        std::lock_guard<std::mutex>lk(mut);

        if(data_queue.empty())

            return false;

        value=std::move(data_queue.front());

        return true;

    }

    /*
     * 返回队列是否为空
     * */
    auto empty() const->decltype(data_queue.empty()) {
        std::lock_guard<std::mutex>lk(mut);
        return data_queue.empty();
    }
    /*
     * 返回队列中元素数个
     * */
    auto size() const->decltype(data_queue.size()){
        std::lock_guard<std::mutex>lk(mut);
        return data_queue.size();
    }
}; /* threadsafe_queue */

#endif /* COMMON_SOURCE_CPP_THREADSAFE_QUEUE_H_ */
