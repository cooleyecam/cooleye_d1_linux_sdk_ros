#ifndef _CPU_SET_H_
#define _CPU_SET_H_

#define CPU_SET_MAX 32

class CCpuSet
{
public:
    static CCpuSet * & instance();
    static void release();

    int SetCpu(const char *pThreadName, int para = -1);
    int SetCpu(int nCpuId, const char *pThreadName, int para = -1);

private:
    CCpuSet();
    ~CCpuSet();

public:
    int m_nCpuSetFlag;
    int m_nCpuOccNum;
    int m_nCpuPoolSize;
    int m_aCpuPool[CPU_SET_MAX];

private:
    pthread_mutex_t m_Mutex;
};

#endif
