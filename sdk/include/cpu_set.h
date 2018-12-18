#ifndef _CPU_SET_H_
#define _CPU_SET_H_

class CCpuSet
{
public:
    static CCpuSet * & instance();
    static void release();

    int SetCpu(char *pThreadName, int para = -1);
    int SetCpu(int nCpuId, char *pThreadName, int para = -1);

private:
    CCpuSet();
    ~CCpuSet();

public:
    int m_nCpuSetFlag;
    int m_nCpuOccNum;
    int m_nCpuPoolSize;
    int m_aCpuPool[32];

private:
    pthread_mutex_t m_Mutex;
};

#endif
