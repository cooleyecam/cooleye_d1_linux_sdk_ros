#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

#include "logmsg.h"
#include "cpu_set.h"

CCpuSet * & CCpuSet::instance()
{
    static CCpuSet * _instance = NULL;
    if( NULL == _instance)
    {
        _instance = new CCpuSet;
    }
    return _instance;
}
void CCpuSet::release()
{
    CCpuSet * & pInstance = CCpuSet::instance();
    delete pInstance;
    pInstance = NULL;
}

CCpuSet::CCpuSet()
{
    m_nCpuSetFlag = 0;
    m_nCpuOccNum = 0;
    m_nCpuPoolSize = 0;
    memset(m_aCpuPool, 0, sizeof(m_aCpuPool));
    pthread_mutex_init(&m_Mutex, NULL);
}

CCpuSet::~CCpuSet()
{
}

int CCpuSet::SetCpu(char *pThreadName, int para)
{
    int nRet = 0;
    pthread_mutex_lock(&m_Mutex);
    if (0 != m_nCpuSetFlag && m_nCpuOccNum < m_nCpuPoolSize)
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);
        int nCpuId = m_aCpuPool[m_nCpuOccNum++];
        CPU_SET(nCpuId, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
        {
            nRet = -1;
            if (-1 != para)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "Set CPU[%d] affinity failed! %s%d \n", nCpuId, pThreadName, para);
            }
            else
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "Set CPU[%d] affinity failed! %s \n", nCpuId, pThreadName);
            }

        }
        else
        {
            if (-1 != para)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "Set CPU[%d] affinitysucceed! %s%d \n", nCpuId, pThreadName, para);
            }
            else
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "Set CPU[%d] affinitysucceed! %s \n", nCpuId, pThreadName);
            }
        }
    }
    pthread_mutex_unlock(&m_Mutex);

    return nRet;
}

int CCpuSet::SetCpu(int nCpuId, char *pThreadName, int para)
{
    int nRet = 0;
    pthread_mutex_lock(&m_Mutex);
    if (0 != m_nCpuSetFlag)
    {
        cpu_set_t mask;
        CPU_ZERO(&mask);

        CPU_SET(nCpuId, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) == -1)
        {
            nRet = -1;
            if (-1 != para)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "Set CPU[%d] affinity failed! %s%d \n", nCpuId, pThreadName, para);
            }
            else
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_ERROR, "Set CPU[%d] affinity failed! %s \n", nCpuId, pThreadName);
            }

        }
        else
        {
            if (-1 != para)
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "Set CPU[%d] affinitysucceed! %s%d \n", nCpuId, pThreadName, para);
            }
            else
            {
                WRITE_LOG(LOGMSG_ALL, LOGMSG_LEVEL_INFO, "Set CPU[%d] affinitysucceed! %s \n", nCpuId, pThreadName);
            }
        }
    }
    pthread_mutex_unlock(&m_Mutex);

    return nRet;
}


