#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include "logmsg.h"


LogMsg * & LogMsg::instance()
{
	static LogMsg * _instance = NULL;
	if( NULL == _instance)
	{
		_instance = new LogMsg;
	}
	return _instance;
}
void LogMsg::release()
{
    LogMsg * & pInstance = LogMsg::instance();
    delete pInstance;
    pInstance = NULL;
}

int LogMsg::init(int nQuantity, int nSize, int nLevel)
{
    m_nMaxFileQuantity = nQuantity;
    m_nMaxFileSize = nSize * 1024 * 1024;
    m_nLevel = nLevel;
    m_pLog = NULL;
    m_nCount = 0;
    m_tTime = 0;

    strcpy(m_cPath, "./runlog");

    if(-1 == access(m_cPath, R_OK|W_OK))
    {
        if(mkdir(m_cPath, 0755))
            printf("Make log dir[%s] error\n", m_cPath);
    }

    pthread_mutex_init(&m_Mutex, NULL);

    return 0;
}

int LogMsg::fini()
{
}

const char * LogMsg::levelStr(int level)
{
	switch(level) {
	case LOGMSG_LEVEL_DEBUG:
		return "DEBUG";
	case LOGMSG_LEVEL_INFO:
		return "INFO ";
	case LOGMSG_LEVEL_WARN:
		return "WARN ";
	case LOGMSG_LEVEL_ERROR:
		return "ERROR";
	default:
		return "UNKNOW";
	}
}


int LogMsg::writeLog(int nLogFlag, int nLevel)
{
    if (m_nLevel > nLevel)
        return 0;

    int nRet = 0;
    pthread_mutex_lock(&m_Mutex);

    if (NULL == m_pLog)
    {
        time_t tT = time(NULL);
        if (m_tTime != tT)
        {
            m_tTime = tT;
            m_nCount = 0;
        }

        tm * stT = localtime(&tT);
        sprintf(m_cFileName, "%s/runlog_%04d%02d%02d%02d%02d%02d_%03d.log",
                m_cPath, 1900 + stT->tm_year, 1 + stT->tm_mon, stT->tm_mday, stT->tm_hour, stT->tm_min, stT->tm_sec,m_nCount++);

        LogFileInfo stLogFileInfo = {0};
        strcpy(stLogFileInfo.cName, m_cFileName);
        m_vFileList.push_back(stLogFileInfo);
        if (m_vFileList.size() > m_nMaxFileQuantity)
        {
            vLogFile::iterator it = m_vFileList.begin();
            rename(it->cName, m_cFileName);
            m_vFileList.erase(it);
        }

        m_pLog = fopen( m_cFileName, "w" );
    }

    if (LOGMSG_SHELL & nLogFlag)
    {
        fprintf(stdout, m_cBuff);
    }

    if(NULL == m_pLog)
    {
        nRet = -1;
        goto _quit;
    }

    if (LOGMSG_FILE & nLogFlag)
    {
        fprintf(m_pLog, "%s", m_cBuff);
        fflush(m_pLog);
    }

    m_unLogIndex++;

    if (ftell(m_pLog) >= m_nMaxFileSize)
    {
        fclose(m_pLog);
        m_pLog = NULL;
    }

_quit:
    pthread_mutex_unlock(&m_Mutex);

    return nRet;
}

