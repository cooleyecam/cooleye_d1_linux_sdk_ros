
#ifndef _LOGMSG_HDR_
#define _LOGMSG_HDR_
#include <sys/sysinfo.h>
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <vector>
using std::vector;

#define FILE_NAME_LENGTH_MAX    256
#define SINGLE_LOG_SIZE_MAX     3072

enum
{
	LOGMSG_NULL = 0,
	LOGMSG_SHELL,
	LOGMSG_FILE,
	LOGMSG_ALL,
};


enum
{
    LOGMSG_LEVEL_NULL = 0,
	LOGMSG_LEVEL_DEBUG,
	LOGMSG_LEVEL_INFO,
	LOGMSG_LEVEL_WARN,
	LOGMSG_LEVEL_ERROR,
};


struct LogFileInfo
{
    char cName[FILE_NAME_LENGTH_MAX];
};

typedef vector<LogFileInfo> vLogFile;

class LogMsg
{
public:
    static LogMsg * & instance();
	static void release();

    int init(int nQuantity, int nSize, int nLevel = LOGMSG_LEVEL_ERROR);
    int fini();
    int writeLog(int nLogFlag, int nLevel);
    const char * levelStr(int level);

private:
    LogMsg(){};
    virtual ~LogMsg(){};

public:
    char m_cBuff[SINGLE_LOG_SIZE_MAX];
    unsigned int m_unLogIndex;

private:
    int m_nMaxFileQuantity;
    int m_nMaxFileSize;
    int m_nLevel;
    FILE * m_pLog;
    time_t m_tTime;
    int m_nCount;
    char m_cPath[FILE_NAME_LENGTH_MAX];
    char m_cFileName[FILE_NAME_LENGTH_MAX];
    vLogFile m_vFileList;
    pthread_mutex_t m_Mutex;
};


#define INIT_LOG(quantity, size, level) \
    LogMsg::instance()->init(quantity, size, level)


#define WRITE_LOG(logFlag, level,strFormat, args...) \
{\
    time_t _t = time(NULL);  \
    tm _struTm;   \
    char _cTimeStr[64];  \
    strftime( _cTimeStr, 64, "%Y/%m/%d %H:%M:%S", localtime_r(&_t, &_struTm) );  \
    int _nBuffLen = snprintf(LogMsg::instance()->m_cBuff, SINGLE_LOG_SIZE_MAX-1, \
        "[0x%04x]%-16s[%s]", LogMsg::instance()->m_unLogIndex, _cTimeStr, LogMsg::instance()->levelStr(level)); \
    snprintf(&LogMsg::instance()->m_cBuff[_nBuffLen], SINGLE_LOG_SIZE_MAX-1-_nBuffLen, strFormat, ##args);\
    LogMsg::instance()->writeLog(logFlag, level);\
}

#define FINI_LOG() \
    LogMsg::instance()->fini()

#endif /*_LOGMSG_HDR_*/
