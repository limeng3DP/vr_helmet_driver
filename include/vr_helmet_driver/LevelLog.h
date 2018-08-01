#ifndef LEVELLOG_H
#define LEVELLOG_H

#ifndef LOGLEVEL
#define LOGLEVEL 2
#endif

#define LOG(_level, _levelstr, ...) do{ if(_level >= LOGLEVEL){ printf("[%s] ", (_levelstr)); printf(__VA_ARGS__); puts(""); } } while(0)

#if LOGLEVEL == 0
#define LOGD(...) LOG(0, "DD", __VA_ARGS__)
#else
#define LOGD(...)
#endif

#define LOGV(...) LOG(1, "VV", __VA_ARGS__)
#define LOGI(...) LOG(2, "II", __VA_ARGS__)
#define LOGW(...) LOG(3, "WW", __VA_ARGS__)
#define LOGE(...) LOG(4, "EE", __VA_ARGS__)

#ifdef _MSC_VER
#define snprintf _snprintf
#endif


#endif // LEVELLOG_H
