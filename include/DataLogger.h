#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <SdFat.h>
#include <RingBuf.h>

#include <stdint.h>

#include <array>

enum class logId : uint16_t
{
    first,
    second,
    third,
    fourth,
    fifth,
    sixth,
    seventh,
    eight,
    ninth,
    tenth,
    last
};

class DataLogger {

public:
    DataLogger(const char* version) : m_version(version) {
    }

    ~DataLogger();

    bool start();
    bool stop();

    void log(logId id, const String &value); // add log value to line data
    bool writeLogLine();                     // write log line to ring buffer

private:
    bool writeToRb(const String &str); // write string to ring buffer

    static const uint32_t m_logInterval_us{10'000};                                             // interval between log samples in microseconds
    static const uint32_t m_sectorSize{512};                                                    // sector size which can be efficiently written without waiting
    static const uint32_t m_ringBufSize{2 * m_sectorSize};                                      // ring buffer size
    static const uint32_t m_maxLogFileSize{m_sectorSize * 1'000'000 / m_logInterval_us * 3600}; // size to log samples of sector size every log interval for one hour in bytes (~176 MByte)
    static const size_t m_maxFileNameSize{30};                                                  // maximum size of file name
    static const uint32_t m_maxLogFiles{20};                                                    // maximum number of log files

    char m_logFileName[m_maxFileNameSize];    // log file name
    const char* m_logFileSuffix{"__log.csv"}; // log file suffix
    const String m_lineEnding{"\r\n"};        // log file line ending

    SdFs m_sd;
    FsFile m_root;
    FsFile m_file;
    RingBuf<FsFile, m_ringBufSize> m_rb; // ring buffer for file type FsFile

    const char* m_version; // version number is used inside log file name

    bool m_started{false}; // true if data logger is started
    bool m_logNow{false};  // logging flag used to efficiently log with the specified log interval

    static const uint16_t m_numLineValues{static_cast<uint16_t>(logId::last)}; // number of log values in one line 
    std::array<String, m_numLineValues> m_lineValues{};                        // array which contains log values for one line
};

#endif