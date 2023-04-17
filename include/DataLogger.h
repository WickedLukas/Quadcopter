#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <SdFat.h>
#include <RingBuf.h>

#include <stdint.h>

#include <array>

enum class logId : uint16_t
{
    sample, time,
    fMode, rtlState,
    dt,
    ax, ay, az,
    ax_filtered, ay_filtered, az_filtered,
    roll_angle, pitch_angle, yaw_angle, roll_angle_sp, pitch_angle_sp,
    roll_rate, pitch_rate, yaw_rate, roll_rate_sp, pitch_rate_sp, yaw_rate_sp,
    a_d_rel,
    baroAltitudeRaw, baroAltitude,
    altitude, altitude_sp,
    velocity_v, velocity_x, velocity_y, velocity_v_sp, velocity_x_sp, velocity_y_sp,
    throttle_out,
    last
};

class DataLogger {

public:
    DataLogger(const char* name, uint32_t logInterval_us = 10'000) : m_name{name}, m_logInterval_us{logInterval_us} {
    }

    ~DataLogger();

    bool start();
    bool stop();

    // add log value to line
    template<typename T>
    typename std::enable_if<std::is_integral<T>::value>::type log(logId id, const T value) {
        if (!m_started || !m_logNow) {
            return;
        }
        m_lineValues[static_cast<uint16_t>(id)] = String(value);
    }
    template<typename T>
    typename std::enable_if<!std::is_integral<T>::value>::type log(logId id, const T value, uint16_t digits = 2) {
        if (!m_started || !m_logNow) {
            return;
        }
        m_lineValues[static_cast<uint16_t>(id)] = String(value, digits);
    }

    bool writeLogLine(); // write log line to ring buffer

private:
    bool writeToRb(const String &str); // write string to ring buffer

    const char* m_name; // name used inside log file name
    
    static const uint32_t m_sectorSize{512};                                             // sector size which can be efficiently written without waiting
    static const uint32_t m_ringBufSize{10 * m_sectorSize};                              // ring buffer size
    static const size_t m_maxFileNameSize{30};                                           // maximum size of file name
    const uint32_t m_logInterval_us;                                                     // interval between log samples in microseconds
    const uint32_t m_maxLogFileSize{m_sectorSize * 1'000'000 / m_logInterval_us * 3600}; // size to log samples of sector size every log interval for one hour in bytes (~176 MByte)
    const uint32_t m_maxLogFiles{20};                                                    // maximum number of log files

    char m_logFileName[m_maxFileNameSize];    // log file name
    const char* m_logFileSuffix{"__log.csv"}; // log file suffix
    const String m_lineEnding{"\r\n"};        // log file line ending

    SdFs m_sd;
    FsFile m_root;
    FsFile m_file;
    RingBuf<FsFile, m_ringBufSize> m_rb; // ring buffer for file type FsFile

    bool m_started{false}; // true if data logger is started
    bool m_logNow{false};  // logging flag used to efficiently log with the specified log interval
    
    uint32_t m_lastWriteLog_us{0}; // time when the last log was written in microseconds
    uint32_t m_start_ms{0};        // data logging start time in ms
    uint32_t m_sample{0};          // logging sample number

    static const uint16_t m_numLineValues{static_cast<uint16_t>(logId::last)}; // number of log values in one line 
    std::array<String, m_numLineValues> m_lineValues{};                        // array which contains log values for one line
};

#endif