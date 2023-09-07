#ifndef DATALOGGER_H_
#define DATALOGGER_H_

#include <SdFat.h>
#include <RingBuf.h>

#include <stdint.h>

#include <array>

enum class logId : uint16_t
{
    sampleNumber, timeStamp,
    fMode, rtlState,
    dt,
    ax, ay, az,
    ax_filtered, ay_filtered, az_filtered,
    roll_angle, pitch_angle, yaw_angle, roll_angle_sp, pitch_angle_sp,
    roll_rate, pitch_rate, yaw_rate, roll_rate_sp, pitch_rate_sp, yaw_rate_sp,
    baroAltitudeRaw, baroAltitude,
    altitude, altitude_sp,
    velocity_v, velocity_x, velocity_y, velocity_v_sp, velocity_x_sp, velocity_y_sp,
    throttle_out,
    roll_rate_pTerm, roll_rate_iTerm, roll_rate_dTerm,
    pitch_rate_pTerm, pitch_rate_iTerm, pitch_rate_dTerm,
    yaw_rate_pTerm, yaw_rate_iTerm, yaw_rate_dTerm,
    velocity_v_pTerm, velocity_v_iTerm, velocity_v_dTerm,
    velocity_x_pTerm, velocity_x_iTerm, velocity_x_dTerm,
    velocity_y_pTerm, velocity_y_iTerm, velocity_y_dTerm,
    distance,
    velocity,
    distance_x, distance_y,
    distance_yaw,
    bearing,
    heading,
    headingCorrection,
    last
};

class DataLogger {

public:
    DataLogger(const char* name, uint32_t logInterval_us = 10'000);

    ~DataLogger();

    bool start();
    bool stop();

    template<typename T>
    typename std::enable_if<std::is_integral<T>::value>::type log(logId id, const T value) {
        if (!m_started || !m_logNow) {
            return;
        }
        m_line[(uint16_t) id] = String(value);
    }
    template<typename T>
    typename std::enable_if<!std::is_integral<T>::value>::type log(logId id, const T value, uint16_t digits = 2) {
        if (!m_started || !m_logNow) {
            return;
        }
        m_line[(uint16_t) id] = String(value, digits);
    }

    bool writeLogHeader();
    bool writeLogLine();

private:
    bool writeToRb(const String &str); // write string to ring buffer

    const char* m_name; // name used inside log file name
    
    static const uint32_t m_sectorSize{512};                                                 // sector size which can be efficiently written without waiting
    static const uint32_t m_ringBufSize{20 * m_sectorSize};                                  // ring buffer size
    static const size_t m_maxFileNameSize{255};                                              // maximum size of file name
    const uint32_t m_logInterval_us;                                                         // interval between log samples in microseconds
    const uint32_t m_maxLogFileSize{2 * m_sectorSize * 1'000'000 / m_logInterval_us * 3600}; // size to log samples of 2 * sector size every log interval for one hour in bytes (~369 MByte)
    const uint64_t m_maxLogFiles{32'000'000'000u / m_maxLogFileSize};                        // maximum number of log files

    char m_logFileName[m_maxFileNameSize]; // log file name
    const char* m_suffix{".csv"};          // log file suffix
    const String m_lineEnding{"\r\n"};     // log file line ending

    SdFs m_sd;
    FsFile m_root;
    FsFile m_file;
    RingBuf<FsFile, m_ringBufSize> m_rb; // ring buffer for file type FsFile

    bool m_started{false}; // true if data logger is started
    bool m_logNow{false};  // logging flag used to efficiently log with the specified log interval
    
    uint32_t m_lastWriteLog_us{0}; // time when the last log was written in microseconds
    uint32_t m_start_ms{0};        // data logging start time in ms
    uint32_t m_sample{0};          // logging sample number

    static const uint16_t m_columns{(uint16_t) logId::last}; // number of columns per line
    std::array<String, m_columns> m_header{};                // log header
    std::array<String, m_columns> m_line{};                  // log line
};

#endif