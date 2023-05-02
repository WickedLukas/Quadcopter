#include "DataLogger.h"
#include "common.h"

DataLogger::DataLogger(const char* name, uint32_t logInterval_us) : m_name{name}, m_logInterval_us{logInterval_us} {
    m_header[static_cast<uint16_t>(logId::sampleNumber)] = "Sample Number (" + String(1'000'000 / m_logInterval_us) + " samples per second)";
    m_header[static_cast<uint16_t>(logId::timeStamp)] = "UNIX Timestamp";
    m_header[static_cast<uint16_t>(logId::fMode)] = "fMode ()";
    m_header[static_cast<uint16_t>(logId::rtlState)] = "rtlState ()";
    m_header[static_cast<uint16_t>(logId::dt)] = "dt (us)";
    m_header[static_cast<uint16_t>(logId::ax)] = "ax ()";
    m_header[static_cast<uint16_t>(logId::ay)] = "ay ()";
    m_header[static_cast<uint16_t>(logId::az)] = "az ()";
    m_header[static_cast<uint16_t>(logId::ax_filtered)] = "ax_filtered ()";
    m_header[static_cast<uint16_t>(logId::ay_filtered)] = "ay_filtered ()";
    m_header[static_cast<uint16_t>(logId::az_filtered)] = "az_filtered ()";
    m_header[static_cast<uint16_t>(logId::roll_angle)] = "roll_angle (°)";
    m_header[static_cast<uint16_t>(logId::pitch_angle)] = "pitch_angle (°)";
    m_header[static_cast<uint16_t>(logId::yaw_angle)] = "yaw_angle (°)";
    m_header[static_cast<uint16_t>(logId::roll_angle_sp)] = "roll_angle_sp (°)";
    m_header[static_cast<uint16_t>(logId::pitch_angle_sp)] = "pitch_angle_sp (°)";
    m_header[static_cast<uint16_t>(logId::roll_rate)] = "roll_rate (°/s)";
    m_header[static_cast<uint16_t>(logId::pitch_rate)] = "pitch_rate (°/s)";
    m_header[static_cast<uint16_t>(logId::yaw_rate)] = "yaw_rate (°/s)";
    m_header[static_cast<uint16_t>(logId::roll_rate_sp)] = "roll_rate_sp (°/s)";
    m_header[static_cast<uint16_t>(logId::pitch_rate_sp)] = "pitch_rate_sp (°/s)";
    m_header[static_cast<uint16_t>(logId::yaw_rate_sp)] = "yaw_rate_sp (°/s)";
    m_header[static_cast<uint16_t>(logId::baroAltitudeRaw)] = "baroAltitudeRaw (m)";
    m_header[static_cast<uint16_t>(logId::baroAltitude)] = "baroAltitude (m)";
    m_header[static_cast<uint16_t>(logId::altitude)] = "altitude (m)";
    m_header[static_cast<uint16_t>(logId::altitude_sp)] = "altitude_sp (m)";
    m_header[static_cast<uint16_t>(logId::velocity_v)] = "velocity_v (m/s)";
    m_header[static_cast<uint16_t>(logId::velocity_x)] = "velocity_x (m/s)";
    m_header[static_cast<uint16_t>(logId::velocity_y)] = "velocity_y (m/s)";
    m_header[static_cast<uint16_t>(logId::velocity_v_sp)] = "velocity_v_sp (m/s)";
    m_header[static_cast<uint16_t>(logId::velocity_x_sp)] = "velocity_x_sp (m/s)";
    m_header[static_cast<uint16_t>(logId::velocity_y_sp)] = "velocity_y_sp (m/s)";
    m_header[static_cast<uint16_t>(logId::throttle_out)] = "throttle_out ()";
    m_header[static_cast<uint16_t>(logId::roll_rate_pTerm)] = "roll_rate_pTerm ()";
    m_header[static_cast<uint16_t>(logId::roll_rate_iTerm)] = "roll_rate_iTerm ()";
    m_header[static_cast<uint16_t>(logId::roll_rate_dTerm)] = "roll_rate_dTerm ()";
    m_header[static_cast<uint16_t>(logId::pitch_rate_pTerm)] = "pitch_rate_pTerm ()";
    m_header[static_cast<uint16_t>(logId::pitch_rate_iTerm)] = "pitch_rate_iTerm ()";
    m_header[static_cast<uint16_t>(logId::pitch_rate_dTerm)] = "pitch_rate_dTerm ()";
    m_header[static_cast<uint16_t>(logId::yaw_rate_pTerm)] = "yaw_rate_pTerm ()";
    m_header[static_cast<uint16_t>(logId::yaw_rate_iTerm)] = "yaw_rate_iTerm ()";
    m_header[static_cast<uint16_t>(logId::yaw_rate_dTerm)] = "yaw_rate_dTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_v_pTerm)] = "velocity_v_pTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_v_iTerm)] = "velocity_v_iTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_v_dTerm)] = "velocity_v_dTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_x_pTerm)] = "velocity_x_pTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_x_iTerm)] = "velocity_x_iTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_x_dTerm)] = "velocity_x_dTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_y_pTerm)] = "velocity_y_pTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_y_iTerm)] = "velocity_y_iTerm ()";
    m_header[static_cast<uint16_t>(logId::velocity_y_dTerm)] = "velocity_y_dTerm ()";
}

DataLogger::~DataLogger() {

    stop();
}

bool DataLogger::start() {

    if (m_started) {
        return false;
    }

    // initialise SD card and file system for SDIO mode
    if (!m_sd.begin(SdioConfig(FIFO_SDIO))) {
        // do not return error when SD card is missing
        if (m_sd.sdErrorCode() == 23 || m_sd.sdErrorCode() == 38) {
            return true;
        }
        DEBUG_PRINT(F("DataLogger: Failed to initialise SD card (")); DEBUG_PRINT(m_sd.sdErrorCode()); DEBUG_PRINTLN(")");
        return false;
    }

    // open root directory
    if (!m_root.open("/")) {
        DEBUG_PRINTLN(F("DataLogger: Failed to open root directory."));
        return false;
    }
    
    // iterate through all files inside root directory
    uint32_t minPrefix{ULONG_MAX}, maxPrefix{0}; // lowest and highest positive prefix number
    char firstLogFileName[m_maxFileNameSize];    // first log file name
    uint32_t logFileCount{0};                    // number of log files
    m_root.rewind();                             // start at the beginning of the root directory
    while (m_file.openNext(&m_root, O_RDONLY)) {
        // ignore directories
        if (m_file.isDir()) {
            continue;
        }

        // get file name
        char fileName[m_maxFileNameSize];
        m_file.getName(fileName, m_maxFileNameSize);

        // ignore files with unknown file extension
        const char* fileExtension{".csv"};
        if (strcmp(fileExtension, &fileName[strlen(fileName) - strlen(fileExtension)]) != 0) {
            continue;
        }
 
        uint32_t prefix{strtoul(fileName, nullptr, 10)};
        if (prefix > 0) {
            ++logFileCount;

            if (prefix < minPrefix) {
                minPrefix = prefix;

                // file name with the lowest non negative prefix number
                strcpy(firstLogFileName, fileName);
            }
            // highest non negative prefix number
            maxPrefix = max(maxPrefix, prefix);
        }
    }

    if (logFileCount >= m_maxLogFiles) {
        // remove log file with the lowest prefix number
        m_sd.remove(firstLogFileName);
    }

    char prefixString[m_maxFileNameSize];
    snprintf(prefixString, m_maxFileNameSize, "%lu", ++maxPrefix);

    // new log file name
    m_logFileName[0] = '\0';
    strcat(m_logFileName, prefixString);
    strcat(m_logFileName, " - ");
    strcat(m_logFileName, m_name);
    strcat(m_logFileName, m_suffix);

    // create new log file
    if (!m_file.open(m_logFileName, O_RDWR | O_CREAT | O_TRUNC)) {
        DEBUG_PRINTLN(F("DataLogger: Failed to create new log file."));
		return false;
	}

    // preallocate file to avoid huge delays searching for free clusters
	if (!m_file.preAllocate(m_maxLogFileSize)) {
		m_file.close();
        DEBUG_PRINTLN(F("DataLogger: Failed to preallocate file."));
		return false;
	}

	// initialise ring buffer
	m_rb.begin(&m_file);

    // write log header to ring buffer
    if (!writeLogHeader()) {
        m_rb.sync();
		m_file.truncate();
        m_file.close();
        DEBUG_PRINTLN(F("DataLogger: Failed to write log header."));
        return false;
	}

    m_sample = 0;
    m_start_ms = millis();
    m_logNow = true;
    m_started = true;
    return true;
}

bool DataLogger::stop() {

    bool success{true};

    if (m_started) {
        m_started = false;
        m_logNow = false;

        if (!m_rb.sync()) {
            success = false;
        }
        if (!m_file.truncate()) {
            success = false;
        }
        
	    m_file.rewind();
        if (!m_file.close()) {
            success = false;
        }
    }

    return success;
}

bool DataLogger::writeLogHeader() {

    // write header with comma separated values to ring buffer
    uint16_t id{0};
    if (!writeToRb(m_header[id])) {
        return false;
    }
    for (id = 1; id < m_columns; ++id) {
        if (!writeToRb("," + m_header[id])) {
            return false;
        }
    }

    // write line ending
    if (!writeToRb(m_lineEnding)) {
        return false;
    }

    return true;
}

bool DataLogger::writeLogLine() {

    if (!m_started) {
        return true;
    }

    if (m_logNow) {
        m_logNow = false;

        m_lastWriteLog_us = micros();
        m_line[static_cast<uint16_t>(logId::sampleNumber)] = String(m_sample++);                         // add sample number to log
        m_line[static_cast<uint16_t>(logId::timeStamp)] = String(m_lastWriteLog_us / 1000 - m_start_ms); // add time in ms to log

        // write line with comma separated values to ring buffer
        uint16_t id{0};
        if (!writeToRb(m_line[id])) {
            return false;
        }
        for (id = 1; id < m_columns; ++id) {
            if (!writeToRb("," + m_line[id])) {
                return false;
            }
        }

        // write line ending
        if (!writeToRb(m_lineEnding)) {
            return false;
        }

        // check for maximum log file size
	    if ((m_file.curPosition() + m_rb.bytesUsed()) >= m_maxLogFileSize) {
		    // log file is full
		    return false;
	    }
    }
    else {
        // log every log interval
        if ((micros() - m_lastWriteLog_us) >= m_logInterval_us) {
            m_logNow = true;
        }
    }

    // transfer data from ring buffer to SD card
	if ((m_rb.bytesUsed() >= m_sectorSize) && !m_file.isBusy()) {
		// only one sector can be written before busy wait
        size_t written{m_rb.writeOut(m_sectorSize)};

		if (written != m_sectorSize) {
			return false;
		}
	}

    return true;
}

bool DataLogger::writeToRb(const String &str) {

    const size_t strLength{str.length()};
    if ((m_rb.bytesFree() - strLength) < 0) {
        // not enough free space in ring buffer
        return false;
    }

    const char* cstr{str.c_str()};
    size_t written{0};
    int writeError;
    do {
        written += m_rb.write(&cstr[written], strLength - written);
    } while ((written != strLength) && !(writeError = m_rb.getWriteError()));

    if (writeError) {
        // writing to ring buffer failed
        return false;
    }

    return true;
}