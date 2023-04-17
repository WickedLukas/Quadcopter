#include "DataLogger.h"

DataLogger::~DataLogger() {

    stop();
}

bool DataLogger::start() {

    if (m_started) {
        return false;
    }

    // initialize SD card and file system for SDIO mode
    if (!m_sd.begin(SdioConfig(FIFO_SDIO))) {
        // do not return error when SD card is missing
        if (m_sd.sdErrorCode() == 23) {
            return true;
        }
        return false;
    }

    // open root directory
    if (!m_root.open("/")) {
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
 
        // find the highest positive prefix number and the file name with the lowest positive prefix number
        char prefixString[m_maxFileNameSize];
        strcpy(prefixString, fileName);
        strtok(prefixString, "-");
        uint32_t prefix{strtoul(prefixString, nullptr, 10)};
        if (prefix > 0) {
            ++logFileCount;

            if (prefix < minPrefix) {
                minPrefix = prefix;

                // file name with the lowest positive prefix number
                strcpy(firstLogFileName, fileName);
            }

            maxPrefix = max(maxPrefix, prefix);
        }
    }

    if (logFileCount >= m_maxLogFiles) {
        // remove the log file with the lowest prefix number
        m_sd.remove(firstLogFileName);
    }

    char prefixString[m_maxFileNameSize];
    snprintf(prefixString, m_maxFileNameSize, "%lu", ++maxPrefix);

    // new log file name
    m_logFileName[0] = '\0';
    strcat(m_logFileName, prefixString);
    strcat(m_logFileName, "__");
    strcat(m_logFileName, m_name);
    strcat(m_logFileName, m_logFileSuffix);

    // create new log file
    if (!m_file.open(m_logFileName, O_RDWR | O_CREAT | O_TRUNC)) {
		return false;
	}

    // preallocate file to avoid huge delays searching for free clusters
	if (!m_file.preAllocate(m_maxLogFileSize)) {
		m_file.close();
		return false;
	}

	// initialise ring buffer
	m_rb.begin(&m_file);

    m_sample = 0;
    m_start_ms = millis();
    return (m_started = true);
}

bool DataLogger::stop() {

    bool success{true};

    if (m_started) {
        m_started = false;
        m_logNow = true;

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

bool DataLogger::writeLogLine() {

    if (!m_started) {
        return true;
    }

    if (m_logNow) {
        m_logNow = false;

        // write line with comma separated values to ring buffer
        uint16_t id{0};
        if (!writeToRb(m_lineValues[id])) {
            return false;
        }
        for (id = 1; id < m_numLineValues; ++id) {
            if (!writeToRb("," + m_lineValues[id])) {
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
        uint32_t writeLog_us{micros()};

        // log every log interval
        if ((writeLog_us - m_lastWriteLog_us) >= m_logInterval_us) {
            m_logNow = true;
            m_lastWriteLog_us = writeLog_us;
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

    log(logId::sample, ++m_sample);          // add sample number to log
    log(logId::time, millis() - m_start_ms); // add time in ms to log

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