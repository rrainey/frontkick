/*
 * This file is part of the Kick distribution (https://github.com/rrainey/frontkick
 * Copyright (c) 2022 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef LOG_PLAYBACK_H
#define LOG_PLAYBACK_H

#include <Arduino.h>
#include <functional>
#include <SD.h>
#include <FS.h>

#include "NMEAParserEx.h"

#define CHUNK_BUFFER_SIZE 2048

/*
 * pBuffer - pointer to data bytes
 * nSize - number of bytes in the buffer; -1 indicates playback has reached the 
           end of the log file. If -1 is sent, the callback should respond by invoking 
           EndPlayback() as part of whetever other processing is required.
 */
typedef std::function<void(uint8_t *buffer, size_t size)> LogDataCb;

/*
 * This class plays-back the contents of a log file as if it were arriving in real time.
 * Once StartPlayback is invoked, TimeStep() should be called from loop(). The
 * playback class will use millis() time on the system to pace the sequential replay
 * records from the specified log stream. The callback supplied in OnData will be invoked
 * as to simulate the arrival of data.
 */
class LogPlayback {
    public:
        LogPlayback(SDFile &logFile, LogDataCb cb);
        virtual ~LogPlayback();
        void EndPlayback(void);
        bool TimeStep(void);

        void HandleTimeHack(void);
        void HandleENVRecord(void);
        void HandleIMURecord(void);

    protected:
        typedef enum {
            PLAYBACK_BUFFER_MODE,   // Accumulate bytes to
            PLAYBACK_WAIT_MODE,
            PLAYBACK_COMPLETE
        } PlaybackMode;

        PlaybackMode m_eMode;
        unsigned long m_ulRealStartTime_ms; // millis() time when playback started
        unsigned long m_ulLogStartTime_ms;  // usually "0"
        unsigned long m_ulNextSendTime_ms;  // millis() time to send next chunk
        NMEAParserEx<10> m_parser;
        uint8_t m_cBuffer[CHUNK_BUFFER_SIZE];
        uint8_t * m_pBufferEnd;
        size_t m_nRemaining;
        SDFile m_logFile;
        LogDataCb m_onDataCb;

};

#endif
