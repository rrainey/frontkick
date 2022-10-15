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
#include "LogPlayback.h"

int i;

static LogPlayback * g_pPlayback = NULL;

static void errorHandler()
{
}

static void handleTimeHack()
{
    g_pPlayback->HandleTimeHack();
}

void LogPlayback::HandleTimeHack()
{
  unsigned long ulValue;

  //Serial.println(F("LogPlayback::HandleTimeHack()"));

  m_parser.getArg(0, ulValue);

  // Any time we see a $PENV, $PIMU, or $PTH record, we potentially wait to allow
  // appropriate time to pass before injecting all previous data into the Log pipeline.

  m_ulNextSendTime_ms = m_ulRealStartTime_ms + (ulValue - m_ulLogStartTime_ms);
  m_eMode = PLAYBACK_WAIT_MODE;

  //Serial.println(F("LogPlayback entering wait mode"));

}

LogPlayback::LogPlayback(SDFile &logFile, LogDataCb cb) {

    m_eMode = PLAYBACK_BUFFER_MODE;

    m_ulRealStartTime_ms = millis();
    m_ulLogStartTime_ms = 0;

    m_ulNextSendTime_ms = 0;

    m_parser.addHandler("PTH", handleTimeHack);
    m_parser.addHandler("PIMU", handleTimeHack);
    m_parser.addHandler("PENV", handleTimeHack);
    m_parser.setHandleCRC(false);

    m_pBufferEnd = m_cBuffer;
    m_nRemaining = CHUNK_BUFFER_SIZE;
    m_logFile = logFile;
    m_onDataCb = cb;

    g_pPlayback = this;
}

LogPlayback::~LogPlayback() {

}

void LogPlayback::EndPlayback(void) {
  if (m_eMode != PLAYBACK_COMPLETE) {
    int nSize = m_pBufferEnd - m_cBuffer;
    if (nSize > 0) {
      m_onDataCb( m_cBuffer, nSize );
      m_pBufferEnd = m_cBuffer;
      m_nRemaining = CHUNK_BUFFER_SIZE;
    }
    m_logFile.close();
    m_eMode = PLAYBACK_COMPLETE;
  }
}

/*
 * Call this from loop() whenever playing back a logfile.
 * Returns false when the playback has ended. Caller should respond by destroying
 * the instance.
 */
bool LogPlayback::TimeStep() {

  bool bResult = true;

  //Serial.println(F("LogPlayback::TimeStep()"));

  // if in "buffer"" mode, read till next PTH/PENV/PIMU record appears in the log, record time of 
  // arrival of this record, switch to "wait" mode
  while (m_eMode == PLAYBACK_BUFFER_MODE && m_logFile.available()) {
    if (m_nRemaining > 0 ) {
      uint8_t c = m_logFile.read();
      *m_pBufferEnd++ = c;
      --m_nRemaining;
      m_parser << c;
    }
    else {
      Serial.println(F("LogPlayback buffer full; stopping"));
      while(1) delay(10);
    }    
  }

  // if in "wait" mode; if current millis() >= m_ulNextSendTime_ms, send all buffered bytes and
  // return to "buffer" mode.
  if (m_eMode == PLAYBACK_WAIT_MODE) {
    if (millis() >= m_ulNextSendTime_ms) {
      m_onDataCb( m_cBuffer, m_pBufferEnd - m_cBuffer );
      m_pBufferEnd = m_cBuffer;
      m_nRemaining = CHUNK_BUFFER_SIZE;
      m_eMode = PLAYBACK_BUFFER_MODE;
    }
  }

  if (! m_logFile.available()) {
    EndPlayback();
    bResult = false;
  }

  return bResult;
}
