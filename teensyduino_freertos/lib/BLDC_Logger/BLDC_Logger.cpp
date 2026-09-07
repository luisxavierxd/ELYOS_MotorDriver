#include "BLDC_Logger.h"
#include <SD.h>

BLDC_Logger::BLDC_Logger() : file_handle_(nullptr), sd_detected_(false), sample_id_(0) {}

BLDC_Logger::~BLDC_Logger() {
    if (file_handle_) {
        File *f = static_cast<File*>(file_handle_);
        f->close();
        delete f;
        file_handle_ = nullptr;
    }
}

void BLDC_Logger::init()
{
    sd_detected_ = true;
    if(!SD.begin(BUILTIN_SDCARD)){
        sd_detected_ = false;
        return;
    }

    if(sd_detected_){
        if (SD.exists(logger_file_name_)) {
            SD.remove(logger_file_name_);
        }

        File f = SD.open(logger_file_name_, FILE_WRITE);
        if (f) {
            f.println("time(ms),Raw Throtttle,VBat,CurrentA,CurrentB,CurrentC,RPMs");
            f.flush();
            file_handle_ = new File(f);
        } else {
            sd_detected_ = false;
        }
    }
}

void BLDC_Logger::logMotorDataSD()
{
    logMotorDataSD(this->data);
}

void BLDC_Logger::logMotorDataSD(const BLDC_Logger_Data &record)
{ 
    if(!sd_detected_ || !file_handle_){
        return;
    }

    File *f = static_cast<File*>(file_handle_);
    f->print(record.timestamp);
    f->print(',');
    f->print(record.raw_throttle);
    f->print(',');
    f->print(record.VBat);
    f->print(',');
    f->print(record.currentA, 3);
    f->print(',');
    f->print(record.currentB, 3);
    f->print(',');
    f->print(record.currentC, 3);
    f->print(',');
    f->println(record.rpm);

    sample_id_++;

    /* flush after N samples */
    if(sample_id_ >= kSamplesToFlushSD){
        f->flush();
        sample_id_ = 0;
    }
}
