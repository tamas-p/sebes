// Copyright 2016 Tamas Palagyi
#ifndef APP_SRC_DICOM_H_
#define APP_SRC_DICOM_H_

#include <limits.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include <sstream>
#include <set>

// Since DCMTK does not manage blocking select - it is not able to
// give a NULL timeout value to select (see ASC_associationWaiting
// source code), we need to force to a very long timeout that is
// 2147483647 seconds ~ 68 years on my x86_64.
#define TIMEOUT INT_MAX

// STUDY LEVEL
typedef std::string PatientID;
typedef std::string PatientName;
typedef std::string StudyUID;
typedef std::string StudyDesc;

// SERIES LEVEL
typedef std::string SeriesUID;
typedef std::string SeriesDesc;

// IMAGE LEVEL
typedef std::string SOPInstanceUID;
typedef std::string SOPClassUID;

typedef std::string TransferSyntaxUID;
typedef std::map<std::string, std::string> AetMap;
typedef std::set<std::string> Xfers;

class DcmDataset;

//------------------------------------------------------------------------------

#define THROW_DICOM_EXCEPTION(a) throw dicom_exception(a, 0, __FILE__, __LINE__);
class dicom_exception : public std::exception {
  std::string what_;
  unsigned int code_;
  std::string file_;
  int line_;
  std::string where_;

public:
  dicom_exception(const std::string& what, int code, const char* file, int line)
    : what_(what), code_(code), file_(file), line_(line) {
    std::ostringstream out;
    out << "Exception from " << file_ << ":" << line_;
    where_ = out.str();

    std::cerr << where_ << std::endl;
  }

  virtual const char* what() const throw() {
    return what_.c_str();
  }
  virtual unsigned int code() const throw() {
    return code_;
  }
  virtual const char* where() const throw() {
    return where_.c_str();
  }
};

//------------------------------------------------------------------------------

struct Image {
  std::string source_;

  TransferSyntaxUID transfer_syntax_uid_;
  
  // STUDY LEVEL
  PatientID patient_id_;
  PatientName patient_name_;
  StudyUID study_uid_;
  StudyDesc study_desc_;

  // SERIES LEVEL
  SeriesUID series_uid_;
  SeriesDesc series_desc_;

  // IMAGE LEVEL
  SOPInstanceUID sop_instance_uid_;
  SOPClassUID sop_class_uid_;

  void* dataset_;

  void init(std::string source,
            TransferSyntaxUID transfer_syntax_uid,

            // STUDY
            PatientID patient_id,
            PatientName patient_name,
            StudyUID study_uid,
            StudyDesc study_desc,

            // SERIES LEVEL
            SeriesUID series_uid,
            SeriesDesc series_desc,

            // IMAGE LEVEL
            SOPInstanceUID sop_instance_uid,
            SOPClassUID sop_class_uid) {
    source_ = source;
    transfer_syntax_uid_ = transfer_syntax_uid;

    // STUDY
    patient_id_ = patient_id;
    patient_name_ = patient_name;
    study_uid_ =  study_uid;
    study_desc_ = study_desc;

    // SERIES LEVEL
    series_uid_ = series_uid;
    series_desc_ = series_desc;

    // IMAGE LEVEL
    sop_instance_uid_ = sop_instance_uid;
    sop_class_uid_ = sop_class_uid;
  }
};

//------------------------------------------------------------------------------

class ImageStore;
class Dicom {

public:

  enum QueryLevel {IMAGE, SERIES, STUDY};
  
  const ImageStore* imagestore_;
  const AetMap* rhosts_;
  const std::string aet_;
  const std::string dicom_host_;
  const int dicom_port_;
  const int timeout_;
  const bool multi_;
  const bool need_response_;

  // FIXME: Maybe imagestore shall be only given to the movescp_execute
  explicit Dicom(int port,
                 const std::string& aet,
                 const ImageStore* imagestore,
                 const AetMap* rhosts,
                 bool multi,
                 bool need_response)
    : imagestore_(imagestore),
      rhosts_(rhosts),
      aet_(aet),
      dicom_port_(port),
      timeout_(TIMEOUT),
      multi_(multi),
      need_response_(need_response) {};

  explicit Dicom(const std::string& aet,
                 const std::string& host,
                 int port,
                 bool need_response)
    : imagestore_(0),
      rhosts_(0),
      aet_(aet),
      dicom_host_(host),
      dicom_port_(port),
      timeout_(TIMEOUT),
      multi_(false),
      need_response_(need_response) {};

  // Implementer must make sure all data is loaded into memory.
  // No leasy loading.
  static void load_image_file(const std::string& path, Image* image);
  
  virtual void init() = 0;
  
  virtual void waitfor_association() = 0;
  
  virtual void process_request() = 0;
  
  virtual void movescu_execute(const std::string& raet,
                               const std::string& raddress,
                               QueryLevel level,
                               const std::string& study_uid,
                               const std::string& series_uid,
                               const std::string& instance_uid,
                               const std::string& xfer) = 0;

  virtual void cleanup() = 0;
};

//------------------------------------------------------------------------------

#endif  // APP_SRC_DICOM_H_
